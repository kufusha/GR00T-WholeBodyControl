"""Stream the MuJoCo ego camera to XRoboToolkit on PICO.

The H.264 transport is adapted from G1-Head-Control's zed_pico_zmq.py:
https://github.com/Whole-Body-Control-Unitree-G1/G1-Head-Control
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from fractions import Fraction
import ipaddress
import logging
import queue
import re
import socket
import struct
import threading
import time
from typing import Iterable, Sequence

import cv2
import msgpack
import msgpack_numpy as msgpack_numpy
import numpy as np
import zmq

from gear_sonic.camera.sensor_server import ImageMessageSchema

_IPV4_PATTERN = re.compile(rb"(?:\d{1,3}\.){3}\d{1,3}")

EYE_WIDTH = 640
EYE_HEIGHT = 480
STEREO_WIDTH = EYE_WIDTH * 2
STEREO_HEIGHT = EYE_HEIGHT
MAX_CONTROL_MESSAGE_BYTES = 4096
MAX_CONTROL_CLIENTS = 8
LOGGER = logging.getLogger(__name__)


class ControlServerError(RuntimeError):
    """Raised after bridge shutdown when the control server fails."""


class PicoTargetStore:
    """Share the current PICO address between control and video threads."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._host: str | None = None
        self._generation = 0

    def set(self, host: str) -> int:
        with self._lock:
            self._generation += 1
            self._host = host
            return self._generation

    def clear(self, generation: int) -> None:
        with self._lock:
            if generation == self._generation:
                self._host = None

    def snapshot(self) -> tuple[str | None, int]:
        with self._lock:
            return self._host, self._generation


@dataclass(frozen=True)
class PicoStreamConfig:
    camera_host: str
    camera_port: int
    camera_key: str
    command_host: str
    command_port: int
    pico_camera_port: int
    fps: int
    bitrate: int


def positive_int(value: str) -> int:
    """Parse a strictly positive command-line integer."""
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be positive")
    return parsed


def tcp_port(value: str) -> int:
    """Parse a command-line TCP port in the inclusive range 1 through 65535."""
    parsed = int(value)
    if not 1 <= parsed <= 65535:
        raise argparse.ArgumentTypeError("TCP port must be between 1 and 65535")
    return parsed


def parse_args(argv: Sequence[str] | None = None) -> PicoStreamConfig:
    """Parse bridge arguments without changing the data-collection launcher."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-host", default="localhost")
    parser.add_argument("--camera-port", type=tcp_port, default=5555)
    parser.add_argument("--camera-key", default="ego_view")
    parser.add_argument("--command-host", default="0.0.0.0")
    parser.add_argument("--command-port", type=tcp_port, default=13579)
    parser.add_argument("--pico-camera-port", type=tcp_port, default=12345)
    parser.add_argument("--fps", type=positive_int, default=30)
    parser.add_argument("--bitrate", type=positive_int, default=1_000_000)
    return PicoStreamConfig(**vars(parser.parse_args(argv)))


class CameraFrameError(ValueError):
    """Raised when a camera message cannot provide a usable frame."""


def decode_camera_frame(raw_message: bytes, camera_key: str) -> np.ndarray:
    """Decode one image from the current GR00T camera message format."""
    try:
        payload = msgpack.unpackb(raw_message, object_hook=msgpack_numpy.decode)
        message = ImageMessageSchema.deserialize(payload)
        frame = message.images[camera_key]
    except (
        AttributeError,
        KeyError,
        TypeError,
        ValueError,
        cv2.error,
        msgpack.UnpackException,
    ) as error:
        raise CameraFrameError(f"Camera message has no usable '{camera_key}' frame") from error

    if not isinstance(frame, np.ndarray) or frame.ndim != 3 or frame.shape[2] != 3:
        raise CameraFrameError(f"Camera '{camera_key}' must be an HxWx3 image")
    if frame.shape[0] == 0 or frame.shape[1] == 0 or frame.dtype != np.uint8:
        raise CameraFrameError(f"Camera '{camera_key}' must be a nonempty uint8 image")
    return np.ascontiguousarray(frame)


def make_side_by_side(frame: np.ndarray) -> np.ndarray:
    """Duplicate a monoscopic RGB frame for the left and right eyes."""
    if frame.ndim != 3 or frame.shape[2] != 3:
        raise CameraFrameError("Side-by-side input must be an HxWx3 image")
    return np.ascontiguousarray(np.concatenate((frame, frame), axis=1))


def frame_h264_packet(payload: bytes) -> bytes:
    """Prefix one H.264 payload with XRoboToolkit's four-byte length field."""
    return struct.pack(">I", len(payload)) + payload


def parse_open_camera(data: bytes, peer_ip: str | None = None) -> str | None:
    """Return the requested PICO address from an XRoboToolkit control message."""
    if b"OPEN_CAMERA" not in data:
        return None

    candidates = [match.decode("ascii") for match in _IPV4_PATTERN.findall(data)]
    if peer_ip is not None:
        candidates.append(peer_ip)
    for candidate in candidates:
        try:
            address = ipaddress.ip_address(candidate)
        except ValueError:
            continue
        if address.version == 4:
            return str(address)
    return None


def extract_open_camera_request(data: bytes, peer_ip: str | None) -> str | None:
    """Parse one bounded control buffer when it contains an OPEN_CAMERA request."""
    if len(data) > MAX_CONTROL_MESSAGE_BYTES or b"OPEN_CAMERA" not in data:
        return None
    return parse_open_camera(data, peer_ip)


def send_h264_packets(sock: socket.socket, packets: Iterable[bytes]) -> None:
    """Send encoder packets using XRoboToolkit's length-prefixed framing."""
    for packet in packets:
        sock.sendall(frame_h264_packet(packet))


class H264Encoder:
    """Encode fixed-size RGB frames as low-latency H.264 baseline packets."""

    def __init__(self, width: int, height: int, fps: int, bitrate: int) -> None:
        import av

        self._av = av
        self._codec = av.CodecContext.create("libx264", "w")
        self._codec.width = width
        self._codec.height = height
        self._codec.time_base = Fraction(1, fps)
        self._codec.framerate = Fraction(fps, 1)
        self._codec.pix_fmt = "yuv420p"
        self._codec.bit_rate = bitrate
        self._codec.options = {
            "preset": "ultrafast",
            "tune": "zerolatency",
            "profile": "baseline",
            "repeat-headers": "1",
        }
        self._codec.open()

    def encode(self, rgb_frame: np.ndarray) -> list[bytes]:
        frame = self._av.VideoFrame.from_ndarray(rgb_frame, format="rgb24")
        return [bytes(packet) for packet in self._codec.encode(frame)]

    def close(self) -> None:
        self._codec.encode(None)


def _handle_control_client(
    client: socket.socket, peer_ip: str, targets: PicoTargetStore, stop_event: threading.Event
) -> None:
    """Accumulate one request, then retain its target until the client disconnects."""
    client.settimeout(0.1)
    buffer = bytearray()
    generation = None
    try:
        while not stop_event.is_set():
            try:
                chunk = client.recv(MAX_CONTROL_MESSAGE_BYTES + 1)
            except socket.timeout:
                if generation is not None or not buffer:
                    continue
                host = extract_open_camera_request(bytes(buffer), peer_ip)
                if host is None:
                    LOGGER.warning("Ignoring malformed control request from %s", peer_ip)
                    return
                generation = targets.set(host)
                LOGGER.info("PICO target: %s", host)
                buffer.clear()
                continue
            if not chunk:
                return
            if generation is None:
                buffer.extend(chunk)
                if len(buffer) > MAX_CONTROL_MESSAGE_BYTES:
                    LOGGER.warning("Ignoring oversized control request from %s", peer_ip)
                    return
    except OSError as error:
        LOGGER.info("Control client %s disconnected: %s", peer_ip, error)
    finally:
        if generation is not None:
            targets.clear(generation)
            LOGGER.info("PICO control disconnected: %s", peer_ip)


def run_control_server(
    config: PicoStreamConfig,
    targets: PicoTargetStore,
    stop_event: threading.Event,
    errors: queue.SimpleQueue[Exception] | None = None,
) -> None:
    """Serve bounded XRoboToolkit requests until shutdown."""
    clients = []

    def serve_client(client: socket.socket, peer_ip: str) -> None:
        with client:
            _handle_control_client(client, peer_ip, targets, stop_event)

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind((config.command_host, config.command_port))
            listener.listen()
            listener.settimeout(0.5)
            LOGGER.info("Control listening on %s:%s", config.command_host, config.command_port)
            while not stop_event.is_set():
                try:
                    client, address = listener.accept()
                except socket.timeout:
                    continue
                clients = [worker for worker in clients if worker.is_alive()]
                if len(clients) >= MAX_CONTROL_CLIENTS:
                    client.close()
                    LOGGER.warning("Too many control clients; rejecting %s", address[0])
                    continue
                try:
                    worker = threading.Thread(
                        target=serve_client,
                        args=(client, address[0]),
                        name="xrobotoolkit-control-client",
                        daemon=True,
                    )
                    worker.start()
                except (OSError, RuntimeError):
                    client.close()
                    raise
                clients.append(worker)
    except (OSError, RuntimeError) as error:
        LOGGER.error("Control server failed: %s", error)
        if errors is not None:
            errors.put(error)
        stop_event.set()
    finally:
        for worker in clients:
            worker.join(timeout=0.5)


def _close_video_connection(sock: socket.socket | None, encoder: H264Encoder | None) -> None:
    """Release only the resources owned by the current video connection."""
    try:
        if sock is not None:
            sock.close()
    finally:
        if encoder is not None:
            encoder.close()


def run_video_loop(
    config: PicoStreamConfig,
    targets: PicoTargetStore,
    subscriber: zmq.Socket,
    stop_event: threading.Event,
) -> None:
    """Forward current camera frames to the requested PICO with bounded retries."""
    video_socket = None
    encoder = None
    active_target = (None, 0)
    last_warning = float("-inf")
    resize_logged = False
    try:
        while not stop_event.is_set():
            requested_target = targets.snapshot()
            if requested_target != active_target:
                _close_video_connection(video_socket, encoder)
                video_socket = encoder = None
                active_target = requested_target
            if not subscriber.poll(100):
                continue
            message = subscriber.recv()
            requested_target = targets.snapshot()
            if requested_target != active_target:
                _close_video_connection(video_socket, encoder)
                video_socket = encoder = None
                active_target = requested_target
            host, _ = requested_target
            if host is None or stop_event.is_set():
                continue
            try:
                frame = decode_camera_frame(message, config.camera_key)
            except CameraFrameError as error:
                now = time.monotonic()
                if now - last_warning >= 5.0:
                    LOGGER.warning("Camera decode failed: %s", error)
                    last_warning = now
                continue
            if frame.shape[:2] != (EYE_HEIGHT, EYE_WIDTH):
                if not resize_logged:
                    LOGGER.info("Resizing camera from %s to 640x480", frame.shape[:2])
                    resize_logged = True
                frame = cv2.resize(frame, (EYE_WIDTH, EYE_HEIGHT), interpolation=cv2.INTER_AREA)
            try:
                if video_socket is None:
                    video_socket = socket.create_connection(
                        (host, config.pico_camera_port), timeout=1.0
                    )
                    encoder = H264Encoder(STEREO_WIDTH, STEREO_HEIGHT, config.fps, config.bitrate)
                    LOGGER.info("Video connected to %s:%s", host, config.pico_camera_port)
                if targets.snapshot() != requested_target:
                    continue
                send_h264_packets(video_socket, encoder.encode(make_side_by_side(frame)))
            except OSError as error:
                LOGGER.info("Video disconnected from %s; reconnecting: %s", host, error)
                _close_video_connection(video_socket, encoder)
                video_socket = encoder = None
                if targets.snapshot() == requested_target:
                    stop_event.wait(1.0)
    finally:
        _close_video_connection(video_socket, encoder)


def run_bridge(config: PicoStreamConfig) -> None:
    """Own the control thread and latest-frame camera subscriber."""
    stop_event = threading.Event()
    targets = PicoTargetStore()
    control_errors: queue.SimpleQueue[Exception] = queue.SimpleQueue()
    control_thread = threading.Thread(
        target=run_control_server,
        args=(config, targets, stop_event, control_errors),
        name="xrobotoolkit-control",
        daemon=True,
    )
    control_thread.start()
    context = None
    subscriber = None
    try:
        context = zmq.Context()
        subscriber = context.socket(zmq.SUB)
        subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
        subscriber.setsockopt(zmq.CONFLATE, 1)
        subscriber.setsockopt(zmq.LINGER, 0)
        endpoint = f"tcp://{config.camera_host}:{config.camera_port}"
        subscriber.connect(endpoint)
        LOGGER.info("Camera endpoint: %s (%s)", endpoint, config.camera_key)
        run_video_loop(config, targets, subscriber, stop_event)
    finally:
        stop_event.set()
        if subscriber is not None:
            subscriber.close()
        if context is not None:
            context.term()
        control_thread.join(timeout=2.0)
    try:
        error = control_errors.get_nowait()
    except queue.Empty:
        return
    raise ControlServerError(f"Control server failed: {error}") from error


def main(argv: Sequence[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    config = parse_args(argv)
    try:
        run_bridge(config)
    except KeyboardInterrupt:
        LOGGER.info("Stopping XRoboToolkit ego stream")
    except ControlServerError:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
