"""Stream the MuJoCo ego camera to XRoboToolkit on PICO.

The H.264 transport is adapted from G1-Head-Control's zed_pico_zmq.py:
https://github.com/Whole-Body-Control-Unitree-G1/G1-Head-Control
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import ipaddress
import re
import struct
from typing import Sequence

import msgpack
import msgpack_numpy as msgpack_numpy
import numpy as np

from gear_sonic.camera.sensor_server import ImageMessageSchema

_IPV4_PATTERN = re.compile(rb"(?:\d{1,3}\.){3}\d{1,3}")

EYE_WIDTH = 640
EYE_HEIGHT = 480
STEREO_WIDTH = EYE_WIDTH * 2
STEREO_HEIGHT = EYE_HEIGHT


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


def parse_args(argv: Sequence[str] | None = None) -> PicoStreamConfig:
    """Parse bridge arguments without changing the data-collection launcher."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-host", default="localhost")
    parser.add_argument("--camera-port", type=positive_int, default=5555)
    parser.add_argument("--camera-key", default="ego_view")
    parser.add_argument("--command-host", default="0.0.0.0")
    parser.add_argument("--command-port", type=positive_int, default=13579)
    parser.add_argument("--pico-camera-port", type=positive_int, default=12345)
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
    except (KeyError, TypeError, ValueError, msgpack.UnpackException) as error:
        raise CameraFrameError(f"Camera message has no usable '{camera_key}' frame") from error

    if not isinstance(frame, np.ndarray) or frame.ndim != 3 or frame.shape[2] != 3:
        raise CameraFrameError(f"Camera '{camera_key}' must be an HxWx3 image")
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
