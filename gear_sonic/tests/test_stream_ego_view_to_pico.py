from fractions import Fraction
import queue
import socket
import threading

import av
import cv2
import msgpack
import msgpack_numpy
import numpy as np
import pytest
import zmq

from gear_sonic.camera.sensor_server import ImageMessageSchema
from gear_sonic.scripts import stream_ego_view_to_pico as bridge
from gear_sonic.scripts.stream_ego_view_to_pico import (
    CameraFrameError,
    H264Encoder,
    PicoTargetStore,
    decode_camera_frame,
    extract_open_camera_request,
    frame_h264_packet,
    main,
    make_side_by_side,
    parse_args,
    parse_open_camera,
    run_bridge,
    run_control_server,
    run_video_loop,
    send_h264_packets,
)


def test_pico_target_store_replaces_target_and_ignores_stale_disconnect():
    store = PicoTargetStore()
    first = store.set("192.168.0.10")
    second = store.set("192.168.0.11")

    store.clear(first)

    assert store.snapshot() == ("192.168.0.11", second)
    assert second > first


def test_pico_target_store_clears_current_connection():
    store = PicoTargetStore()
    generation = store.set("192.168.0.10")

    store.clear(generation)

    assert store.snapshot() == (None, generation)


class FakeSocket:
    def __init__(self):
        self.sent = []

    def sendall(self, data: bytes) -> None:
        self.sent.append(data)


def test_extract_open_camera_request_waits_for_fragmented_address():
    assert extract_open_camera_request(b"OPEN_CAM", "192.168.0.20") is None
    assert extract_open_camera_request(b"OPEN_CAMERA 192.168.", None) is None
    assert extract_open_camera_request(b"OPEN_CAMERA 192.168.0.20", None) == "192.168.0.20"


def test_extract_open_camera_request_rejects_oversized_buffer():
    assert extract_open_camera_request(b"OPEN_CAMERA " + b"x" * 4096, "192.168.0.20") is None
    assert extract_open_camera_request(b"OPEN_CAMERA" + b" " * 4085, "192.168.0.20") == (
        "192.168.0.20"
    )


def test_send_h264_packets_frames_each_encoder_packet():
    sock = FakeSocket()

    send_h264_packets(sock, [b"first", b"second"])

    assert sock.sent == [b"\x00\x00\x00\x05first", b"\x00\x00\x00\x06second"]


def test_h264_encoder_outputs_decodable_baseline_frame_without_delay():
    encoder = H264Encoder(1280, 480, 30, 1_000_000)
    rgb = np.zeros((480, 1280, 3), dtype=np.uint8)
    rgb[:, :, 0] = 180
    try:
        packets = encoder.encode(rgb)
    finally:
        encoder.close()

    decoder = av.CodecContext.create("h264", "r")
    decoded = [frame for packet in packets for frame in decoder.decode(av.Packet(packet))]
    assert len(decoded) == 1
    assert decoder.profile in ("Baseline", "Constrained Baseline")
    assert (decoded[0].width, decoded[0].height) == (1280, 480)
    assert np.mean(decoded[0].to_ndarray(format="rgb24")[:, :, 0]) > 170
    assert encoder._codec.time_base == Fraction(1, 30)
    assert encoder._codec.framerate == Fraction(30, 1)


class ScriptedClient(FakeSocket):
    def __init__(self, chunks, targets):
        super().__init__()
        self.chunks = iter(chunks)
        self.targets = targets
        self.observed = []
        self.closed = False
        self.closed_event = threading.Event()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.closed = True
        self.closed_event.set()

    def settimeout(self, timeout):
        self.timeout = timeout

    def recv(self, size):
        self.observed.append(self.targets.snapshot())
        chunk = next(self.chunks)
        if isinstance(chunk, Exception):
            raise chunk
        return chunk


class ScriptedListener:
    def __init__(self, clients, stop_event):
        self.clients = iter(clients)
        self.stop_event = stop_event
        self.closed = False
        self.options = []
        self.accepted = []

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.closed = True

    def setsockopt(self, *option):
        self.options.append(option)

    def bind(self, endpoint):
        self.endpoint = endpoint

    def listen(self):
        pass

    def settimeout(self, timeout):
        self.timeout = timeout

    def accept(self):
        try:
            client = next(self.clients)
        except StopIteration:
            for client in self.accepted:
                assert client.closed_event.wait(1.0)
            self.stop_event.set()
            raise socket.timeout
        self.accepted.append(client)
        return client, ("192.168.0.77", 50000)


def test_control_server_waits_for_quiet_buffer_and_clears_disconnect(monkeypatch, caplog):
    targets = PicoTargetStore()
    stop = threading.Event()
    client = ScriptedClient(
        [b"OPEN_CAMERA 192.168.0.2", b"0", socket.timeout(), socket.timeout(), b""], targets
    )
    listener = ScriptedListener([client], stop)
    monkeypatch.setattr(socket, "socket", lambda *args: listener)

    run_control_server(parse_args([]), targets, stop)

    assert client.observed[:3] == [(None, 0), (None, 0), (None, 0)]
    assert client.observed[3:] == [("192.168.0.20", 1), ("192.168.0.20", 1)]
    assert targets.snapshot() == (None, 1)
    assert listener.endpoint == ("0.0.0.0", 13579)
    assert listener.options == [(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)]
    assert listener.timeout == 0.5
    assert client.timeout == 0.1
    assert client.closed and listener.closed
    assert sum("PICO target: 192.168.0.20" in r.message for r in caplog.records) == 1


@pytest.mark.parametrize("request_bytes", [b"CLOSE_CAMERA", b"OPEN_CAMERA" + b"x" * 4096])
def test_control_server_recovers_after_bad_request(monkeypatch, caplog, request_bytes):
    targets = PicoTargetStore()
    stop = threading.Event()
    bad = ScriptedClient([request_bytes, socket.timeout(), b""], targets)
    good = ScriptedClient([b"OPEN_CAMERA", socket.timeout(), b""], targets)
    listener = ScriptedListener([bad, good], stop)
    monkeypatch.setattr(socket, "socket", lambda *args: listener)

    run_control_server(parse_args([]), targets, stop)

    assert targets.snapshot() == (None, 1)
    assert good.observed[-1] == ("192.168.0.77", 1)
    assert bad.closed and good.closed and listener.closed
    assert any(r.levelname == "WARNING" for r in caplog.records)


def test_control_server_clears_target_on_connection_error(monkeypatch):
    targets = PicoTargetStore()
    stop = threading.Event()
    client = ScriptedClient([b"OPEN_CAMERA", socket.timeout(), ConnectionResetError()], targets)
    listener = ScriptedListener([client], stop)
    monkeypatch.setattr(socket, "socket", lambda *args: listener)

    run_control_server(parse_args([]), targets, stop)

    assert targets.snapshot() == (None, 1)
    assert client.closed and listener.closed


def test_control_server_replaces_live_client_and_preserves_target_on_stale_disconnect(monkeypatch):
    targets = PicoTargetStore()
    stop = threading.Event()

    class LiveClient(ScriptedClient):
        def __init__(self, host):
            super().__init__([f"OPEN_CAMERA {host}".encode(), socket.timeout()], targets)
            self.incoming = queue.Queue()
            self.registered = threading.Event()

        def recv(self, size):
            try:
                return super().recv(size)
            except StopIteration:
                self.registered.set()
                try:
                    return self.incoming.get(timeout=self.timeout)
                except queue.Empty:
                    raise socket.timeout from None

    class LiveListener(ScriptedListener):
        def __init__(self):
            super().__init__([], stop)
            self.incoming = queue.Queue()

        def accept(self):
            try:
                return self.incoming.get(timeout=0.1), ("192.168.0.77", 50000)
            except queue.Empty:
                raise socket.timeout from None

    first, second = LiveClient("192.168.0.10"), LiveClient("192.168.0.11")
    listener = LiveListener()
    listener.incoming.put(first)
    monkeypatch.setattr(socket, "socket", lambda *args: listener)
    server = threading.Thread(target=run_control_server, args=(parse_args([]), targets, stop))
    server.start()
    try:
        assert first.registered.wait(1.0)
        assert targets.snapshot() == ("192.168.0.10", 1)
        listener.incoming.put(second)

        assert second.registered.wait(1.0), "A live old client blocked the replacement request"
        assert not first.closed
        assert targets.snapshot() == ("192.168.0.11", 2)

        first.incoming.put(b"")
        assert first.closed_event.wait(1.0)
        assert targets.snapshot() == ("192.168.0.11", 2)

        second.incoming.put(b"")
        assert second.closed_event.wait(1.0)
        assert targets.snapshot() == (None, 2)
    finally:
        stop.set()
        first.incoming.put(b"")
        second.incoming.put(b"")
        server.join(timeout=2.0)
    assert not server.is_alive() and listener.closed


class FakeStopEvent(threading.Event):
    def __init__(self):
        super().__init__()
        self.waits = []

    def wait(self, timeout=None):
        self.waits.append(timeout)
        return self.is_set()


class ScriptedSubscriber:
    def __init__(self, messages, stop):
        self.messages = iter(messages)
        self.stop = stop
        self.polls = []
        self.received = 0

    def poll(self, timeout):
        self.polls.append(timeout)
        message = next(self.messages, self.stop.set)
        if callable(message):
            message = message()
        self.message = message
        return zmq.POLLIN if message is not None else 0

    def recv(self):
        self.received += 1
        return self.message


class VideoSocket(FakeSocket):
    def __init__(self, error=None):
        super().__init__()
        self.error = error
        self.closed = False

    def sendall(self, data):
        if self.error:
            raise self.error
        super().sendall(data)

    def close(self):
        self.closed = True


def decode_video_socket(sock):
    decoder = av.CodecContext.create("h264", "r")
    frames = []
    for packet in sock.sent:
        assert int.from_bytes(packet[:4], "big") == len(packet) - 4
        frames.extend(decoder.decode(av.Packet(packet[4:])))
    return [frame.to_ndarray(format="rgb24") for frame in frames]


def install_video_connections(monkeypatch, sockets):
    connections = []
    remaining = iter(sockets)

    def connect(endpoint, timeout):
        connections.append((endpoint, timeout))
        result = next(remaining)
        if isinstance(result, Exception):
            raise result
        return result

    monkeypatch.setattr(socket, "create_connection", connect)
    return connections


def test_video_loop_drains_camera_without_connecting_before_request(monkeypatch):
    targets = PicoTargetStore()
    stop = FakeStopEvent()
    subscriber = ScriptedSubscriber([b"discard this frame"], stop)
    connections = install_video_connections(monkeypatch, [])

    run_video_loop(parse_args([]), targets, subscriber, stop)

    assert connections == []
    assert subscriber.received == 1
    assert all(0 < timeout <= 500 for timeout in subscriber.polls)


def test_video_loop_resizes_selected_camera_and_streams_rgb_stereo(monkeypatch, caplog):
    targets = PicoTargetStore()
    targets.set("192.168.0.20")
    stop = FakeStopEvent()
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    frame[:, :, 0] = 180
    message = pack_camera_message({"chosen": frame})
    subscriber = ScriptedSubscriber([message, message], stop)
    sock = VideoSocket()
    connections = install_video_connections(monkeypatch, [sock])
    resize = cv2.resize
    resize_modes = []

    def record_resize(*args, **kwargs):
        resize_modes.append(kwargs["interpolation"])
        return resize(*args, **kwargs)

    monkeypatch.setattr(cv2, "resize", record_resize)

    run_video_loop(parse_args(["--camera-key", "chosen"]), targets, subscriber, stop)

    frames = decode_video_socket(sock)
    assert len(frames) == 2
    assert frames[0].shape == (480, 1280, 3)
    assert np.mean(frames[0][:, :, 0]) > 170
    np.testing.assert_allclose(frames[0][:, :640], frames[0][:, 640:], atol=2)
    assert connections == [(("192.168.0.20", 12345), 1.0)]
    assert resize_modes == [cv2.INTER_AREA, cv2.INTER_AREA]
    assert sum("Resizing camera" in r.message for r in caplog.records) == 1
    assert sock.closed


def test_video_loop_recreates_connection_for_same_host_new_generation(monkeypatch):
    targets = PicoTargetStore()
    targets.set("192.168.0.20")
    stop = FakeStopEvent()
    message = pack_camera_message({"ego_view": np.zeros((480, 640, 3), dtype=np.uint8)})

    def replace_target():
        targets.set("192.168.0.20")
        return message

    subscriber = ScriptedSubscriber([message, replace_target], stop)
    first, second = VideoSocket(), VideoSocket()
    connections = install_video_connections(monkeypatch, [first, second])

    run_video_loop(parse_args([]), targets, subscriber, stop)

    assert len(connections) == 2
    assert len(decode_video_socket(first)) == len(decode_video_socket(second)) == 1
    assert first.closed and second.closed


def test_video_loop_drops_connection_when_target_disconnects(monkeypatch):
    targets = PicoTargetStore()
    generation = targets.set("192.168.0.20")
    stop = FakeStopEvent()
    message = pack_camera_message({"ego_view": np.zeros((480, 640, 3), dtype=np.uint8)})

    def disconnect():
        targets.clear(generation)
        return message

    subscriber = ScriptedSubscriber([message, disconnect], stop)
    sock = VideoSocket()
    connections = install_video_connections(monkeypatch, [sock])

    run_video_loop(parse_args([]), targets, subscriber, stop)

    assert len(connections) == 1
    assert len(decode_video_socket(sock)) == 1
    assert sock.closed


@pytest.mark.parametrize("failure", [ConnectionRefusedError(), VideoSocket(BrokenPipeError())])
def test_video_loop_retries_transport_failure_with_fresh_headers(monkeypatch, failure):
    targets = PicoTargetStore()
    targets.set("192.168.0.20")
    stop = FakeStopEvent()
    message = pack_camera_message({"ego_view": np.zeros((480, 640, 3), dtype=np.uint8)})
    subscriber = ScriptedSubscriber([message, message], stop)
    recovered = VideoSocket()
    connections = install_video_connections(monkeypatch, [failure, recovered])

    run_video_loop(parse_args([]), targets, subscriber, stop)

    assert len(connections) == 2
    assert stop.waits == [1.0]
    assert targets.snapshot() == ("192.168.0.20", 1)
    assert len(decode_video_socket(recovered)) == 1
    assert recovered.closed
    if isinstance(failure, VideoSocket):
        assert failure.closed


def test_video_loop_rate_limits_malformed_camera_warnings(monkeypatch, caplog):
    targets = PicoTargetStore()
    targets.set("192.168.0.20")
    stop = FakeStopEvent()
    subscriber = ScriptedSubscriber([b"malformed"] * 3, stop)
    install_video_connections(monkeypatch, [])
    times = iter([0.0, 1.0, 6.0])
    monkeypatch.setattr(bridge.time, "monotonic", lambda: next(times))

    run_video_loop(parse_args([]), targets, subscriber, stop)

    assert sum("Camera decode failed" in r.message for r in caplog.records) == 2


class BridgeSubscriber(ScriptedSubscriber):
    def __init__(self, messages, stop, connect_error=None):
        super().__init__(messages, stop)
        self.options = []
        self.closed = False
        self.connect_error = connect_error

    def setsockopt_string(self, *args):
        self.options.append(args)

    def setsockopt(self, *args):
        self.options.append(args)

    def connect(self, endpoint):
        self.endpoint = endpoint
        if self.connect_error:
            raise self.connect_error

    def close(self):
        self.closed = True


class BridgeContext:
    def __init__(self, subscriber):
        self.subscriber = subscriber
        self.terminated = False

    def socket(self, kind):
        assert kind == zmq.SUB
        return self.subscriber

    def term(self):
        self.terminated = True


def install_bridge_resources(monkeypatch, messages=(), connect_error=None):
    threads = []
    subscriber = BridgeSubscriber(messages, FakeStopEvent(), connect_error)
    context = BridgeContext(subscriber)

    class ControlThread:
        def __init__(self, *, target, args, name, daemon):
            self.target = target
            self.args = args
            self.name = name
            self.daemon = daemon
            self.started = False
            self.joined = False
            subscriber.stop = args[2]
            threads.append(self)

        def start(self):
            self.started = True

        def join(self, timeout):
            self.joined = timeout

    monkeypatch.setattr(threading, "Thread", ControlThread)
    monkeypatch.setattr(zmq, "Context", lambda: context)
    return subscriber, context, threads


def test_run_bridge_configures_latest_frame_subscriber_and_closes_resources(monkeypatch):
    subscriber, context, threads = install_bridge_resources(monkeypatch)
    connections = install_video_connections(monkeypatch, [])

    run_bridge(parse_args(["--camera-host", "camera", "--camera-port", "6000"]))

    assert subscriber.endpoint == "tcp://camera:6000"
    assert subscriber.options == [(zmq.SUBSCRIBE, ""), (zmq.CONFLATE, 1), (zmq.LINGER, 0)]
    assert subscriber.closed and context.terminated
    assert threads[0].target is run_control_server
    assert threads[0].started and threads[0].joined == 2.0
    assert threads[0].name == "xrobotoolkit-control" and threads[0].daemon
    assert threads[0].args[2].is_set()
    assert connections == []


def test_run_bridge_closes_resources_when_subscriber_setup_fails(monkeypatch):
    subscriber, context, threads = install_bridge_resources(
        monkeypatch, connect_error=OSError("setup failed")
    )

    with pytest.raises(OSError, match="setup failed"):
        run_bridge(parse_args([]))

    assert subscriber.closed and context.terminated
    assert threads[0].args[2].is_set() and threads[0].joined == 2.0


def test_main_handles_interrupt_and_closes_bridge_resources(monkeypatch, caplog):
    def interrupt():
        raise KeyboardInterrupt

    subscriber, context, threads = install_bridge_resources(monkeypatch, [interrupt])

    assert main([]) == 0

    assert subscriber.closed and context.terminated
    assert threads[0].args[2].is_set() and threads[0].joined == 2.0
    assert any("Stopping XRoboToolkit ego stream" in r.message for r in caplog.records)


@pytest.mark.parametrize("payload", [[], 7, {"images": []}, {"images": {"ego_view": b""}}])
def test_decode_camera_frame_wraps_malformed_camera_payload(payload):
    with pytest.raises(CameraFrameError):
        decode_camera_frame(msgpack.packb(payload, use_bin_type=True), "ego_view")


def test_control_server_stops_bridge_when_listen_endpoint_is_unavailable(monkeypatch, caplog):
    targets = PicoTargetStore()
    stop = threading.Event()
    listener = ScriptedListener([], stop)

    def fail_bind(endpoint):
        raise OSError("address in use")

    listener.bind = fail_bind
    monkeypatch.setattr(socket, "socket", lambda *args: listener)

    run_control_server(parse_args([]), targets, stop)

    assert stop.is_set() and listener.closed
    assert any("address in use" in r.message and r.levelname == "ERROR" for r in caplog.records)


MALFORMED_ARRAYS = [
    np.empty((0, 640, 3), dtype=np.uint8),
    np.empty((480, 0, 3), dtype=np.uint8),
    np.zeros((480, 640, 3), dtype=np.uint16),
    np.zeros((480, 640, 3), dtype=np.float32),
    np.zeros((480, 640, 3), dtype=np.bool_),
    np.zeros((480, 640, 3), dtype=np.complex64),
]


def pack_numpy_camera_message(frame):
    return msgpack.packb(
        {"timestamps": {"ego_view": 1.0}, "images": {"ego_view": frame}},
        default=msgpack_numpy.encode,
        use_bin_type=True,
    )


@pytest.mark.parametrize("frame", MALFORMED_ARRAYS)
def test_decode_camera_frame_rejects_empty_or_unsupported_numpy_images(frame):
    with pytest.raises(CameraFrameError):
        decode_camera_frame(pack_numpy_camera_message(frame), "ego_view")


@pytest.mark.parametrize("frame", MALFORMED_ARRAYS)
def test_video_loop_continues_after_empty_or_unsupported_numpy_image(monkeypatch, caplog, frame):
    targets = PicoTargetStore()
    targets.set("192.168.0.20")
    stop = FakeStopEvent()
    valid = np.zeros((480, 640, 3), dtype=np.uint8)
    valid[:, :, 0] = 180
    subscriber = ScriptedSubscriber(
        [pack_numpy_camera_message(frame), pack_numpy_camera_message(valid)], stop
    )
    sock = VideoSocket()
    connections = install_video_connections(monkeypatch, [sock])

    run_video_loop(parse_args([]), targets, subscriber, stop)

    frames = decode_video_socket(sock)
    assert len(frames) == 1 and frames[0].shape == (480, 1280, 3)
    assert np.mean(frames[0][:, :, 0]) > 170
    assert connections == [(("192.168.0.20", 12345), 1.0)]
    assert any("Camera decode failed" in r.message for r in caplog.records)
    assert sock.closed


def pack_camera_message(images: dict[str, np.ndarray]) -> bytes:
    message = ImageMessageSchema(timestamps={"ego_view": 1.0}, images=images)
    return msgpack.packb(message.serialize(), use_bin_type=True)


def test_decode_camera_frame_reads_current_gr00t_payload():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    frame[:, :, 0] = 23

    decoded = decode_camera_frame(pack_camera_message({"ego_view": frame}), "ego_view")

    assert decoded.shape == (480, 640, 3)
    assert np.allclose(decoded[:, :, 0], 23, atol=2)


def test_decode_camera_frame_rejects_missing_camera_key():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    with pytest.raises(CameraFrameError, match="ego_view"):
        decode_camera_frame(pack_camera_message({"head": frame}), "ego_view")


def test_make_side_by_side_duplicates_mono_frame():
    frame = np.arange(480 * 640 * 3, dtype=np.uint8).reshape(480, 640, 3)

    stereo = make_side_by_side(frame)

    assert stereo.shape == (480, 1280, 3)
    np.testing.assert_array_equal(stereo[:, :640], frame)
    np.testing.assert_array_equal(stereo[:, 640:], frame)


def test_frame_h264_packet_uses_big_endian_length_prefix():
    payload = b"\x00\x00\x00\x01\x67encoded"

    packet = frame_h264_packet(payload)

    assert packet[:4] == len(payload).to_bytes(4, "big")
    assert packet[4:] == payload


@pytest.mark.parametrize(
    ("request_bytes", "peer_ip", "expected"),
    [
        (b"OPEN_CAMERA 192.168.0.91", None, "192.168.0.91"),
        (b"prefix OPEN_CAMERA:10.0.0.42 suffix", None, "10.0.0.42"),
        (b"OPEN_CAMERA", "192.168.0.77", "192.168.0.77"),
        (b"CLOSE_CAMERA 192.168.0.91", None, None),
        (b"OPEN_CAMERA 999.1.2.3", None, None),
    ],
)
def test_parse_open_camera(request_bytes, peer_ip, expected):
    assert parse_open_camera(request_bytes, peer_ip) == expected


def test_cli_defaults_match_mujoco_profile():
    config = parse_args([])

    assert config.camera_host == "localhost"
    assert config.camera_port == 5555
    assert config.camera_key == "ego_view"
    assert config.command_host == "0.0.0.0"
    assert config.command_port == 13579
    assert config.pico_camera_port == 12345
    assert config.fps == 30
    assert config.bitrate == 1_000_000


@pytest.mark.parametrize(
    ("flag", "value"),
    [
        ("--camera-port", "0"),
        ("--command-port", "-1"),
        ("--pico-camera-port", "0"),
        ("--fps", "0"),
        ("--bitrate", "-1"),
    ],
)
def test_cli_rejects_non_positive_numbers(flag, value):
    with pytest.raises(SystemExit):
        parse_args([flag, value])
