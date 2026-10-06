import msgpack
import numpy as np
import pytest

from gear_sonic.camera.sensor_server import ImageMessageSchema
from gear_sonic.scripts.stream_ego_view_to_pico import (
    CameraFrameError,
    decode_camera_frame,
    frame_h264_packet,
    make_side_by_side,
    parse_args,
    parse_open_camera,
)


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
