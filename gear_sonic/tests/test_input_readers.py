import numpy as np

from gear_sonic.utils.teleop.input_readers import _body_data_to_24x7


def test_body_data_to_24x7_converts_wire_format():
    payload = {
        "joint_positions": [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]],
        "joint_orientations": [[0.0, 0.0, 0.0, 1.0], [0.5, 0.5, 0.5, 0.5]],
    }

    body_poses = _body_data_to_24x7(payload)

    assert body_poses is not None
    assert body_poses.shape == (24, 7)
    np.testing.assert_allclose(
        body_poses[0],
        np.array([1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]),
    )
    np.testing.assert_allclose(
        body_poses[1],
        np.array([4.0, 5.0, 6.0, 0.5, 0.5, 0.5, 0.5]),
    )
    np.testing.assert_array_equal(body_poses[2:], np.zeros((22, 7)))


def test_body_data_to_24x7_rejects_empty_wire_format():
    assert _body_data_to_24x7({"joint_positions": [], "joint_orientations": []}) is None
