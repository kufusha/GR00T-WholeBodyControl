from pathlib import Path

import mujoco
import numpy as np
import pytest

from gear_sonic.utils.mujoco_sim.inspire_sim_mapping import (
    actuator_indices_for_joints,
    control_array_to_normalized,
    normalized_array_to_control,
    pack_inspire_state_positions,
    pack_inspire_state_velocities,
    split_inspire_command_positions,
)


ROOT = Path(__file__).resolve().parents[2]
MODEL_DIR = ROOT / "gear_sonic/data/robot_model/model_data/g1"


def test_inspire_command_order_is_right_then_left_on_dds():
    command = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 0.95]
    left, right = split_inspire_command_positions(command, 6)

    np.testing.assert_array_equal(left, [0.6, 0.7, 0.8, 0.9, 1.0, 0.95])
    np.testing.assert_array_equal(right, [0.0, 0.1, 0.2, 0.3, 0.4, 0.5])


def test_short_inspire_command_falls_back_to_open_hands():
    left, right = split_inspire_command_positions([0.0] * 11, 6)

    np.testing.assert_array_equal(left, np.ones(6))
    np.testing.assert_array_equal(right, np.ones(6))


def test_non_finite_inspire_command_values_fall_back_to_open():
    command = np.zeros(12)
    command[[0, 7, 11]] = [np.nan, np.inf, -np.inf]

    left, right = split_inspire_command_positions(command, 6)

    np.testing.assert_array_equal(right, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    np.testing.assert_array_equal(left, [0.0, 1.0, 0.0, 0.0, 0.0, 1.0])


def test_inspire_state_order_is_right_then_left_on_dds():
    ordered = pack_inspire_state_positions(
        left=[0.6, 0.7, 0.8],
        right=[0.0, 0.1, 0.2],
    )

    np.testing.assert_array_equal(ordered, [0.0, 0.1, 0.2, 0.6, 0.7, 0.8])


def test_inspire_state_velocity_order_preserves_signed_values():
    ordered = pack_inspire_state_velocities(
        left=[-2.0, 0.5, np.nan],
        right=[1.5, -0.25, np.inf],
    )

    np.testing.assert_array_equal(ordered, [1.5, -0.25, 0.0, -2.0, 0.5, 0.0])


def test_normalized_arrays_map_open_and_closed_to_actuator_ranges():
    ranges = np.array([[0.0, 1.47], [0.1, 0.57], [0.0, 1.308]])

    np.testing.assert_allclose(normalized_array_to_control(np.ones(3), ranges), ranges[:, 0])
    np.testing.assert_allclose(normalized_array_to_control(np.zeros(3), ranges), ranges[:, 1])
    np.testing.assert_allclose(control_array_to_normalized(ranges[:, 0], ranges), np.ones(3))
    np.testing.assert_allclose(control_array_to_normalized(ranges[:, 1], ranges), np.zeros(3))


def test_non_finite_normalized_values_map_to_open_controls():
    ranges = np.array([[0.0, 1.0], [0.1, 0.5], [0.2, 0.6]])

    controls = normalized_array_to_control([np.nan, np.inf, -np.inf], ranges)

    np.testing.assert_allclose(controls, ranges[:, 0])


def test_invalid_actuator_ranges_are_rejected():
    with pytest.raises(ValueError, match="positive width"):
        normalized_array_to_control([0.5], [[1.0, 1.0]])
    with pytest.raises(ValueError, match="positive width"):
        control_array_to_normalized([0.5], [[1.0, 0.0]])


def test_default_dex3_actuator_index_mapping_is_preserved():
    model = mujoco.MjModel.from_xml_path(str(MODEL_DIR / "scene_43dof.xml"))
    actuator_joint_ids = model.actuator_trnid[:, 0]

    indices = actuator_indices_for_joints(model, actuator_joint_ids)

    np.testing.assert_array_equal(indices, np.arange(model.nu))
