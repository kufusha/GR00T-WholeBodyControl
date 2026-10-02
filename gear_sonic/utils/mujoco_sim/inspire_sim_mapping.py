"""Pure Inspire command/state mappings shared by the MuJoCo bridge."""

import numpy as np


def _finite_unit_array(values) -> np.ndarray:
    values = np.asarray(values, dtype=np.float64).reshape(-1)
    return np.where(np.isfinite(values), np.clip(values, 0.0, 1.0), 1.0)


def _validated_ranges(ranges, expected_size: int) -> np.ndarray:
    ranges = np.asarray(ranges, dtype=np.float64)
    if ranges.shape != (expected_size, 2):
        raise ValueError(f"Expected {expected_size} actuator ranges, got {ranges.shape}")
    if not np.all(np.isfinite(ranges)) or np.any(ranges[:, 1] <= ranges[:, 0]):
        raise ValueError("Actuator control ranges must have positive width")
    return ranges


def normalized_array_to_control(values, ranges) -> np.ndarray:
    """Map normalized Inspire positions (1=open) to actuator controls."""
    normalized = _finite_unit_array(values)
    ranges = _validated_ranges(ranges, normalized.size)
    return ranges[:, 0] + (1.0 - normalized) * (ranges[:, 1] - ranges[:, 0])


def control_array_to_normalized(values, ranges) -> np.ndarray:
    """Map actuator controls to normalized Inspire positions (1=open)."""
    values = np.asarray(values, dtype=np.float64).reshape(-1)
    ranges = _validated_ranges(ranges, values.size)
    normalized = 1.0 - (values - ranges[:, 0]) / (ranges[:, 1] - ranges[:, 0])
    return _finite_unit_array(normalized)


def split_inspire_command_positions(values, num_hand_motors: int) -> tuple[np.ndarray, np.ndarray]:
    """Decode DDS right-then-left command positions into left/right arrays."""
    values = np.asarray(values, dtype=np.float64).reshape(-1)
    if values.size < 2 * num_hand_motors:
        opened = np.ones(num_hand_motors, dtype=np.float64)
        return opened, opened.copy()
    right = _finite_unit_array(values[:num_hand_motors])
    left = _finite_unit_array(values[num_hand_motors : 2 * num_hand_motors])
    return left, right


def pack_inspire_state_positions(left, right) -> np.ndarray:
    """Encode left/right normalized positions in DDS right-then-left order."""
    left = _finite_unit_array(left)
    right = _finite_unit_array(right)
    if left.size != right.size:
        raise ValueError("Left and right Inspire state arrays must have equal length")
    return np.concatenate((right, left))


def pack_inspire_state_velocities(left, right) -> np.ndarray:
    """Encode signed velocities in DDS right-then-left order."""
    left = np.asarray(left, dtype=np.float64).reshape(-1)
    right = np.asarray(right, dtype=np.float64).reshape(-1)
    if left.size != right.size:
        raise ValueError("Left and right Inspire velocity arrays must have equal length")
    left = np.where(np.isfinite(left), left, 0.0)
    right = np.where(np.isfinite(right), right, 0.0)
    return np.concatenate((right, left))


def actuator_indices_for_joints(model, joint_ids) -> np.ndarray:
    """Resolve actuator indices for one-actuator-per-joint MuJoCo models."""
    joint_to_actuator = {
        int(joint_id): actuator_id
        for actuator_id, joint_id in enumerate(model.actuator_trnid[:, 0])
        if joint_id >= 0
    }
    try:
        return np.array([joint_to_actuator[int(joint_id)] for joint_id in joint_ids])
    except KeyError as error:
        raise ValueError(f"No actuator found for joint id {error.args[0]}") from error
