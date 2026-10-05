"""Validation helpers for simulation-only Inspire and valve telemetry."""

import numpy as np


DEFAULT_SIMULATION_TELEMETRY_PORT = 5558
SIMULATION_TELEMETRY_SCHEMA_VERSION = 1

_VECTOR_FIELDS = (
    ("inspire_position", "left_position", "right_position"),
    ("action_inspire_position", "left_command_position", "right_command_position"),
)
_SCALAR_FIELDS = ("valve_angle",)


def _finite_vector(name: str, values, size: int = 6) -> np.ndarray:
    result = np.asarray(values, dtype=np.float64).reshape(-1)
    if result.shape != (size,):
        raise ValueError(f"{name} must have shape ({size},), got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} contains non-finite values")
    return result


def build_simulation_telemetry(*, source_timestamp_ns: int, sim_time: float, **values) -> dict:
    if source_timestamp_ns < 0:
        raise ValueError("source_timestamp_ns must be non-negative")
    if not np.isfinite(sim_time):
        raise ValueError("sim_time must be finite")
    payload = {
        "schema_version": SIMULATION_TELEMETRY_SCHEMA_VERSION,
        "source_timestamp_ns": int(source_timestamp_ns),
        "sim_time": float(sim_time),
    }
    for output_name, left_name, right_name in _VECTOR_FIELDS:
        payload[output_name] = np.concatenate(
            (_finite_vector(left_name, values[left_name]),
             _finite_vector(right_name, values[right_name]))
        ).tolist()
    for field in _SCALAR_FIELDS:
        value = float(values[field])
        if not np.isfinite(value):
            raise ValueError(f"{field} must be finite")
        payload[field] = value
    return payload


def is_fresh_simulation_telemetry(message: dict, now_ns: int, max_age_ns: int) -> bool:
    if max_age_ns < 0 or "source_timestamp_ns" not in message:
        return False
    try:
        timestamp = int(message["source_timestamp_ns"])
    except (TypeError, ValueError, OverflowError):
        return False
    age = int(now_ns) - timestamp
    return 0 <= age <= int(max_age_ns)
