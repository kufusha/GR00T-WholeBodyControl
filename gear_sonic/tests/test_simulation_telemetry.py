import msgpack
import numpy as np
import pytest

from gear_sonic.utils.mujoco_sim.configs import SimLoopConfig
from gear_sonic.utils.mujoco_sim.simulation_telemetry import (
    build_simulation_telemetry,
    is_fresh_simulation_telemetry,
)


def _payload(**overrides):
    values = {
        "source_timestamp_ns": 10,
        "sim_time": 0.25,
        "left_position": np.arange(6),
        "right_position": np.arange(6) + 10,
        "left_command_position": np.arange(6) + 60,
        "right_command_position": np.arange(6) + 70,
        "valve_angle": 0.5,
    }
    values.update(overrides)
    return build_simulation_telemetry(**values)


def test_payload_uses_dataset_left_then_right_order_and_is_msgpack_safe():
    payload = _payload()

    assert set(payload) == {
        "schema_version",
        "source_timestamp_ns",
        "sim_time",
        "inspire_position",
        "action_inspire_position",
        "valve_angle",
    }
    np.testing.assert_array_equal(payload["inspire_position"], np.r_[0:6, 10:16])
    np.testing.assert_array_equal(payload["action_inspire_position"], np.r_[60:66, 70:76])
    msgpack.packb(payload, use_bin_type=True)


def test_payload_rejects_wrong_shapes_and_non_finite_values():
    with pytest.raises(ValueError, match="left_position must have shape"):
        _payload(left_position=np.zeros(5))
    with pytest.raises(ValueError, match="valve_angle must be finite"):
        _payload(valve_angle=np.nan)


def test_freshness_uses_source_timestamp_and_includes_boundary():
    message = {"source_timestamp_ns": 1_000}

    assert is_fresh_simulation_telemetry(message, now_ns=1_100, max_age_ns=100)
    assert not is_fresh_simulation_telemetry(message, now_ns=1_101, max_age_ns=100)
    assert not is_fresh_simulation_telemetry(message, now_ns=999, max_age_ns=100)


class _StubInspireBridge:
    low_cmd = None
    joystick = None

    def PublishLowState(self, obs):
        self.last_obs = obs

    def GetInspireCommand(self):
        return np.full(6, 0.25), np.full(6, 0.75)

def test_headless_valve_scene_builds_finite_telemetry():
    try:
        from gear_sonic.utils.mujoco_sim.base_sim import DefaultEnv
    except ModuleNotFoundError as error:
        if error.name in {"unitree_sdk2py", "cyclonedds"}:
            pytest.skip(f"Unitree DDS runtime unavailable: {error.name}")
        raise
    config = SimLoopConfig(
        hand_type="inspire",
        object_load="valve",
        enable_onscreen=False,
        enable_offscreen=False,
    )
    wbc = config.load_wbc_yaml()
    wbc.update(
        {
            "ROBOT_SCENE": "gear_sonic/data/robot_model/model_data/g1/scene_29dof_inspire_base.xml",
            "NUM_HAND_MOTORS": 6,
            "NUM_HAND_JOINTS": 6,
            "OBJECT_LOAD": "valve",
            "ENABLE_SIMULATION_TELEMETRY": False,
        }
    )
    env = DefaultEnv(wbc, onscreen=False, offscreen=False)
    try:
        env.set_unitree_bridge(_StubInspireBridge())
        for _ in range(4):
            env.sim_step()
        payload = env.build_simulation_telemetry()
    finally:
        env.close()

    assert np.asarray(payload["inspire_position"]).shape == (12,)
    assert np.asarray(payload["action_inspire_position"]).shape == (12,)
    assert np.isfinite(payload["valve_angle"])
    assert payload["source_timestamp_ns"] > 0
