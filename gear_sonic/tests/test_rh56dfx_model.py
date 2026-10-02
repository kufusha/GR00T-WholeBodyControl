import importlib.util
from pathlib import Path
import unittest

import mujoco
import numpy as np

from gear_sonic.utils.mujoco_sim.inspire_sim_mapping import (
    control_array_to_normalized,
    normalized_array_to_control,
)


ROOT = Path(__file__).resolve().parents[2]
MODEL_DIR = ROOT / "gear_sonic/data/robot_model/model_data/g1"
LOADER_PATH = ROOT / "gear_sonic/utils/mujoco_sim/rh56dfx_model.py"


class Rh56dfxModelTests(unittest.TestCase):
    def test_bimanual_model_compiles_with_six_active_actuators_per_hand(self):
        self.assertTrue(LOADER_PATH.is_file(), "RH56DFX model loader is missing")
        spec = importlib.util.spec_from_file_location("rh56dfx_model", LOADER_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        model = module.load_g1_with_rh56dfx(MODEL_DIR)
        active_suffixes = (
            "pinky_proximal_joint",
            "ring_proximal_joint",
            "middle_proximal_joint",
            "index_proximal_joint",
            "thumb_proximal_pitch_joint",
            "thumb_proximal_yaw_joint",
        )

        for side in ("left", "right"):
            for suffix in active_suffixes:
                joint_name = f"{side}_hand_{suffix}"
                joint_id = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
                )
                self.assertNotEqual(joint_id, -1, joint_name)
                actuator_ids = [
                    actuator_id
                    for actuator_id, transmission in enumerate(model.actuator_trnid)
                    if transmission[0] == joint_id
                ]
                self.assertEqual(len(actuator_ids), 1, joint_name)

    def test_normalized_position_maps_open_to_low_joint_load(self):
        self.assertTrue(LOADER_PATH.is_file(), "RH56DFX model loader is missing")
        spec = importlib.util.spec_from_file_location("rh56dfx_model", LOADER_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        model = module.load_g1_with_rh56dfx(MODEL_DIR)

        actuator_id = model.actuator("right_hand_index").id
        lower, upper = model.actuator_ctrlrange[actuator_id]
        open_control = module.normalized_to_control(1.0, lower, upper)
        closed_control = module.normalized_to_control(0.0, lower, upper)

        self.assertEqual(open_control, lower)
        self.assertEqual(closed_control, upper)
        self.assertEqual(module.control_to_normalized(lower, lower, upper), 1.0)
        self.assertEqual(module.control_to_normalized(upper, lower, upper), 0.0)

    def test_all_twelve_hand_actuators_round_trip_open_and_closed(self):
        spec = importlib.util.spec_from_file_location("rh56dfx_model", LOADER_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        model = module.load_g1_with_rh56dfx(MODEL_DIR)
        hand_actuator_ids = [
            actuator_id
            for actuator_id in range(model.nu)
            if "hand" in model.actuator(actuator_id).name
        ]
        self.assertEqual(len(hand_actuator_ids), 12)
        ranges = model.actuator_ctrlrange[hand_actuator_ids]

        open_controls = normalized_array_to_control(np.ones(12), ranges)
        closed_controls = normalized_array_to_control(np.zeros(12), ranges)

        np.testing.assert_allclose(open_controls, ranges[:, 0])
        np.testing.assert_allclose(closed_controls, ranges[:, 1])
        np.testing.assert_allclose(control_array_to_normalized(open_controls, ranges), 1.0)
        np.testing.assert_allclose(control_array_to_normalized(closed_controls, ranges), 0.0)


if __name__ == "__main__":
    unittest.main()
