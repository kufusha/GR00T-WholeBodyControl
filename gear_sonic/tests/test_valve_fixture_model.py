import importlib.util
import math
from pathlib import Path
import unittest

import mujoco

from gear_sonic.utils.mujoco_sim.configs import SimLoopConfig


ROOT = Path(__file__).resolve().parents[2]
MODEL_DIR = ROOT / "gear_sonic/data/robot_model/model_data/g1"
LOADER_PATH = ROOT / "gear_sonic/utils/mujoco_sim/object_scene_loader.py"


class ValveFixtureModelTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(LOADER_PATH.is_file(), "Object scene loader is missing")
        spec = importlib.util.spec_from_file_location("object_scene_loader", LOADER_PATH)
        self.loader = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.loader)

    def test_object_loading_is_opt_in(self):
        self.assertIsNone(SimLoopConfig().object_load)

    def test_vertical_fixture_is_composed_with_g1_and_inspire_hands(self):
        model = self.loader.load_g1_with_rh56dfx_and_valve(
            MODEL_DIR,
            valve_orientation="vertical",
            valve_position=(0.55, 0.0, 0.0),
            valve_yaw=math.pi,
        )
        self.assertNotEqual(
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "valve_valve_spin_joint"),
            -1,
        )
        self.assertNotEqual(
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_hand_base"), -1
        )

    def test_horizontal_fixture_is_composed(self):
        model = self.loader.load_g1_with_rh56dfx_and_valve(
            MODEL_DIR,
            valve_orientation="horizontal",
            valve_position=(0.55, 0.0, 0.0),
            valve_yaw=math.pi,
        )
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, "valve_valve_spin_joint"
        )
        self.assertNotEqual(joint_id, -1)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        self.assertAlmostEqual(data.body("valve_valve_rotor").xpos[2], 0.95, places=4)

    def test_none_keeps_fixture_out_of_scene(self):
        model = self.loader.load_g1_with_rh56dfx_and_valve(
            MODEL_DIR, valve_orientation="none"
        )
        self.assertEqual(
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "valve_valve_spin_joint"),
            -1,
        )

    def test_invalid_orientation_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "valve_orientation"):
            self.loader.load_g1_with_rh56dfx_and_valve(
                MODEL_DIR, valve_orientation="diagonal"
            )

    def test_generic_fixture_path_and_body_name_are_supported(self):
        fixture_path = (
            MODEL_DIR
            / "valve_fixture/valve_fixture_print_only_standalone_vertical.xml"
        )
        model = self.loader.load_g1_with_rh56dfx_and_fixture(
            MODEL_DIR,
            fixture_path=fixture_path,
            fixture_body_name="stand_base",
            fixture_prefix="training_",
        )
        self.assertNotEqual(
            mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, "training_valve_spin_joint"
            ),
            -1,
        )

    def test_named_object_selects_horizontal_valve(self):
        model = self.loader.load_g1_with_rh56dfx_and_object(
            MODEL_DIR, object_load="valve-horizontal"
        )
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        self.assertAlmostEqual(data.body("object_valve_rotor").xpos[2], 0.95, places=4)


if __name__ == "__main__":
    unittest.main()
