import numpy as np

from gear_sonic.utils.teleop.inspire_hand_control import (
    TriggerGraspController,
    inspire_closure_to_dex3,
)


def _update(controller, trigger, grip, dt=1.0):
    controller.last_update_time -= dt
    return controller.update(trigger, grip)


def test_trigger_without_grip_is_limited_to_half_closure():
    controller = TriggerGraspController(max_closure_rate=100.0)
    assert _update(controller, 1.0, 0.0) == 0.5


def test_grip_held_before_trigger_enables_full_closure():
    controller = TriggerGraspController(max_closure_rate=100.0)
    assert _update(controller, 0.0, 1.0) == 0.0
    assert _update(controller, 0.5, 1.0) == 0.5
    assert _update(controller, 1.0, 1.0) == 1.0


def test_grip_pressed_mid_trigger_does_not_upgrade_latched_mode():
    controller = TriggerGraspController(max_closure_rate=100.0)
    assert _update(controller, 0.8, 0.0) == 0.4
    assert _update(controller, 1.0, 1.0) == 0.5

    assert _update(controller, 0.0, 1.0) == 0.0
    assert _update(controller, 1.0, 1.0) == 1.0


def test_output_rate_is_limited():
    controller = TriggerGraspController(max_closure_rate=2.0)
    controller.last_update_time -= 0.05
    closure = controller.update(1.0, 1.0)
    assert 0.09 <= closure <= 0.11


def test_inspire_mapping_keeps_thumb_fixed_and_interpolates_four_fingers():
    open_right = inspire_closure_to_dex3(0.0, "right").reshape(-1)
    closed_right = inspire_closure_to_dex3(1.0, "right").reshape(-1)

    assert np.allclose(open_right[3:], [0.0, 0.0, 0.0, 0.0])
    assert np.allclose(closed_right[3:], [1.57, 1.75, 1.57, 1.75])
    assert np.allclose(open_right[:3], closed_right[:3])


def test_inspire_mapping_uses_mirrored_right_thumb_calibration():
    left = inspire_closure_to_dex3(0.0, "left").reshape(-1)
    right = inspire_closure_to_dex3(0.0, "right").reshape(-1)

    # Values are the inverse Dex3 representation of the measured Inspire
    # targets: left=(0.896, 0.0), right=(0.270, 0.978).
    assert np.allclose(left[:3], [-1.05, 0.1092, 0.182])
    assert np.allclose(right[:3], [-0.0231, -0.7665, -1.2775])


def test_non_finite_trigger_opens_hand_safely():
    for trigger in (np.nan, np.inf, -np.inf):
        controller = TriggerGraspController(max_closure_rate=100.0)
        assert _update(controller, 1.0, 1.0) == 1.0
        assert _update(controller, trigger, 1.0) == 0.0


def test_non_finite_grip_disables_full_grasp():
    for grip in (np.nan, np.inf, -np.inf):
        controller = TriggerGraspController(max_closure_rate=100.0)
        assert _update(controller, 1.0, grip) == 0.5


def test_non_finite_closure_maps_to_open_fingers():
    expected = inspire_closure_to_dex3(0.0, "right")
    for closure in (np.nan, np.inf, -np.inf):
        assert np.allclose(inspire_closure_to_dex3(closure, "right"), expected)
