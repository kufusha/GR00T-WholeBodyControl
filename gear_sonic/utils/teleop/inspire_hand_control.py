"""Safe PICO controller mapping for Inspire dexterous hands."""

import threading
import time

import numpy as np


def _finite_unit_interval(value: float, fallback: float = 0.0) -> float:
    value = float(value)
    if not np.isfinite(value):
        return fallback
    return float(np.clip(value, 0.0, 1.0))


class TriggerGraspController:
    """Convert analog Trigger/Grip input into a safe grasp closure.

    A Trigger press that starts without Grip is limited to 50% closure. Full
    closure is enabled only when Grip is already held as Trigger crosses the
    press threshold. The selected mode stays latched until Trigger is released,
    so pressing Grip midway through a grasp cannot cause a 50% -> 100% jump.
    """

    def __init__(
        self,
        press_threshold: float = 0.05,
        release_threshold: float = 0.02,
        grip_threshold: float = 0.5,
        max_closure_rate: float = 2.0,
    ) -> None:
        self.press_threshold = press_threshold
        self.release_threshold = release_threshold
        self.grip_threshold = grip_threshold
        self.max_closure_rate = max_closure_rate
        self.active = False
        self.full_grasp_mode = False
        self.release_required = False
        self.closure = 0.0
        self.last_update_time = time.monotonic()
        self._lock = threading.Lock()

    def reset_for_mode_transition(self) -> None:
        """Clear the grasp session and require a fresh Trigger release."""
        with self._lock:
            self.active = False
            self.full_grasp_mode = False
            self.release_required = True
            self.closure = 0.0
            self.last_update_time = time.monotonic()

    def update(self, trigger: float, grip: float) -> float:
        trigger_value = float(trigger)
        trigger_valid = bool(np.isfinite(trigger_value))
        trigger = _finite_unit_interval(trigger_value)
        grip = _finite_unit_interval(grip)

        with self._lock:
            now = time.monotonic()
            dt = max(0.0, min(now - self.last_update_time, 0.1))
            self.last_update_time = now

            if self.release_required:
                if not trigger_valid or trigger > self.release_threshold:
                    return 0.0
                self.release_required = False

            if self.active:
                if trigger <= self.release_threshold:
                    self.active = False
                    self.full_grasp_mode = False
            elif trigger >= self.press_threshold:
                self.active = True
                self.full_grasp_mode = grip >= self.grip_threshold

            if trigger <= self.release_threshold:
                target = 0.0
            else:
                scale = 1.0 if self.full_grasp_mode else 0.5
                target = trigger * scale

            max_step = self.max_closure_rate * dt
            self.closure += float(np.clip(target - self.closure, -max_step, max_step))
            self.closure = float(np.clip(self.closure, 0.0, 1.0))
            return self.closure


class BimanualGraspController:
    """Own left/right grasp state for one active streaming session."""

    def __init__(
        self,
        left: TriggerGraspController | None = None,
        right: TriggerGraspController | None = None,
    ) -> None:
        self.left = left or TriggerGraspController()
        self.right = right or TriggerGraspController()

    def update(
        self,
        left_trigger: float,
        left_grip: float,
        right_trigger: float,
        right_grip: float,
    ) -> tuple[float, float]:
        return (
            self.left.update(left_trigger, left_grip),
            self.right.update(right_trigger, right_grip),
        )

    def on_mode_transition(self) -> None:
        self.left.reset_for_mode_transition()
        self.right.reset_for_mode_transition()


def inspire_closure_to_dex3(closure: float, hand: str) -> np.ndarray:
    """Build a Dex3-compatible command for the Inspire adapter.

    Only pinky/ring/middle/index follow ``closure``. The thumbs stay at
    side-specific measured poses because the mirrored RH56DFX mechanisms need
    different actuator targets to produce the same physical posture. The C++
    adapter converts this 7-DOF command back to Inspire's
    [pinky, ring, middle, index, thumb_bend, thumb_rotate] convention.
    """
    closure = _finite_unit_interval(closure)
    if hand == "left":
        thumb_bend_closure = 1.0 - 0.896
        thumb_rotate_closure = 1.0 - 0.0
        command = np.array(
            [
                -1.05 * thumb_rotate_closure,
                1.05 * thumb_bend_closure,
                1.75 * thumb_bend_closure,
                -1.57 * closure,
                -1.75 * closure,
                -1.57 * closure,
                -1.75 * closure,
            ],
            dtype=np.float32,
        )
    elif hand == "right":
        # Mirrored counterpart of the preferred left-thumb posture, measured
        # with the Inspire calibration tool.
        thumb_bend_closure = 1.0 - 0.270
        thumb_rotate_closure = 1.0 - 0.978
        command = np.array(
            [
                -1.05 * thumb_rotate_closure,
                -1.05 * thumb_bend_closure,
                -1.75 * thumb_bend_closure,
                1.57 * closure,
                1.75 * closure,
                1.57 * closure,
                1.75 * closure,
            ],
            dtype=np.float32,
        )
    else:
        raise ValueError(f"hand must be 'left' or 'right', got {hand!r}")

    return command.reshape(1, 7)
