"""Valve episode angle tracking, quality statistics, and asset metadata."""

import math

import numpy as np


VALVE_MODEL_CONFIG = {
    "geometry_version": "ValveRotor_PrintOnly.stl",
    "overall_diameter_m": 0.250,
    "overall_thickness_m": 0.0337545,
    "handwheel_body_thickness_m": 0.0167545,
    "shaft_hole_diameter_m": 0.0202,
    "outer_ring_width_m": 0.0245,
    "spoke_count": 4,
    "spoke_width_m": 0.025,
    "center_hub_diameter_m": 0.0354,
    "rear_disc_diameter_m": 0.080,
    "fixed_shaft_diameter_m": 0.0198,
    "fixture_position_m": [0.55, 0.0, 0.0],
    "fixture_yaw_rad": math.pi,
    "wheel_center_height_m": 0.95,
    "joint_damping": 0.02,
    "joint_friction_loss": 0.3,
    "contact_friction": [1.0, 0.01, 0.0001],
    "rotor_mass_kg": 0.5,
}

_HAND_MODES = {"two_hand", "left_only", "right_only"}
_VALVE_ORIENTATIONS = {"vertical", "horizontal"}
_DROP_REASONS = {"missing", "stale", "invalid_shape"}


class ValveEpisodeTracker:
    def __init__(
        self,
        target_angle_rad: float,
        tolerance_rad: float,
        hand_mode: str,
        valve_orientation: str = "vertical",
    ):
        if hand_mode not in _HAND_MODES:
            raise ValueError(f"Unknown valve hand mode: {hand_mode}")
        if valve_orientation not in _VALVE_ORIENTATIONS:
            raise ValueError(f"Unknown valve orientation: {valve_orientation}")
        if not np.isfinite(target_angle_rad):
            raise ValueError("Target angle must be finite")
        if not np.isfinite(tolerance_rad) or tolerance_rad <= 0:
            raise ValueError("Tolerance must be finite and positive")
        self.target_angle_rad = float(target_angle_rad)
        self.tolerance_rad = float(tolerance_rad)
        self.hand_mode = hand_mode
        self.valve_orientation = valve_orientation
        self._started = False
        self._reset_quality()

    @property
    def started(self) -> bool:
        return self._started

    def _reset_quality(self):
        self.sample_count = 0
        self.drop_counts = {reason: 0 for reason in sorted(_DROP_REASONS)}
        self.max_source_age_ns = 0
        self.first_sample_timestamp_ns = None
        self.last_sample_timestamp_ns = None

    def start(self, raw_angle: float, source_timestamp_ns: int):
        if not np.isfinite(raw_angle):
            raise ValueError("Valve angle must be finite")
        self._started = True
        self.initial_raw_angle = float(raw_angle)
        self.previous_raw_angle = float(raw_angle)
        self.final_raw_angle = float(raw_angle)
        self.current_angle = 0.0
        self.minimum_angle = 0.0
        self.maximum_angle = 0.0
        self.start_timestamp_ns = int(source_timestamp_ns)
        self.end_timestamp_ns = int(source_timestamp_ns)

    def update(self, raw_angle: float, source_timestamp_ns: int):
        if not self._started:
            raise RuntimeError("Valve episode has not started")
        if not np.isfinite(raw_angle):
            raise ValueError("Valve angle must be finite")
        raw_angle = float(raw_angle)
        delta = (raw_angle - self.previous_raw_angle + np.pi) % (2 * np.pi) - np.pi
        self.current_angle += float(delta)
        self.previous_raw_angle = raw_angle
        self.final_raw_angle = raw_angle
        self.minimum_angle = min(self.minimum_angle, self.current_angle)
        self.maximum_angle = max(self.maximum_angle, self.current_angle)
        self.end_timestamp_ns = int(source_timestamp_ns)

    def record_sample(self, source_timestamp_ns: int, source_age_ns: int):
        timestamp = int(source_timestamp_ns)
        age = int(source_age_ns)
        if age < 0:
            raise ValueError("Source age must be non-negative")
        if self.first_sample_timestamp_ns is None:
            self.first_sample_timestamp_ns = timestamp
        self.last_sample_timestamp_ns = timestamp
        self.sample_count += 1
        self.max_source_age_ns = max(self.max_source_age_ns, age)

    def record_drop(self, reason: str):
        if reason not in _DROP_REASONS:
            raise ValueError(f"Unknown drop reason: {reason}")
        self.drop_counts[reason] += 1

    def finish(self, manual_completed: bool, discarded: bool) -> dict:
        if not self._started:
            raise RuntimeError("Valve episode has not started")
        frequency = 0.0
        if (
            self.sample_count >= 2
            and self.first_sample_timestamp_ns is not None
            and self.last_sample_timestamp_ns > self.first_sample_timestamp_ns
        ):
            frequency = (self.sample_count - 1) * 1e9 / (
                self.last_sample_timestamp_ns - self.first_sample_timestamp_ns
            )
        return {
            "source": "mujoco",
            "hand_type": "inspire_rh56dfx",
            "hand_mode": self.hand_mode,
            "valve_orientation": self.valve_orientation,
            "initial_raw_angle_rad": self.initial_raw_angle,
            "final_raw_angle_rad": self.final_raw_angle,
            "final_angle_rad": self.current_angle,
            "minimum_angle_rad": self.minimum_angle,
            "maximum_angle_rad": self.maximum_angle,
            "target_angle_rad": self.target_angle_rad,
            "tolerance_rad": self.tolerance_rad,
            "angle_success": abs(self.current_angle - self.target_angle_rad)
            <= self.tolerance_rad,
            "manual_completed": bool(manual_completed),
            "discarded": bool(discarded),
            "start_timestamp_ns": self.start_timestamp_ns,
            "end_timestamp_ns": self.end_timestamp_ns,
            "sample_count": self.sample_count,
            "drop_counts": self.drop_counts.copy(),
            "max_source_age_ns": self.max_source_age_ns,
            "effective_frequency_hz": frequency,
            "valve_model": VALVE_MODEL_CONFIG.copy(),
        }
