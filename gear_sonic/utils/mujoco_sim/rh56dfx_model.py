"""Compose the bimanual RH56DFX model onto the G1 wrists."""

from pathlib import Path

import mujoco


def normalized_to_control(value: float, lower: float, upper: float) -> float:
    """Map Inspire position (1=open, 0=closed) to an MJCF control range."""
    clipped = min(max(float(value), 0.0), 1.0)
    return lower + (1.0 - clipped) * (upper - lower)


def control_to_normalized(value: float, lower: float, upper: float) -> float:
    """Map an MJCF control value to Inspire position (1=open, 0=closed)."""
    if upper <= lower:
        raise ValueError("Actuator control range must have positive width")
    normalized = 1.0 - (float(value) - lower) / (upper - lower)
    return min(max(normalized, 0.0), 1.0)


def load_g1_with_rh56dfx(model_dir: Path) -> mujoco.MjModel:
    """Load the handless G1 and attach the calibrated RH56DFX hand MJCFs."""
    base = mujoco.MjSpec.from_file(str(model_dir / "scene_29dof_inspire_base.xml"))
    hand_dir = model_dir / "rh56dfx"

    attachments = (
        (
            "left",
            "left_wrist_yaw_link",
            "inspire_left.xml",
            [0.054, 0.0, 0.0],
            [0.5, -0.5, 0.5, -0.5],
        ),
        (
            "right",
            "right_wrist_yaw_link",
            "inspire_right.xml",
            [0.054, 0.0, 0.0],
            [0.5, 0.5, 0.5, 0.5],
        ),
    )

    for side, wrist_name, filename, pos, quat in attachments:
        hand = mujoco.MjSpec.from_file(str(hand_dir / filename))
        wrist = base.body(wrist_name)
        if wrist is None:
            raise ValueError(f"RH56DFX attachment body not found: {wrist_name}")
        frame = wrist.add_frame(pos=pos, quat=quat)
        frame.attach_body(hand.body("base"), f"{side}_hand_", "")

    return base.compile()
