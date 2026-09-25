import math
from pathlib import Path
from typing import Literal

import mujoco


def _attach_inspire_hands(base: mujoco.MjSpec, model_dir: Path) -> None:
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


def load_g1_with_rh56dfx_and_fixture(
    model_dir: Path,
    fixture_path: Path,
    fixture_body_name: str = "stand_base",
    fixture_prefix: str = "object_",
    fixture_position: tuple[float, float, float] = (0.55, 0.0, 0.0),
    fixture_yaw: float = math.pi,
) -> mujoco.MjModel:
    base = mujoco.MjSpec.from_file(str(model_dir / "scene_29dof_inspire_base.xml"))
    _attach_inspire_hands(base, model_dir)

    fixture_path = Path(fixture_path).expanduser().resolve()
    fixture = mujoco.MjSpec.from_file(str(fixture_path))
    fixture_body = fixture.body(fixture_body_name)
    if fixture_body is None:
        raise ValueError(
            f"Fixture body '{fixture_body_name}' not found in {fixture_path}"
        )

    half_yaw = fixture_yaw / 2.0
    frame = base.worldbody.add_frame(
        pos=list(fixture_position),
        quat=[math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)],
    )
    frame.attach_body(fixture_body, fixture_prefix, "")
    return base.compile()


def load_g1_with_rh56dfx_and_valve(
    model_dir: Path,
    valve_orientation: Literal["none", "vertical", "horizontal"] = "vertical",
    valve_position: tuple[float, float, float] = (0.55, 0.0, 0.0),
    valve_yaw: float = math.pi,
) -> mujoco.MjModel:
    if valve_orientation not in ("none", "vertical", "horizontal"):
        raise ValueError(
            "valve_orientation must be none, vertical, or horizontal"
        )
    if valve_orientation == "none":
        base = mujoco.MjSpec.from_file(
            str(model_dir / "scene_29dof_inspire_base.xml")
        )
        _attach_inspire_hands(base, model_dir)
        return base.compile()

    return load_g1_with_rh56dfx_and_fixture(
        model_dir,
        fixture_path=(
            model_dir
            / "valve_fixture"
            / f"valve_fixture_print_only_standalone_{valve_orientation}.xml"
        ),
        fixture_body_name="stand_base",
        fixture_prefix="valve_",
        fixture_position=valve_position,
        fixture_yaw=valve_yaw,
    )


def load_g1_with_rh56dfx_and_object(
    model_dir: Path,
    object_load: str,
) -> mujoco.MjModel:
    # Accept a registered name or model.xml#body_name.
    aliases = {
        "valve": "vertical",
        "valve-vertical": "vertical",
        "valve-horizontal": "horizontal",
    }
    if object_load in aliases:
        orientation = aliases[object_load]
        return load_g1_with_rh56dfx_and_fixture(
            model_dir,
            fixture_path=(
                model_dir
                / "valve_fixture"
                / f"valve_fixture_print_only_standalone_{orientation}.xml"
            ),
            fixture_body_name="stand_base",
            fixture_prefix="object_",
        )

    path_text, separator, body_name = object_load.partition("#")
    object_path = Path(path_text).expanduser()
    if not object_path.is_file():
        choices = ", ".join(aliases)
        raise ValueError(
            f"Unknown object_load '{object_load}'. Use one of [{choices}] "
            "or /path/to/model.xml#body_name"
        )
    return load_g1_with_rh56dfx_and_fixture(
        model_dir,
        fixture_path=object_path,
        fixture_body_name=body_name if separator and body_name else "stand_base",
        fixture_prefix="object_",
    )
