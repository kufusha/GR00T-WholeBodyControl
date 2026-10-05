from pathlib import Path
import math

import pytest

from gear_sonic.scripts.launch_data_collection import (
    DataCollectionLaunchConfig,
    _build_exporter_command,
    _build_sim_command,
    validate_launch_config,
)


def test_inspire_valve_builds_sim_and_exporter_commands():
    config = DataCollectionLaunchConfig(
        sim=True,
        inspire_valve=True,
        valve_hand_mode="two_hand",
        valve_target_angle_deg=-90.0,
        valve_tolerance_deg=5.0,
    )

    sim = _build_sim_command(config, Path("/repo"))
    exporter = _build_exporter_command(config, Path("/repo"))

    assert "--hand-type inspire" in sim
    assert "--object-load valve" in sim
    assert "--enable-simulation-telemetry" in sim
    assert "--inspire-valve" in exporter
    assert "--valve-target-angle-deg -90.0" in exporter


def test_horizontal_valve_builds_matching_sim_and_exporter_commands():
    config = DataCollectionLaunchConfig(
        sim=True,
        inspire_valve=True,
        valve_orientation="horizontal",
        valve_target_angle_deg=-90.0,
        valve_tolerance_deg=5.0,
    )

    sim = _build_sim_command(config, Path("/repo"))
    exporter = _build_exporter_command(config, Path("/repo"))

    assert "--object-load valve-horizontal" in sim
    assert "--valve-orientation horizontal" in exporter


def test_inspire_valve_rejects_real_mode_and_missing_target():
    with pytest.raises(ValueError, match="requires --sim"):
        validate_launch_config(DataCollectionLaunchConfig(inspire_valve=True))
    with pytest.raises(ValueError, match="target-angle"):
        validate_launch_config(
            DataCollectionLaunchConfig(sim=True, inspire_valve=True)
        )


@pytest.mark.parametrize("target", [math.nan, math.inf, -math.inf])
def test_inspire_valve_rejects_non_finite_target(target):
    with pytest.raises(ValueError, match="finite target angle"):
        validate_launch_config(
            DataCollectionLaunchConfig(
                sim=True,
                inspire_valve=True,
                valve_target_angle_deg=target,
                valve_tolerance_deg=5.0,
            )
        )


@pytest.mark.parametrize("tolerance", [0.0, -1.0, math.nan, math.inf])
def test_inspire_valve_rejects_invalid_tolerance(tolerance):
    with pytest.raises(ValueError, match="positive finite tolerance"):
        validate_launch_config(
            DataCollectionLaunchConfig(
                sim=True,
                inspire_valve=True,
                valve_target_angle_deg=90.0,
                valve_tolerance_deg=tolerance,
            )
        )


def test_launcher_rejects_non_positive_collection_frequency():
    with pytest.raises(ValueError, match="frequency must be positive"):
        validate_launch_config(DataCollectionLaunchConfig(data_exporter_frequency=0))


def test_launcher_rejects_unknown_valve_orientation():
    with pytest.raises(ValueError, match="Invalid --valve-orientation"):
        validate_launch_config(
            DataCollectionLaunchConfig(valve_orientation="diagonal")
        )


def test_generic_sim_command_remains_dex3_compatible():
    command = _build_sim_command(DataCollectionLaunchConfig(sim=True), Path("/repo"))

    assert "--hand-type inspire" not in command
    assert "--object-load" not in command
