import numpy as np
import pytest

from gear_sonic.utils.data_collection.valve_episode import ValveEpisodeTracker


def test_angle_unwrap_crosses_pi_without_jump():
    tracker = ValveEpisodeTracker(np.pi, 0.1, "two_hand")
    tracker.start(raw_angle=3.0, source_timestamp_ns=1)
    tracker.update(raw_angle=-3.0, source_timestamp_ns=2)

    assert tracker.current_angle == pytest.approx(2 * np.pi - 6.0)


def test_success_uses_rotation_relative_to_episode_start():
    tracker = ValveEpisodeTracker(1.0, 0.05, "left_only")
    tracker.start(raw_angle=0.2, source_timestamp_ns=10)
    tracker.update(raw_angle=1.21, source_timestamp_ns=20)

    summary = tracker.finish(manual_completed=True, discarded=False)

    assert summary["angle_success"] is True
    assert summary["hand_mode"] == "left_only"
    assert summary["final_angle_rad"] == pytest.approx(1.01)


def test_episode_summary_records_horizontal_valve_orientation():
    tracker = ValveEpisodeTracker(
        1.0, 0.05, "two_hand", valve_orientation="horizontal"
    )
    tracker.start(raw_angle=0.0, source_timestamp_ns=10)

    summary = tracker.finish(manual_completed=True, discarded=False)

    assert summary["valve_orientation"] == "horizontal"


def test_episode_summary_reports_source_quality():
    tracker = ValveEpisodeTracker(1.0, 0.05, "two_hand")
    tracker.record_drop("missing")
    tracker.start(raw_angle=0.0, source_timestamp_ns=1_000_000_000)
    tracker.record_sample(1_000_000_000, 2_000_000)
    tracker.record_sample(1_020_000_000, 3_000_000)
    tracker.record_drop("stale")

    summary = tracker.finish(manual_completed=False, discarded=False)

    assert summary["sample_count"] == 2
    assert summary["drop_counts"]["stale"] == 1
    assert summary["drop_counts"]["missing"] == 1
    assert summary["max_source_age_ns"] == 3_000_000
    assert summary["effective_frequency_hz"] == pytest.approx(50.0)


def test_tracker_rejects_unknown_hand_modes_and_drop_reasons():
    with pytest.raises(ValueError, match="hand mode"):
        ValveEpisodeTracker(1.0, 0.1, "both")
    with pytest.raises(ValueError, match="valve orientation"):
        ValveEpisodeTracker(1.0, 0.1, "two_hand", valve_orientation="diagonal")
    tracker = ValveEpisodeTracker(1.0, 0.1, "two_hand")
    with pytest.raises(ValueError, match="drop reason"):
        tracker.record_drop("other")
