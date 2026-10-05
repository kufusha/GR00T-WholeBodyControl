from gear_sonic.data.features_sonic_vla import get_inspire_valve_features


def test_inspire_valve_features_have_fixed_shapes_and_left_right_names():
    features = get_inspire_valve_features()

    assert set(features) == {
        "observation.inspire.position",
        "action.inspire.position",
        "observation.sim.valve_angle",
    }
    assert features["observation.inspire.position"]["shape"] == (12,)
    assert features["action.inspire.position"]["shape"] == (12,)
    assert features["observation.sim.valve_angle"]["shape"] == (1,)
    assert features["observation.inspire.position"]["names"][0] == "left_pinky"
    assert features["observation.inspire.position"]["names"][6] == "right_pinky"
