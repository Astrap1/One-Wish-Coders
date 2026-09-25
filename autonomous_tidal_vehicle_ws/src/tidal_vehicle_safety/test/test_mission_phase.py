from tidal_vehicle_safety.mission_phase import MissionPhase


def test_mission_phases_are_internal_and_named_stably():
    assert [phase.value for phase in MissionPhase] == [
        "PRELAUNCH",
        "OUTBOUND",
        "DELIVERED",
        "RETURNING",
    ]
