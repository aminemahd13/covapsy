from covapsy_bridge.race_profiles import resolve_profile_speed_cap


def test_bridge_profile_cap_homologation():
    cap = resolve_profile_speed_cap(
        race_profile="HOMOLOGATION",
        deployment_mode="real",
        max_speed_real_cap=2.0,
        max_speed_sim_cap=2.5,
    )
    assert cap == 0.8


def test_bridge_profile_cap_respects_external_limit():
    cap = resolve_profile_speed_cap(
        race_profile="RACE_AGGRESSIVE",
        deployment_mode="real",
        max_speed_real_cap=2.2,
        max_speed_sim_cap=3.2,
    )
    assert cap == 2.2
