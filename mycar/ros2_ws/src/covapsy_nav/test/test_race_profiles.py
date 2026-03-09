from covapsy_nav.race_profiles import resolve_profile_speed_cap


def test_homologation_cap_is_locked():
    cap = resolve_profile_speed_cap(
        race_profile="HOMOLOGATION",
        deployment_mode="real",
        max_speed_real_cap=3.0,
        max_speed_sim_cap=3.0,
    )
    assert cap == 0.8


def test_stable_profile_uses_sim_cap():
    cap = resolve_profile_speed_cap(
        race_profile="RACE_STABLE",
        deployment_mode="sim",
        max_speed_real_cap=2.0,
        max_speed_sim_cap=3.0,
    )
    assert cap == 2.5


def test_external_cap_can_reduce_profile_limit():
    cap = resolve_profile_speed_cap(
        race_profile="RACE_AGGRESSIVE",
        deployment_mode="real",
        max_speed_real_cap=2.2,
        max_speed_sim_cap=3.2,
    )
    assert cap == 2.2
