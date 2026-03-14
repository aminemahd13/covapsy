"""Race profile speed caps for bridge-side hard limiting."""

from __future__ import annotations


_PROFILE_CAPS = {
    "HOMOLOGATION": {"real": 0.8, "sim": 0.8},
    "RACE_STABLE": {"real": 2.5, "sim": 2.5},
    "RACE_AGGRESSIVE": {"real": 2.6, "sim": 3.2},
}


def resolve_profile_speed_cap(
    race_profile: str,
    deployment_mode: str,
    max_speed_real_cap: float,
    max_speed_sim_cap: float,
) -> float:
    profile = str(race_profile).strip().upper()
    if profile not in _PROFILE_CAPS:
        profile = "RACE_STABLE"
    mode = "sim" if str(deployment_mode).strip().lower() == "sim" else "real"
    external_cap = max(0.05, float(max_speed_sim_cap if mode == "sim" else max_speed_real_cap))
    return min(float(_PROFILE_CAPS[profile][mode]), external_cap)
