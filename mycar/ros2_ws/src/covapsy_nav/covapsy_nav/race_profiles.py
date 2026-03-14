"""Race profile helpers shared across navigation nodes."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RaceProfileCaps:
    real_max_speed_m_s: float
    sim_max_speed_m_s: float


_PROFILE_CAPS = {
    "HOMOLOGATION": RaceProfileCaps(real_max_speed_m_s=0.8, sim_max_speed_m_s=0.8),
    "RACE_STABLE": RaceProfileCaps(real_max_speed_m_s=2.5, sim_max_speed_m_s=2.5),
    "RACE_AGGRESSIVE": RaceProfileCaps(real_max_speed_m_s=2.6, sim_max_speed_m_s=3.2),
}


def normalize_profile_name(name: str) -> str:
    candidate = str(name).strip().upper()
    if candidate in _PROFILE_CAPS:
        return candidate
    return "RACE_STABLE"


def resolve_profile_speed_cap(
    race_profile: str,
    deployment_mode: str,
    max_speed_real_cap: float,
    max_speed_sim_cap: float,
) -> float:
    """Return locked profile max speed clamped by external cap overrides."""
    profile = _PROFILE_CAPS[normalize_profile_name(race_profile)]
    mode = str(deployment_mode).strip().lower()

    real_cap = max(0.05, float(max_speed_real_cap))
    sim_cap = max(0.05, float(max_speed_sim_cap))

    if mode == "sim":
        return min(profile.sim_max_speed_m_s, sim_cap)
    return min(profile.real_max_speed_m_s, real_cap)
