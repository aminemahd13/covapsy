#!/usr/bin/env python3
"""Helpers for STM32 serial device/baud candidate handling."""

from __future__ import annotations

from typing import Iterable, List, Sequence, Tuple


def split_csv_tokens(raw: str) -> List[str]:
    if raw is None:
        return []
    return [token.strip() for token in str(raw).split(',') if token.strip()]


def build_device_candidates(primary_device: str, candidates_csv: str) -> List[str]:
    ordered: List[str] = []
    seen = set()

    def _add(value: str) -> None:
        token = str(value or '').strip()
        if not token or token in seen:
            return
        seen.add(token)
        ordered.append(token)

    _add(primary_device)
    for item in split_csv_tokens(candidates_csv):
        _add(item)
    return ordered


def build_baud_candidates(primary_baud: int, candidates_csv: str) -> List[int]:
    ordered: List[int] = []
    seen = set()

    def _add_int(value: int) -> None:
        try:
            parsed = int(value)
        except Exception:
            return
        if parsed <= 0 or parsed in seen:
            return
        seen.add(parsed)
        ordered.append(parsed)

    _add_int(primary_baud)
    for token in split_csv_tokens(candidates_csv):
        _add_int(token)
    return ordered


def build_probe_pairs(devices: Sequence[str], bauds: Sequence[int]) -> List[Tuple[str, int]]:
    probes: List[Tuple[str, int]] = []
    for device in devices:
        token = str(device or '').strip()
        if not token:
            continue
        for baud in bauds:
            try:
                parsed = int(baud)
            except Exception:
                continue
            if parsed <= 0:
                continue
            probes.append((token, parsed))
    return probes

