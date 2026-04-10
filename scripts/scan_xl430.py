#!/usr/bin/env python3
"""Scan DYNAMIXEL Protocol 2.0 servo IDs and baud rates."""

from __future__ import annotations

import argparse
from typing import Iterable, List, Tuple

try:
    from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
except Exception:
    COMM_SUCCESS = 0
    PacketHandler = None
    PortHandler = None


def parse_bauds(value: str) -> List[int]:
    return [int(v.strip()) for v in value.split(',') if v.strip()]


def scan_once(
    *,
    device: str,
    baud: int,
    protocol: float,
    id_range: Iterable[int],
) -> List[Tuple[int, int]]:
    found: List[Tuple[int, int]] = []
    port = PortHandler(device)
    if not port.openPort():
        return found
    if not port.setBaudRate(baud):
        port.closePort()
        return found

    packet = PacketHandler(protocol)
    try:
        for dxl_id in id_range:
            try:
                model, comm_result, dxl_error = packet.ping(port, int(dxl_id))
            except Exception:
                continue
            if comm_result == COMM_SUCCESS and dxl_error == 0:
                found.append((int(dxl_id), int(model)))
    finally:
        port.closePort()
    return found


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', default='/dev/waveshare_servo', help='Serial device path')
    parser.add_argument(
        '--bauds',
        default='57600,115200,1000000,2000000,3000000,4000000,4500000',
        help='Comma-separated baud rates to test',
    )
    parser.add_argument('--id-min', type=int, default=0, help='Minimum DYNAMIXEL ID')
    parser.add_argument('--id-max', type=int, default=20, help='Maximum DYNAMIXEL ID')
    parser.add_argument('--protocol', type=float, default=2.0, help='Protocol version')
    args = parser.parse_args()

    if PacketHandler is None or PortHandler is None:
        print('dynamixel_sdk is not installed. Install ros-jazzy-dynamixel-sdk.')
        return 2

    bauds = parse_bauds(args.bauds)
    if not bauds:
        print('No baud rates supplied.')
        return 2
    if args.id_min < 0 or args.id_max < args.id_min:
        print('Invalid ID range.')
        return 2

    print(f'Scanning device: {args.device}')
    print(f'Protocol: {args.protocol}')
    print(f'ID range: {args.id_min}..{args.id_max}')
    print(f'Baud list: {bauds}')

    any_found = False
    for baud in bauds:
        found = scan_once(
            device=args.device,
            baud=baud,
            protocol=args.protocol,
            id_range=range(args.id_min, args.id_max + 1),
        )
        if not found:
            print(f'[{baud}] no servos found')
            continue
        any_found = True
        pretty = ', '.join(f'id={dxl_id} model={model}' for dxl_id, model in found)
        print(f'[{baud}] found: {pretty}')

    if not any_found:
        print('No DYNAMIXEL servo responded.')
        print('Check power, USB mode jumper, device path, and ID/baud settings.')
        return 1

    print('Scan complete.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
