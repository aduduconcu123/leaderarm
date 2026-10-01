#!/usr/bin/env python3
"""Inspect Mirabo command encoding offline; never opens or sends on CAN."""

import argparse
import struct

from leader_controller import mapping
from leader_controller.mirabo_can import (
    COMMAND_PAYLOAD_FORMAT, pack_position_command, validate_command_field,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--position-deg', type=float, required=True)
    parser.add_argument('--speed', type=int, default=mapping.COMMAND_SPEED)
    parser.add_argument('--acceleration', type=int,
                        default=mapping.COMMAND_ACCEL)
    args = parser.parse_args()
    try:
        validate_command_field(args.speed, 'speed')
        validate_command_field(args.acceleration, 'acceleration')
        payload = pack_position_command(
            args.position_deg, args.speed, args.acceleration,
        )
    except ValueError as exc:
        parser.error(str(exc))
    position_raw, speed_raw, acceleration_raw = struct.unpack(
        COMMAND_PAYLOAD_FORMAT, payload,
    )
    print(f'position input (deg): {args.position_deg}')
    print(f'speed input: {args.speed}')
    print(f'acceleration input: {args.acceleration}')
    print(f'encoded position raw: {position_raw}')
    print(f'encoded speed raw: {speed_raw}')
    print(f'encoded acceleration raw: {acceleration_raw}')
    print(f'CAN payload format: {COMMAND_PAYLOAD_FORMAT}')
    print(f'8-byte payload hex: {payload.hex()}')


if __name__ == '__main__':
    main()
