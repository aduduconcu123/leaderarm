#!/usr/bin/env python3
"""Read-only signal test for the three leader-arm Feetech motors."""

import argparse

from feetech_driver.motor import STS3250
from feetech_driver.protocol import FeetechProtocol


DEFAULT_MOTORS = [
    ("J1", 1),
    ("J2", 3),
    ("J3", 2),
]


def parse_motor_list(value):
    """Parse IDs like '1,3,2' or joint pairs like 'J1:1,J2:3,J3:2'."""
    motors = []
    for index, item in enumerate(value.split(","), start=1):
        item = item.strip()
        if not item:
            continue
        if ":" in item:
            joint, motor_id = item.split(":", 1)
            joint = joint.strip() or f"M{index}"
        else:
            joint, motor_id = f"M{index}", item
        motors.append((joint, int(motor_id)))
    if not motors:
        raise argparse.ArgumentTypeError("motor list is empty")
    return motors


def read_field(label, reader):
    try:
        return True, reader()
    except Exception as exc:
        return False, f"{type(exc).__name__}: {exc}"


def test_motor(motor):
    checks = [
        ("PING", motor.ping),
        ("RAW_POSITION", motor.read_raw_position),
        ("POSITION_DEG", motor.read_position),
        ("VELOCITY", motor.read_velocity),
        ("LOAD", motor.read_load),
        ("CURRENT_RAW", motor.read_current),
        ("STATUS", motor.read_status),
        ("MOVING", motor.is_moving),
        ("TORQUE", motor.is_torque_enabled),
        ("VOLTAGE", motor.read_voltage),
        ("TEMPERATURE", motor.read_temperature),
    ]

    ok_count = 0
    for label, reader in checks:
        ok, value = read_field(label, reader)
        marker = "OK" if ok else "ERR"
        print(f"  [{marker}] {label}: {value}")
        ok_count += int(ok)
        if label == "PING" and not ok:
            break
    return ok_count == len(checks)


def main():
    parser = argparse.ArgumentParser(
        description="Test whether J1/ID1, J2/ID3, J3/ID2 respond on one Feetech bus."
    )
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port")
    parser.add_argument("--baudrate", type=int, default=1000000, help="Serial baudrate")
    parser.add_argument(
        "--motors",
        type=parse_motor_list,
        default=DEFAULT_MOTORS,
        help="Motor list, for example '1,3,2' or 'J1:1,J2:3,J3:2'",
    )
    args = parser.parse_args()

    protocol = None
    results = []
    try:
        protocol = FeetechProtocol(port=args.port, baudrate=args.baudrate)
        print(f"Port: {args.port} @ {args.baudrate}")
        print("Read-only test. No torque or goal position is changed.")
        for joint, motor_id in args.motors:
            print(f"\n{joint} / ID {motor_id}")
            motor = STS3250(motor_id=motor_id, protocol=protocol)
            results.append((joint, motor_id, test_motor(motor)))
    finally:
        if protocol is not None:
            protocol.close()

    print("\nSummary")
    for joint, motor_id, ok in results:
        marker = "OK" if ok else "FAIL"
        print(f"  [{marker}] {joint} / ID {motor_id}")

    if not results or not all(ok for _, _, ok in results):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
