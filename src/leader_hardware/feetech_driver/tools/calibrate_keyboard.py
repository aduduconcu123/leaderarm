#!/usr/bin/env python3
"""Calibrate J1/ID1, J2/ID3, J3/ID2 on one serial bus.

Z/L/H capture fresh ZERO/MIN/MAX encoder samples. L and H are the
negative/positive raw-encoder sides of zero (independent of direction).
Each limit must be less than half a revolution from zero. R reverses the
reported angle. All three landmarks must be captured this session before
saving; old files may contain defaults and are shown only as reference.

Torque is preserved on startup, motor selection, errors and exit. T alone
changes torque on the selected motor. Support the mechanism before releasing
it. A/D jog by -/+10 raw counts only with torque on and calibrated limits.
N selects the next motor, S saves all, Q or Ctrl+C exits.
"""

import sys
import termios
import time
import tty
from pathlib import Path

from feetech_driver.calibration import (
    CalibrationManager,
    MotorCalibration,
    encoder_offset,
)
from feetech_driver.motor import STS3250
from feetech_driver.protocol import FeetechProtocol

PORT = "/dev/ttyUSB0"
BAUDRATE = 1000000
MOTORS = [
    {"motor_id": 1, "joint": "J1"},
    {"motor_id": 3, "joint": "J2"},
    {"motor_id": 2, "joint": "J3"},
]
CALIBRATION_FILE = (
    Path(__file__).resolve().parent.parent / "calibration" / "calibration.json"
)
STEP = 10
PING_ATTEMPTS = 3
PING_RETRY_DELAY = 0.1
LANDMARKS = {"z": "zero_raw", "l": "limit_min", "h": "limit_max"}


def get_key():
    """Read one key, restoring the terminal even on failure."""
    fd = sys.stdin.fileno()
    settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, settings)


def read_raw(motor):
    """Reject missing or invalid encoder samples before using them."""
    raw = motor.read_raw_position()
    if not isinstance(raw, int) or not 0 <= raw <= 4095:
        raise ValueError(f"Invalid encoder sample: {raw}")
    return raw


def connect_motor(motor, joint):
    """Retry startup ping to tolerate a brief delay on the shared bus."""
    last_error = None
    for attempt in range(1, PING_ATTEMPTS + 1):
        try:
            motor.ping()
            print(f"Connected {joint} / ID {motor.motor_id}")
            return
        except Exception as exc:
            last_error = exc
            if attempt < PING_ATTEMPTS:
                print(
                    f"No response from {joint} / ID {motor.motor_id}; "
                    f"retrying ({attempt + 1}/{PING_ATTEMPTS})..."
                )
                time.sleep(PING_RETRY_DELAY)
    raise RuntimeError(
        f"{joint} / ID {motor.motor_id} did not respond after "
        f"{PING_ATTEMPTS} attempts: {last_error}"
    ) from last_error


def offset(raw, zero):
    """Return signed encoder displacement across the 4095/0 boundary."""
    return encoder_offset(raw, zero)


def validate(data, captured):
    """Require measured landmarks and an interval containing zero."""
    missing = set(LANDMARKS.values()) - captured
    if missing:
        raise ValueError("Capture first: " + ", ".join(sorted(missing)))
    for field in LANDMARKS.values():
        if not isinstance(data[field], int) or not 0 <= data[field] <= 4095:
            raise ValueError(f"Invalid {field}")
    low = offset(data["limit_min"], data["zero_raw"])
    high = offset(data["limit_max"], data["zero_raw"])
    if not -2048 < low <= 0 <= high < 2048 or low >= high:
        raise ValueError("L/H must bracket ZERO on raw -/+ sides, each <180 deg")
    if data["direction"] not in (-1, 1):
        raise ValueError("Direction must be +1 or -1")
    return low, high


def run_session(motors, calibration):
    """Interact with already connected motors without implicit torque writes."""
    data = {}
    captured = {}
    for info in MOTORS:
        mid = info["motor_id"]
        previous = calibration.get_motor(mid)
        data[mid] = dict(joint=info["joint"], zero_raw=None,
                         limit_min=None, limit_max=None, direction=1)
        captured[mid] = set()
        if previous is not None:
            print(f"Previous ID {mid}: {vars(previous)} (recapture Z/L/H)")
            if previous.joint == info["joint"] and previous.direction in (-1, 1):
                data[mid]["direction"] = previous.direction

    index = 0
    print("Torque is preserved, including on exit. Support arm before T/OFF.")
    print("Capture Z/L/H for each motor; L/H follow raw -/+ sides of zero.")
    print("T torque | A/D jog | Z zero | L min | H max | R reverse | N next | S save | Q quit")
    while True:
        info = MOTORS[index]
        mid = info["motor_id"]
        motor = motors[mid]
        current = data[mid]
        raw = read_raw(motor)
        torque = motor.is_torque_enabled()
        angle = None
        if current["zero_raw"] is not None:
            angle = MotorCalibration(**current).raw_to_angle(raw)
        print(f"{info['joint']} ID {mid}: raw={raw}, angle={angle}, torque={torque}")
        print(f"Landmarks: {current}; captured={sorted(captured[mid])}")
        print("Key > ", end="", flush=True)
        key = get_key().lower()
        print()
        if key in ("q", "\x03", ""):
            return
        try:
            if key in LANDMARKS:
                # Read AFTER the key: the arm may have moved while waiting.
                current[LANDMARKS[key]] = read_raw(motor)
                captured[mid].add(LANDMARKS[key])
            elif key == "r":
                current["direction"] *= -1
            elif key == "n":
                index = (index + 1) % len(MOTORS)
            elif key == "t":
                if motor.is_torque_enabled():
                    motor.disable_torque()
                else:
                    # Replace any stale goal with the current encoder position
                    # before enabling holding torque.
                    motor.set_position(read_raw(motor))
                    motor.enable_torque()
            elif key in ("a", "d"):
                low, high = validate(current, captured[mid])
                if not motor.is_torque_enabled():
                    raise ValueError("Torque is OFF; use T explicitly to enable")
                raw = read_raw(motor)
                start = offset(raw, current["zero_raw"])
                target_offset = start + (STEP if key == "d" else -STEP)
                if not low <= start <= high or not low <= target_offset <= high:
                    raise ValueError("Jog would exceed calibrated limits")
                target = raw + (STEP if key == "d" else -STEP)
                if not 0 <= target <= 4095:
                    raise ValueError("Jog across encoder seam disabled; capture manually")
                motor.set_position(target)
            elif key == "s":
                for info in MOTORS:
                    validate(data[info["motor_id"]], captured[info["motor_id"]])
                for info in MOTORS:
                    mid = info["motor_id"]
                    calibration.set_motor(motor_id=mid, **data[mid])
                calibration.save()
                print(f"All calibration saved: {calibration.calibration_file}")
        except ValueError as exc:
            print(f"Cannot apply action: {exc}")


def main():
    """Own one connection and close it on every session exit."""
    protocol = None
    try:
        calibration = CalibrationManager(CALIBRATION_FILE)
        calibration.load()
        protocol = FeetechProtocol(port=PORT, baudrate=BAUDRATE)
        motors = {}
        for info in MOTORS:
            mid = info["motor_id"]
            motor = STS3250(motor_id=mid, protocol=protocol)
            connect_motor(motor, info["joint"])
            motors[mid] = motor
        run_session(motors, calibration)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Calibration stopped: {exc}")
    finally:
        if protocol is not None:
            protocol.close()
        print("Calibration closed. No torque change requested on exit.")


if __name__ == "__main__":
    main()
