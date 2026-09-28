"""
Calibration utilities for Feetech leader arm.

This module stores and converts motor encoder calibration data.
"""

import json
from pathlib import Path


def encoder_offset(raw, zero_raw):
    """Return the shortest signed offset on a 4096-count encoder."""
    if not all(isinstance(value, int) and 0 <= value < 4096
               for value in (raw, zero_raw)):
        raise ValueError("Encoder positions must be integers from 0 to 4095")
    return (raw - zero_raw + 2048) % 4096 - 2048


class MotorCalibration:
    """Calibration data for one motor."""

    def __init__(
        self,
        joint,
        zero_raw=0,
        direction=1,
        limit_min=0,
        limit_max=4095,
    ):
        self.joint = joint
        self.zero_raw = zero_raw
        self.direction = direction
        self.limit_min = limit_min
        self.limit_max = limit_max

    def raw_to_angle(self, raw):
        """
        Convert raw encoder position to joint angle in degrees.
        """

        # Shortest signed displacement; intended for joints within half a
        # revolution of zero. Continuous multi-turn tracking needs history.
        delta = encoder_offset(raw, self.zero_raw)

        return (
            delta
            * self.direction
            * 360.0
            / 4096.0
        )

    def angle_to_raw(self, angle):
        """
        Convert joint angle in degrees to raw encoder position.
        """

        raw = (
            self.zero_raw
            + (
                angle
                * self.direction
                * 4096.0
                / 360.0
            )
        )

        return int(round(raw)) % 4096


class CalibrationManager:
    """Load, save and manage leader arm calibration."""

    def __init__(self, calibration_file):
        self.calibration_file = Path(calibration_file)
        self.motors = {}

    def load(self):
        """Load calibration data from JSON."""

        if not self.calibration_file.exists():
            return False

        with open(
            self.calibration_file,
            "r",
            encoding="utf-8",
        ) as file:
            data = json.load(file)

        self.motors = {}

        for motor_id, config in data.get(
            "motors",
            {},
        ).items():

            self.motors[int(motor_id)] = MotorCalibration(
                joint=config["joint"],
                zero_raw=config["zero_raw"],
                direction=config["direction"],
                limit_min=config["limit_min"],
                limit_max=config["limit_max"],
            )

        return True

    def save(self):
        """Save calibration data to JSON."""

        self.calibration_file.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        data = {
            "motors": {}
        }

        for motor_id, calibration in self.motors.items():

            data["motors"][str(motor_id)] = {
                "joint": calibration.joint,
                "zero_raw": calibration.zero_raw,
                "direction": calibration.direction,
                "limit_min": calibration.limit_min,
                "limit_max": calibration.limit_max,
            }

        with open(
            self.calibration_file,
            "w",
            encoding="utf-8",
        ) as file:

            json.dump(
                data,
                file,
                indent=4,
            )

    def set_motor(
        self,
        motor_id,
        joint,
        zero_raw,
        direction=1,
        limit_min=0,
        limit_max=4095,
    ):
        """Create or update calibration for one motor."""

        self.motors[motor_id] = MotorCalibration(
            joint=joint,
            zero_raw=zero_raw,
            direction=direction,
            limit_min=limit_min,
            limit_max=limit_max,
        )

    def get_motor(self, motor_id):
        """Get calibration for a motor."""

        return self.motors.get(motor_id)
