"""
High-level STS3250 motor interface with physics conversions and validation.
"""

from . import control_table as ct
from .protocol import FeetechProtocol


class STS3250:
    """High-level interface for one STS3250 servo motor."""

    def __init__(
        self,
        port="/dev/ttyUSB0",
        baudrate=1000000,
        motor_id=3,
        protocol=None,
    ):
        self.motor_id = motor_id
        self._owns_protocol = protocol is None

        if protocol is None:
            self.protocol = FeetechProtocol(
                port=port,
                baudrate=baudrate,
            )
        else:
            self.protocol = protocol

    def close(self):
        """Only close a connection created by this motor object."""
        if self._owns_protocol:
            self.protocol.close()

    def ping(self):
        """Ping motor and validate connection."""
        response = self.protocol.ping(self.motor_id)
        return bool(response)

    # ==========================================================
    # RAW READ METHODS
    # ==========================================================

    def read_raw_position(self):
        """Read present position (0 - 4095)."""
        data = self.protocol.read(
            self.motor_id,
            ct.PRESENT_POSITION,
            2,
        )
        if not data:
            return None
        return self.protocol.bytes_to_int(data)

    def read_raw_velocity(self):
        """Read present velocity (raw 16-bit value)."""
        data = self.protocol.read(
            self.motor_id,
            ct.PRESENT_VELOCITY,
            2,
        )
        if not data:
            return None
        return self.protocol.bytes_to_int(data)

    def read_raw_load(self):
        """Read present load (raw 16-bit value)."""
        data = self.protocol.read(
            self.motor_id,
            ct.PRESENT_LOAD,
            2,
        )
        if not data:
            return None
        return self.protocol.bytes_to_int(data)

    def read_raw_voltage(self):
        """Read present voltage in 0.1V units (e.g., 120 = 12.0V)."""
        data = self.protocol.read(
            self.motor_id,
            ct.PRESENT_VOLTAGE,
            1,
        )
        if not data:
            return None
        return data[0]

    # ==========================================================
    # CONVERTED / PHYSICAL VALUES
    # ==========================================================

    def read_position(self):
        """Read present position converted to Degrees (0.0° - 360.0°)."""
        raw = self.read_raw_position()
        if raw is None:
            return None
        return round((raw / 4095.0) * 360.0, 2)

    def read_velocity(self):
        """Read present velocity converted to Signed Int (Direction + Speed)."""
        raw = self.read_raw_velocity()
        if raw is None:
            return None
        if raw > 32767:
            return -(raw - 32768)
        return raw

    def read_load(self):
        """Read present load converted to Signed Int (Direction + Torque Load)."""
        raw = self.read_raw_load()
        if raw is None:
            return None
        if raw > 1024:
            return -(raw - 1024)
        return raw

    def read_voltage(self):
        """Read present voltage converted to Volts (V)."""
        raw = self.read_raw_voltage()
        if raw is None:
            return None
        return raw / 10.0

    def read_temperature(self):
        """Read present temperature in Celsius (°C)."""
        data = self.protocol.read(
            self.motor_id,
            ct.PRESENT_TEMPERATURE,
            1,
        )
        if not data:
            return None
        return data[0]

    def read_current(self):
        """Read present current in raw Feetech units."""
        data = self.protocol.read(
            self.motor_id,
            ct.PRESENT_CURRENT,
            2,
        )
        if not data:
            return None
        return self.protocol.bytes_to_int(data)

    def read_status(self):
        """Read servo hardware/error status byte."""
        data = self.protocol.read(
            self.motor_id,
            ct.STATUS,
            1,
        )
        if not data:
            return None
        return data[0]

    def is_moving(self):
        """Return True when the servo is currently moving."""
        data = self.protocol.read(
            self.motor_id,
            ct.MOVING,
            1,
        )
        if not data:
            return False
        return bool(data[0])

    # ==========================================================
    # CONTROL METHODS
    # ==========================================================

    def is_torque_enabled(self):
        """Check if motor torque is enabled."""
        data = self.protocol.read(
            self.motor_id,
            ct.TORQUE_ENABLE,
            1,
        )
        if not data:
            return False
        return bool(data[0])

    def enable_torque(self):
        """Enable motor torque."""
        return self.protocol.write(
            self.motor_id,
            ct.TORQUE_ENABLE,
            [1],
        )

    def disable_torque(self):
        """Disable motor torque."""
        return self.protocol.write(
            self.motor_id,
            ct.TORQUE_ENABLE,
            [0],
        )
    def set_velocity(self, velocity):
        """Set goal velocity using raw Feetech velocity units."""

        velocity = int(velocity)

        if velocity < -32767 or velocity > 32767:
            raise ValueError(
                "Velocity must be between -32767 and 32767"
            )

        # Feetech STS velocity:
        # bit 15 = direction
        # bits 0-14 = magnitude
        if velocity < 0:
            raw_velocity = (-velocity) | 0x8000
        else:
            raw_velocity = velocity

        data = self.protocol.int_to_bytes(
            raw_velocity,
            2,
        )

        return self.protocol.write(
            self.motor_id,
            ct.GOAL_VELOCITY,
            data,
        )
    def set_position(self, position):
        """Set goal position (raw 0 - 4095 steps)."""
        position = int(position)

        if not (ct.MIN_POSITION <= position <= ct.MAX_POSITION):
            raise ValueError(
                f"Position must be between {ct.MIN_POSITION} and {ct.MAX_POSITION}"
            )

        data = self.protocol.int_to_bytes(
            position,
            2,
        )

        return self.protocol.write(
            self.motor_id,
            ct.GOAL_POSITION,
            data,
        )

    def set_angle(self, angle_deg):
        """Set goal position using angle in Degrees (0° - 360°)."""
        if not (0 <= angle_deg <= 360):
            raise ValueError("Angle must be between 0 and 360 degrees")

        raw_position = int((angle_deg / 360.0) * 4095.0)
        return self.set_position(raw_position)