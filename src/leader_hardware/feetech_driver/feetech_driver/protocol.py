import threading

import serial


class FeetechProtocolError(Exception):
    """Base exception for Feetech protocol errors."""


class FeetechTimeoutError(FeetechProtocolError):
    """Raised when the motor does not respond in time."""


class FeetechResponseError(FeetechProtocolError):
    """Raised when the motor response is invalid."""


class FeetechMotorError(FeetechProtocolError):
    """Raised when the motor reports a protocol error."""


class FeetechProtocol:
    """Low-level communication with Feetech STS-series motors."""

    HEADER = bytes([0xFF, 0xFF])

    PING = 0x01
    READ = 0x02
    WRITE = 0x03

    def __init__(
        self,
        port="/dev/ttyUSB0",
        baudrate=1000000,
        timeout=0.2,
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self.serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
        )

        # Prevent two commands from using the half-duplex bus at the same time.
        self._lock = threading.Lock()

    def close(self):
        """Close serial port."""
        if self.serial.is_open:
            self.serial.close()

    @staticmethod
    def checksum(values):
        """
        Calculate Feetech checksum.

        Checksum = bitwise NOT of the sum of all bytes,
        keeping only the lowest 8 bits.
        """
        return (~sum(values)) & 0xFF

    @staticmethod
    def int_to_bytes(value, size):
        """Convert integer to little-endian byte list."""
        return [(value >> (8 * i)) & 0xFF for i in range(size)]

    @staticmethod
    def bytes_to_int(values):
        """Convert little-endian byte list to integer."""
        result = 0
        for i, value in enumerate(values):
            result |= value << (8 * i)
        return result

    @staticmethod
    def decode_signed_magnitude(value):
        """
        Decode Feetech signed-magnitude values.

        Bit 15 is the direction bit.
        Lower 15 bits contain magnitude.
        """
        direction = (value >> 15) & 0x01
        magnitude = value & 0x7FFF

        if direction:
            return -magnitude

        return magnitude

    def _build_packet(self, motor_id, instruction, parameters=None):
        """Build a Feetech instruction packet."""
        if parameters is None:
            parameters = []

        length = 2 + len(parameters)

        data = [
            motor_id,
            length,
            instruction,
            *parameters,
        ]

        checksum = self.checksum(data)

        return bytes([
            *self.HEADER,
            *data,
            checksum,
        ])

    def _read_response(self, expected_motor_id):
        """Read and validate one motor response."""
        response = self.serial.read(4)

        if len(response) < 4:
            raise FeetechTimeoutError(
                f"No response from motor {expected_motor_id}"
            )

        # Header check
        if response[0] != 0xFF or response[1] != 0xFF:
            raise FeetechResponseError(
                f"Invalid header: {response.hex()}"
            )

        motor_id = response[2]
        length = response[3]

        if motor_id != expected_motor_id:
            raise FeetechResponseError(
                f"Unexpected motor ID: expected {expected_motor_id}, got {motor_id}"
            )

        # We already received 4 bytes: FF FF ID LENGTH
        remaining = length
        payload = self.serial.read(remaining)

        if len(payload) != remaining:
            raise FeetechTimeoutError("Incomplete motor response")

        packet = response + payload
        expected_length = 4 + length

        if len(packet) != expected_length:
            raise FeetechResponseError("Invalid packet length")

        received_checksum = packet[-1]
        checksum_data = list(packet[2:-1])
        calculated_checksum = self.checksum(checksum_data)

        if received_checksum != calculated_checksum:
            raise FeetechResponseError(
                f"Checksum error: expected 0x{calculated_checksum:02X}, got 0x{received_checksum:02X}"
            )

        error = packet[4]
        if error != 0:
            raise FeetechMotorError(
                f"Motor {motor_id} reported error 0x{error:02X}"
            )

        # Return parameter bytes
        return packet[5:-1]

    def ping(self, motor_id):
        """Ping a motor."""
        packet = self._build_packet(motor_id, self.PING)

        with self._lock:
            self.serial.reset_input_buffer()
            self.serial.write(packet)
            self.serial.flush()

            self._read_response(motor_id)

        return True

    def read(self, motor_id, address, size):
        """Read registers from a motor."""
        if size <= 0:
            raise ValueError("Read size must be positive")

        if not 0 <= address <= 255:
            raise ValueError("Address must be 0-255")

        if not 0 <= motor_id <= 253:
            raise ValueError("Motor ID must be 0-253")

        packet = self._build_packet(
            motor_id,
            self.READ,
            [address, size],
        )

        with self._lock:
            self.serial.reset_input_buffer()
            self.serial.write(packet)
            self.serial.flush()

            data = self._read_response(motor_id)

        if len(data) != size:
            raise FeetechResponseError(
                f"Expected {size} data bytes, got {len(data)}"
            )

        return data

    def write(self, motor_id, address, values):
        """Write register values to a motor."""

        if not 0 <= address <= 255:
            raise ValueError("Address must be 0-255")

        if not 0 <= motor_id <= 253:
            raise ValueError("Motor ID must be 0-253")

        values = list(values)

        for value in values:
            if not 0 <= value <= 255:
                raise ValueError(
                    f"Invalid byte value: {value}"
                )

        packet = self._build_packet(
            motor_id,
            self.WRITE,
            [address, *values],
        )

        print("TX WRITE:", packet.hex(" "))

        with self._lock:
            self.serial.reset_input_buffer()

            self.serial.write(packet)
            self.serial.flush()

            response = self._read_response(motor_id)

        print("RX WRITE:", response.hex(" "))

        return response