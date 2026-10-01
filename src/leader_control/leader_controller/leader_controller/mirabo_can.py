"""Mirabo CAN wire format and SocketCAN transport."""

from dataclasses import dataclass
import math
import struct

import can

from leader_controller import mapping


CAN_ERR_BUSOFF = 0x40
COMMAND_PAYLOAD_FORMAT = '>ihh'


def validate_command_field(value, name):
    """Validate the C++ int16 input and its existing signed int16 encoding."""
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f'{name} must be a positive integer')
    if value > 32767:
        raise ValueError(f'{name} exceeds signed int16 source range')
    raw = int(value / 10.0)
    if not 1 <= raw <= 32767:
        raise ValueError(
            f'{name} encodes to {raw}, outside positive signed int16 range'
        )
    return raw


def pack_position_command(angle_deg, speed, accel):
    """Pack the verified Mirabo position command payload."""
    if not all(math.isfinite(value) for value in (angle_deg, speed, accel)):
        raise ValueError('CAN command values must be finite')
    try:
        cpp_angle = struct.unpack('>f', struct.pack('>f', angle_deg))[0]
        return struct.pack(
            COMMAND_PAYLOAD_FORMAT, int(cpp_angle * 10000.0),
            int(speed / 10.0), int(accel / 10.0),
        )
    except (struct.error, OverflowError) as exc:
        raise ValueError('CAN command exceeds its integer range') from exc


def unpack_feedback(data):
    """Decode Mirabo angle in CAN degrees and fault byte."""
    if len(data) != 8:
        raise ValueError('Mirabo feedback payload must contain 8 bytes')
    raw_angle = struct.unpack('>h', bytes(data[:2]))[0]
    return raw_angle / 10.0, int(data[7])


@dataclass(frozen=True)
class CanEvent:
    """One decoded receive event, independent of ROS and teleop mapping."""

    kind: str
    arbitration_id: int
    angle_deg: float = math.nan
    fault: int = 0
    detail: str = ''


class MiraboCanDriver:
    """Own the CAN socket and decode frames from the configured motors."""

    def __init__(self, feedback_ids, bus=None, interface=mapping.CAN_INTERFACE,
                 bitrate=mapping.CAN_BITRATE):
        self.bus = bus if bus is not None else can.Bus(
            interface='socketcan',
            channel=interface,
            bitrate=bitrate,
            ignore_rx_error_frames=False,
            ignore_config=True,
        )
        self.feedback_ids = set(feedback_ids)

    def read_events(self):
        """Read one bounded batch; remaining frames wait for the next tick."""
        events = []
        for _ in range(mapping.MAX_CAN_FRAMES_PER_READ):
            frame = self.bus.recv(timeout=0.0)
            if frame is None:
                break
            if frame.is_error_frame:
                events.append(CanEvent(
                    'bus_off' if frame.arbitration_id & CAN_ERR_BUSOFF
                    else 'error', frame.arbitration_id,
                    detail=frame.data.hex(),
                ))
                if events[-1].kind == 'bus_off':
                    break
                continue
            if not frame.is_extended_id or frame.arbitration_id not in self.feedback_ids:
                continue
            if frame.is_remote_frame or frame.is_fd or frame.dlc != 8:
                events.append(CanEvent('malformed', frame.arbitration_id))
                continue
            try:
                angle_deg, fault = unpack_feedback(frame.data)
            except ValueError as exc:
                events.append(CanEvent('malformed', frame.arbitration_id,
                                       detail=str(exc)))
                continue
            events.append(CanEvent('feedback', frame.arbitration_id,
                                   angle_deg=angle_deg, fault=fault))
        return events

    def send_position(self, command_id, angle_deg, speed, accel):
        """Send a verified extended CAN position frame."""
        self.bus.send(can.Message(
            arbitration_id=command_id,
            data=pack_position_command(angle_deg, speed, accel),
            is_extended_id=mapping.COMMAND_IS_EXTENDED_ID,
        ), timeout=0.0)

    def shutdown(self):
        self.bus.shutdown()
