"""Check SocketCAN flag decoding before Mirabo event classification."""

import socket
import struct

from can.interfaces.socketcan import constants, socketcan
from leader_controller.mirabo_can import MiraboCanDriver
import pytest


class RawSocket:
    def __init__(self, can_id, data):
        self.frame = struct.pack('=IBB2x', can_id, len(data), 0) + data

    def recvmsg(self, *_args):
        timestamp = socketcan.RECEIVED_TIMESTAMP_STRUCT.pack(0, 0)
        ancillary = [(socket.SOL_SOCKET, constants.SO_TIMESTAMPNS, timestamp)]
        return self.frame, ancillary, 0, ('can0',)


class FakeBus:
    def __init__(self, messages):
        self.messages = list(messages)

    def recv(self, timeout):
        assert timeout == 0.0
        return self.messages.pop(0) if self.messages else None


def decode(can_id, data=bytes(8)):
    return socketcan.capture_message(RawSocket(can_id, data))


def test_socketcan_flag_and_mask_constants():
    assert constants.CAN_ERR_FLAG == 0x20000000
    assert constants.CAN_EFF_FLAG == 0x80000000
    assert constants.CAN_RTR_FLAG == 0x40000000
    assert constants.MSK_ARBID == 0x1FFFFFFF
    assert (constants.CAN_EFF_FLAG | 0x2968) & constants.MSK_ARBID == 0x2968
    assert (constants.CAN_ERR_FLAG | 0xC) & constants.MSK_ARBID == 0xC


@pytest.mark.parametrize('arbitration_id', [0x2968, 0x2969, 0x668, 0x669])
def test_normal_extended_frames_are_not_errors(arbitration_id):
    raw_id = constants.CAN_EFF_FLAG | arbitration_id
    frame = decode(raw_id)
    assert raw_id & constants.CAN_ERR_FLAG == 0
    assert frame.arbitration_id == arbitration_id
    assert frame.is_extended_id
    assert not frame.is_error_frame


@pytest.mark.parametrize('error_class,kind', [
    (0x4, 'error'), (0xC, 'error'), (0x40, 'bus_off'),
])
def test_error_flag_creates_error_event(error_class, kind):
    raw_id = constants.CAN_ERR_FLAG | error_class
    frame = decode(raw_id)
    assert raw_id & constants.CAN_ERR_FLAG
    assert frame.is_error_frame
    assert frame.arbitration_id == error_class
    events = MiraboCanDriver({0x2968, 0x2969}, FakeBus([frame])).read_events()
    assert [event.kind for event in events] == [kind]


def test_only_extended_feedback_ids_create_feedback_events():
    frames = [decode(constants.CAN_EFF_FLAG | arbitration_id)
              for arbitration_id in (0x2968, 0x2969, 0x668, 0x669)]
    frames.append(decode(0x668))
    events = MiraboCanDriver({0x2968, 0x2969}, FakeBus(frames)).read_events()
    assert [(event.kind, event.arbitration_id) for event in events] == [
        ('feedback', 0x2968), ('feedback', 0x2969),
    ]


def test_error_class_bits_without_error_flag_are_not_warnings():
    frame = decode(constants.CAN_EFF_FLAG | 0x2968)
    assert frame.arbitration_id & 0x40
    assert not frame.is_error_frame
    events = MiraboCanDriver({0x2968}, FakeBus([frame])).read_events()
    assert [event.kind for event in events] == ['feedback']
