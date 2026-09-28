"""Exercise Mirabo teleop with real ROS messages and an in-memory CAN bus."""

from collections import deque
import math
import struct
from types import SimpleNamespace

import can
from leader_controller import mapping, teleop
import pytest
import rclpy
from rclpy.clock import ClockType
from rclpy.context import Context
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState


class FakeBus:
    """Capture all writes without opening SocketCAN or serial hardware."""

    def __init__(self):
        self.messages = deque()
        self.sent = []
        self.state = can.BusState.ACTIVE
        self.receive_error = None
        self.send_error = None
        self.closed = False

    def recv(self, timeout):
        assert timeout == 0.0
        if self.receive_error:
            raise self.receive_error
        return self.messages.popleft() if self.messages else None

    def send(self, message, timeout):
        assert timeout == 0.0
        if self.send_error:
            raise self.send_error
        self.sent.append(message)

    def shutdown(self):
        self.closed = True


@pytest.fixture
def rig(monkeypatch):
    """Create an isolated ROS node with no access to the physical CAN bus."""
    bus = FakeBus()
    opened = []

    def open_bus(**kwargs):
        opened.append(kwargs)
        return bus

    monkeypatch.setattr(teleop.can, 'Bus', open_bus)
    context = Context()
    rclpy.init(args=[], context=context, domain_id=231)
    node = teleop.MiraboTeleop(
        context=context, use_global_arguments=False,
        parameter_overrides=[Parameter('armed', value=True)],
    )
    clock = SimpleNamespace(value=100.0)
    monkeypatch.setattr(node, 'now_sec', lambda: clock.value)
    status, feedback = [], []
    node.status_pub = SimpleNamespace(publish=status.append)
    node.feedback_pub = SimpleNamespace(publish=feedback.append)
    try:
        yield SimpleNamespace(
            node=node, bus=bus, clock=clock, status=status,
            feedback=feedback, opened=opened,
        )
    finally:
        count = len(bus.sent)
        node.destroy_node()
        context.shutdown()
        assert bus.closed
        assert len(bus.sent) == count


def feedback_frame(feedback_id, angle=0.0, fault=0):
    return can.Message(
        arbitration_id=feedback_id, is_extended_id=True,
        data=struct.pack('>h', int(round(angle * 10))) + bytes(5)
        + bytes([fault]),
    )


def leader_sample(rig, j2=10.0, j3=20.0):
    msg = JointState()
    msg.name = ['joint_3', 'joint_1', 'joint_2']
    msg.position = [math.radians(j3), 1.0, math.radians(j2)]
    rig.node.leader_callback(msg)


def fresh_samples(rig, j2=10.0, j3=20.0, motor2=30.0, motor3=40.0):
    rig.clock.value += 0.05
    leader_sample(rig, j2, j3)
    rig.bus.messages.extend([
        feedback_frame(0x2968, motor2), feedback_frame(0x2969, motor3),
    ])
    rig.node.read_can_feedback()


def set_armed(rig, value):
    return rig.node.set_parameters([Parameter('armed', value=value)])[0]


def start(rig):
    fresh_samples(rig)
    assert set_armed(rig, True).successful
    assert not rig.bus.sent
    rig.node.control_step()


def command_angles(rig):
    return [struct.unpack('>ihh', msg.data)[0] / 10000.0
            for msg in rig.bus.sent[-2:]]


@pytest.mark.parametrize('angle,speed,accel,expected', [
    (12.3456, 1000, 1000, '0001e24000640064'),
    (-1.5, 1000, 1000, 'ffffc56800640064'),
    (-1.23459, 1099, 1099, 'ffffcfc7006d006d'),
])
def test_cpp_payload(angle, speed, accel, expected):
    assert teleop.pack_position_command(angle, speed, accel).hex() == expected


@pytest.mark.parametrize('angle', [math.nan, math.inf, 300000.0])
def test_invalid_command_is_rejected(angle):
    with pytest.raises(ValueError):
        teleop.pack_position_command(angle, 1000, 1000)


@pytest.mark.parametrize('raw,angle', [
    (0, 0.0), (-100, -10.0), (-32768, -3276.8), (32767, 3276.7),
])
def test_signed_feedback(raw, angle):
    data = struct.pack('>h', raw) + bytes(5) + bytes([5])
    assert teleop.unpack_feedback(data) == (angle, 5)


@pytest.mark.parametrize('length', [0, 7, 9])
def test_feedback_length(length):
    with pytest.raises(ValueError):
        teleop.unpack_feedback(bytes(length))


def test_startup_override_cannot_arm_and_reading_never_sends(rig):
    fresh_samples(rig)
    for _ in range(3):
        rig.node.control_step()
    assert not rig.node.get_parameter('armed').value
    assert rig.bus.sent == []
    assert 'state=READY' in rig.status[-1].data
    assert '0x68=30.0CANdeg' in rig.status[-1].data
    assert rig.opened[0]['channel'] == 'can0'
    assert rig.opened[0]['bitrate'] == 1000000
    assert not rig.opened[0]['ignore_rx_error_frames']
    assert rig.node.control_timer.timer_period_ns == 50_000_000
    assert rig.node.steady_clock.clock_type == ClockType.STEADY_TIME
    with pytest.raises(ValueError, match='disarmed'):
        rig.node.send_position_command(mapping.MIRABO_JOINTS[0], 30.0)
    assert not rig.bus.sent


@pytest.mark.parametrize('missing', ['leader', 0x2968, 0x2969])
def test_refuses_arming_without_every_input(rig, missing):
    fresh_samples(rig)
    if missing == 'leader':
        rig.node.leader_stamp_sec = -math.inf
    else:
        rig.node.feedback_by_id[missing].stamp_sec = -math.inf
    assert not set_armed(rig, True).successful
    rig.node.control_step()
    assert not rig.bus.sent


def test_latches_both_motors_by_joint_name_and_preserves_zero(rig):
    start(rig)
    assert command_angles(rig) == [30.0, 40.0]
    assert [msg.arbitration_id for msg in rig.bus.sent] == [0x668, 0x669]
    assert all(msg.is_extended_id and msg.dlc == 8 for msg in rig.bus.sent)
    fresh_samples(rig, j2=11.0, j3=21.0)
    rig.node.control_step()
    assert command_angles(rig) == pytest.approx([31.0, 39.0], abs=0.0001)


def test_requires_new_samples_from_all_three_inputs(rig):
    start(rig)
    rig.clock.value += 0.05
    leader_sample(rig, j2=50.0, j3=50.0)
    rig.bus.messages.append(feedback_frame(0x2968, 30.0))
    rig.node.control_step()
    assert len(rig.bus.sent) == 2
    rig.bus.messages.append(feedback_frame(0x2969, 40.0))
    rig.node.control_step()
    assert len(rig.bus.sent) == 4
    assert command_angles(rig) == [32.0, 38.0]


def test_step_rate_and_both_software_limits(rig):
    start(rig)
    previous = command_angles(rig)
    for j2, j3, expected in [(1000.0, -1000.0, [90.0, 90.0]),
                             (-1000.0, 1000.0, [-15.0, 0.0])]:
        for _ in range(65):
            fresh_samples(rig, j2=j2, j3=j3)
            rig.node.control_step()
            current = command_angles(rig)
            for index, item in enumerate(mapping.MIRABO_JOINTS):
                assert item.min_deg <= current[index] <= item.max_deg
                assert abs(current[index] - previous[index]) <= 2.0001
            previous = current
        assert current == expected


@pytest.mark.parametrize('feedback_id,angle', [
    (0x2968, -15.1), (0x2968, 90.1), (0x2969, -0.1), (0x2969, 90.1),
])
def test_out_of_range_baseline_cannot_move_towards_limits(
    rig, feedback_id, angle,
):
    fresh_samples(rig)
    rig.bus.messages.append(feedback_frame(feedback_id, angle))
    rig.node.read_can_feedback()
    result = set_armed(rig, True)
    assert not result.successful
    assert 'outside software limits' in result.reason
    rig.node.control_step()
    assert not rig.bus.sent


@pytest.mark.parametrize('missing', ['leader', 0x2968, 0x2969])
def test_timeout_disarms_both_and_never_auto_rearms(rig, missing):
    start(rig)
    rig.clock.value += 0.51
    if missing != 'leader':
        leader_sample(rig)
    for fid in (0x2968, 0x2969):
        if fid != missing:
            rig.bus.messages.append(feedback_frame(fid, 30.0))
    rig.node.control_step()
    assert len(rig.bus.sent) == 2
    assert not rig.node.get_parameter('armed').value
    assert not rig.node.internal_armed
    assert 'state=FAULT' in rig.status[-1].data
    assert 'timeout' in rig.status[-1].data
    fresh_samples(rig, j2=60.0, j3=70.0, motor2=50.0, motor3=60.0)
    rig.node.control_step()
    assert len(rig.bus.sent) == 2
    assert set_armed(rig, True).successful
    rig.node.control_step()
    assert command_angles(rig) == [50.0, 60.0]


@pytest.mark.parametrize('source', [
    'error_frame', 'receive', 'send', 'passive', 'bus_off', 'motor68', 'motor69',
])
def test_can_and_motor_faults_stop_both(rig, source):
    start(rig)
    fresh_samples(rig)
    if source == 'error_frame':
        rig.bus.messages.append(can.Message(
            arbitration_id=0x40, is_extended_id=False,
            is_error_frame=True, data=bytes(8),
        ))
    elif source == 'receive':
        rig.bus.receive_error = can.CanOperationError('receive failure')
    elif source == 'send':
        rig.bus.send_error = can.CanOperationError('send failure')
    elif source == 'passive':
        rig.bus.state = can.BusState.PASSIVE
    elif source == 'bus_off':
        rig.bus.state = can.BusState.ERROR
    else:
        fid = 0x2968 if source == 'motor68' else 0x2969
        rig.bus.messages.append(feedback_frame(fid, 30.0, fault=5))
    rig.node.control_step()
    assert len(rig.bus.sent) == 2
    assert not rig.node.get_parameter('armed').value
    assert 'state=FAULT' in rig.status[-1].data
    assert 'reason=' in rig.status[-1].data


@pytest.mark.parametrize('kind', ['short', 'remote', 'fd'])
def test_malformed_can_feedback_disarms_without_crashing(rig, kind):
    start(rig)
    message = feedback_frame(0x2968, 30.0)
    if kind == 'short':
        message.data = bytearray(2)
    elif kind == 'remote':
        message.is_remote_frame = True
    else:
        message.is_fd = True
    rig.bus.messages.append(message)
    rig.node.control_step()
    assert not rig.node.get_parameter('armed').value
    assert 'invalid CAN feedback' in rig.status[-1].data
    assert len(rig.bus.sent) == 2


@pytest.mark.parametrize('names,positions', [
    (['joint_2', 'joint_2', 'joint_3'], [0.0, 0.0, 0.0]),
    (['joint_2'], [0.0]),
    (['joint_2', 'joint_3'], [0.0]),
    (['joint_2', 'joint_3'], [math.nan, 0.0]),
    (['joint_2', 'joint_3'], [0.0, math.inf]),
])
def test_invalid_leader_invalidates_previous_sample(rig, names, positions):
    start(rig)
    rig.node.leader_callback(JointState(name=names, position=positions))
    assert not rig.node.get_parameter('armed').value
    assert not set_armed(rig, True).successful
    rig.node.control_step()
    assert len(rig.bus.sent) == 2


@pytest.mark.parametrize('shaft_angle,scale', [(False, 1.0), (True, 8.0)])
def test_explicit_gear_scaling_for_commands_and_feedback(
    rig, monkeypatch, shaft_angle, scale,
):
    monkeypatch.setattr(mapping, 'CAN_ANGLE_IS_MOTOR_SHAFT', shaft_angle)
    start(rig)
    fresh_samples(rig, j2=10.1, j3=20.1)
    rig.node.control_step()
    assert command_angles(rig) == pytest.approx(
        [30.0 + 0.1 * scale, 40.0 - 0.1 * scale], abs=0.00010001,
    )
    assert rig.feedback[-1].position == pytest.approx([
        math.radians(30.0 / scale), math.radians(40.0 / scale),
    ])


def test_manual_disarm_sends_nothing_and_rearm_relatches(rig):
    start(rig)
    assert set_armed(rig, False).successful
    fresh_samples(rig, j2=50.0, j3=60.0, motor2=15.0, motor3=25.0)
    rig.node.control_step()
    assert len(rig.bus.sent) == 2
    assert set_armed(rig, True).successful
    rig.node.control_step()
    assert command_angles(rig) == [15.0, 25.0]


def test_unrelated_can_traffic_is_ignored_but_backlog_is_bounded(rig):
    fresh_samples(rig)
    rig.bus.messages.append(can.Message(
        arbitration_id=0x68, is_extended_id=False, data=bytes(8),
    ))
    rig.bus.messages.append(feedback_frame(0x2868, -100.0, fault=5))
    rig.node.read_can_feedback()
    assert rig.node.ready()[0]
    rig.bus.messages.extend(feedback_frame(0x1234)
                            for _ in range(mapping.MAX_CAN_FRAMES_PER_READ))
    rig.node.control_step()
    assert 'backlog' in rig.status[-1].data
    assert not rig.bus.sent
