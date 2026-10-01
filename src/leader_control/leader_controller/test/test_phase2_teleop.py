"""Check bumpless arm and explicit arming against an in-memory CAN bus."""

import json
import math
from pathlib import Path
import struct
from types import SimpleNamespace
from unittest.mock import Mock

import can
from leader_controller import mapping, teleop
import pytest
import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
import yaml


class FakeBus:
    def __init__(self):
        self.messages = []
        self.sent = []
        self.state = can.BusState.ACTIVE

    def recv(self, timeout):
        assert timeout == 0.0
        return self.messages.pop(0) if self.messages else None

    def send(self, message, timeout):
        assert timeout == 0.0
        self.sent.append(message)

    def shutdown(self):
        pass


def feedback(feedback_id, angle_deg):
    return can.Message(
        arbitration_id=feedback_id, is_extended_id=True,
        data=struct.pack('>h', round(angle_deg * 10)) + bytes(6),
    )


def sample(node, bus, joint2, joint3, motor2, motor3):
    leader = JointState()
    leader.name = ['joint_3', 'joint_2']
    leader.position = [math.radians(joint3), math.radians(joint2)]
    node.leader_callback(leader)
    bus.messages.extend([
        feedback(0x2968, motor2), feedback(0x2969, motor3),
    ])
    node.read_can_feedback()


@pytest.fixture
def rig(monkeypatch, tmp_path):
    bus = FakeBus()
    monkeypatch.setattr(teleop.can, 'Bus', lambda **kwargs: bus)
    context = Context()
    rclpy.init(args=[], context=context, domain_id=231)
    node = teleop.MiraboTeleop(
        context=context, use_global_arguments=False,
        parameter_overrides=[
            Parameter('armed', value=True),
            Parameter('require_origin', value=False),
            Parameter('origin_file', value=str(tmp_path / 'no-origin.json')),
        ],
    )
    clock = SimpleNamespace(now=100.0)
    monkeypatch.setattr(node, 'now_sec', lambda: clock.now)
    try:
        yield node, bus, clock
    finally:
        node.destroy_node()
        context.shutdown()


def test_arm_and_rearm_send_current_follower_position(rig):
    node, bus, clock = rig
    assert node.get_parameter('armed').value is False
    sample(node, bus, 10.0, 20.0, 23.0, -173.0)
    assert node.ready()[0]
    node.control_step()
    assert bus.sent == []
    assert node.set_parameters([Parameter('armed', value=True)])[0].successful
    node.control_step()
    assert [frame.arbitration_id for frame in bus.sent] == [0x668, 0x669]
    commands = [struct.unpack('>ihh', frame.data) for frame in bus.sent]
    assert [entry[0] / 10000.0 for entry in commands] == pytest.approx(
        [23.0, -173.0], abs=0.001,
    )
    assert all(entry[1:] == (100, 100) for entry in commands)
    assert (mapping.COMMAND_SPEED, mapping.COMMAND_ACCEL) == (1000, 1000)

    assert node.set_parameters([Parameter('armed', value=False)])[0].successful
    clock.now += 0.1
    sample(node, bus, 12.0, 22.0, 24.0, -172.0)
    assert node.set_parameters([Parameter('armed', value=True)])[0].successful
    node.control_step()
    new_commands = [struct.unpack('>ihh', frame.data)[0] / 10000.0
                    for frame in bus.sent[-2:]]
    assert new_commands == pytest.approx([24.0, -172.0], abs=0.001)


def test_stale_feedback_blocks_explicit_arm(rig):
    node, bus, clock = rig
    sample(node, bus, 10.0, 20.0, 23.0, -173.0)
    clock.now += node.config['feedback_timeout_sec'] + 0.01
    result = node.set_parameters([Parameter('armed', value=True)])[0]
    assert not result.successful
    assert bus.sent == []


@pytest.mark.parametrize('error_id', [0x4, 0xC])
def test_nonfatal_can_warning_is_counted_without_fault(rig, error_id):
    node, bus, _ = rig
    sample(node, bus, 10.0, 20.0, 23.0, -173.0)
    bus.messages.append(can.Message(
        arbitration_id=error_id, is_error_frame=True, data=bytes(8),
    ))
    node.read_can_feedback()
    assert node.event_counts['can_warning_frames'] == 1
    assert node.fault is None
    assert node.ready()[0]
    assert bus.sent == []


def test_normal_feedback_and_command_ids_do_not_increment_can_warnings(rig):
    node, bus, _ = rig
    bus.messages.extend([
        feedback(0x2968, 23.0), feedback(0x2969, -173.0),
        can.Message(arbitration_id=0x668, is_extended_id=True,
                    data=bytes(8)),
        can.Message(arbitration_id=0x669, is_extended_id=True,
                    data=bytes(8)),
    ])
    node.read_can_feedback()
    assert node.event_counts['can_warning_frames'] == 0
    assert node.event_counts['can_bus_off_frames'] == 0
    assert node.feedback_by_id[0x2968].angle_deg == 23.0
    assert node.feedback_by_id[0x2969].angle_deg == -173.0


def test_startup_log_warns_when_step_cap_is_below_vmax():
    logger = SimpleNamespace(info=Mock(), warning=Mock())
    config = {
        'control_rate_hz': 5.0,
        'max_velocity_deg_s': 5.0,
        'max_acceleration_deg_s2': 25.0,
        'max_step_deg': 0.5,
        'filter_alpha': 1.0,
    }
    timer = SimpleNamespace(timer_period_ns=200_000_000)
    node = SimpleNamespace(config=config, control_timer=timer,
                           get_logger=lambda: logger)
    teleop.MiraboTeleop.log_limiter_config(node)
    startup = logger.info.call_args_list[0].args[0]
    assert startup.startswith('ACTIVE TELEOP PROFILE:')
    assert 'actual_timer_period_sec=0.2' in startup
    assert 'limiter_vmax_0x68=5' in startup
    assert 'limiter_vmax_0x69=5' in startup
    assert 'limiter_amax_0x68=25' in startup
    assert 'limiter_amax_0x69=25' in startup
    assert 'max_step_deg=0.5' in startup
    assert 'effective_step_velocity_cap=2.5 deg/s' in startup
    assert 'required_step_for_vmax=1' in logger.info.call_args_list[1].args[0]
    assert any('Mirabo limiter 0x68:' in call.args[0]
               for call in logger.info.call_args_list)
    assert any('Mirabo limiter 0x69:' in call.args[0]
               for call in logger.info.call_args_list)
    assert 'max_step_deg is limiting effective velocity' in (
        logger.warning.call_args.args[0]
    )
    config['control_rate_hz'] = 20.0
    timer.timer_period_ns = 50_000_000
    logger.warning.reset_mock()
    teleop.MiraboTeleop.log_limiter_config(node)
    logger.warning.assert_not_called()


def test_diagnostics_show_step_limit_for_five_hz_five_dps(rig):
    node, bus, clock = rig
    node.config['max_velocity_deg_s'] = 5.0
    node.config['max_acceleration_deg_s2'] = 25.0
    diagnostics = []
    node.diagnostics_pub = SimpleNamespace(
        publish=lambda msg: diagnostics.append(json.loads(msg.data))
    )
    sample(node, bus, 0.0, 0.0, 0.0, 0.0)
    assert node.set_parameters([Parameter('armed', value=True)])[0].successful
    node.control_step()
    clock.now += 0.2
    sample(node, bus, 30.0, 0.0, 0.0, 0.0)
    node.control_step()
    assert diagnostics[-1]['configured_vmax'] == 5.0
    assert diagnostics[-1]['effective_step_velocity_cap'] == 2.5
    assert diagnostics[-1]['configured_filter_alpha'] == 1.0
    assert diagnostics[-1]['effective_step_velocity_cap_actual'] == pytest.approx(
        0.5 * diagnostics[-1]['actual_control_rate_hz']
    )
    assert diagnostics[-1]['required_step_for_vmax'] == 1.0
    assert diagnostics[-1]['limiter_flags_valid']
    assert diagnostics[-1]['step_limited']
    assert diagnostics[-1]['joints']['0x68']['step_limited']
    assert diagnostics[-1]['joints']['0x68']['active_limit'] == 'STEP'
    assert not diagnostics[-1]['joints']['0x68']['velocity_limited']
    trace = diagnostics[-1]['joints']['0x68']['limiter_trace_can_units']
    assert trace['velocity_after_velocity_limit'] == pytest.approx(-5.0)
    assert trace['position_increment_before_step_limit'] == pytest.approx(-1.0)
    assert trace['position_increment_after_step_limit'] == pytest.approx(-0.5)
    assert trace['final_v_cmd'] == pytest.approx(-2.5)
    last_68 = bus.sent[-2]
    assert last_68.arbitration_id == 0x668
    assert struct.unpack('>ihh', last_68.data)[0] / 10000.0 == pytest.approx(
        -0.5, abs=0.001,
    )


def test_profile_parameters_set_timer_at_startup(monkeypatch, tmp_path):
    bus = FakeBus()
    monkeypatch.setattr(teleop.can, 'Bus', lambda **kwargs: bus)
    path = (Path(__file__).resolve().parents[1] / 'config' / 'experiments' /
            'phase2_20hz_5dps.yaml')
    config = yaml.safe_load(path.read_text())[
        'leader_mirabo_teleop']['ros__parameters']
    overrides = [Parameter(name, value=value)
                 for name, value in config.items()]
    overrides.extend((
        Parameter('armed', value=True),
        Parameter('require_origin', value=False),
        Parameter('origin_file', value=str(tmp_path / 'no-origin.json')),
    ))
    context = Context()
    rclpy.init(args=[], context=context, domain_id=232)
    node = None
    try:
        node = teleop.MiraboTeleop(
            context=context, use_global_arguments=False,
            parameter_overrides=overrides,
        )
        assert node.config['control_rate_hz'] == 20.0
        assert node.config['max_velocity_deg_s'] == 5.0
        assert node.config['max_acceleration_deg_s2'] == 25.0
        assert node.config['max_step_deg'] == 0.5
        assert node.control_timer.timer_period_ns == 50_000_000
        assert node.get_parameter('armed').value is False
        assert bus.sent == []
        clock = SimpleNamespace(now=100.0)
        monkeypatch.setattr(node, 'now_sec', lambda: clock.now)
        diagnostics = []
        node.diagnostics_pub = SimpleNamespace(
            publish=lambda msg: diagnostics.append(json.loads(msg.data))
        )
        sample(node, bus, 0.0, 0.0, 0.0, 0.0)
        assert node.set_parameters([Parameter('armed', value=True)])[0].successful
        node.control_step()
        assert [struct.unpack('>ihh', frame.data)[0] / 10000.0
                for frame in bus.sent] == pytest.approx([0.0, 0.0])
        velocities = {item.motor_id: [] for item in mapping.MIRABO_JOINTS}
        for _ in range(5):
            clock.now += 0.05
            sample(node, bus, 40.0, 40.0, 0.0, 0.0)
            node.control_step()
            for item in mapping.MIRABO_JOINTS:
                velocities[item.motor_id].append(
                    abs(node.last_telemetry[item.motor_id]['v_cmd_deg_s'])
                )
        for values in velocities.values():
            assert values == pytest.approx([1.25, 2.5, 3.75, 5.0, 5.0])
        assert diagnostics[-1]['timer_period_sec'] == pytest.approx(0.05)
        assert diagnostics[-1]['effective_step_velocity_cap'] == 10.0
        assert diagnostics[-1]['configured_filter_alpha'] == 1.0
        assert diagnostics[-1]['effective_step_velocity_cap_actual'] == pytest.approx(
            0.5 * diagnostics[-1]['actual_control_rate_hz']
        )
        assert diagnostics[-1]['required_step_for_vmax'] == 0.25
        assert not diagnostics[-1]['step_limited']
        assert diagnostics[-1]['joints']['0x68']['active_limit'] == 'VELOCITY'
        assert diagnostics[-1]['joints']['0x69']['active_limit'] == 'VELOCITY'
        for motor in ('0x68', '0x69'):
            trace = diagnostics[-1]['joints'][motor]['limiter_trace_can_units']
            assert abs(trace['velocity_after_velocity_limit']) == pytest.approx(5.0)
            assert abs(trace['velocity_after_acceleration_limit']) == pytest.approx(5.0)
            assert abs(trace['position_increment_before_step_limit']) == pytest.approx(0.25)
            assert abs(trace['position_increment_after_step_limit']) == pytest.approx(0.25)
            assert abs(trace['final_v_cmd']) == pytest.approx(5.0)
        assert all(abs(node.last_command[item.motor_id]) <= 5.0 * 0.25
                   for item in mapping.MIRABO_JOINTS)
        assert all(struct.unpack('>ihh', frame.data)[1:] == (100, 100)
                   for frame in bus.sent)
    finally:
        if node is not None:
            node.destroy_node()
        context.shutdown()


@pytest.mark.parametrize('speed,accel,raw_speed,raw_accel', [
    (900, 1000, 90, 100),
    (1000, 1100, 100, 110),
])
def test_command_profile_overrides_reach_can_and_diagnostics(
        monkeypatch, tmp_path, speed, accel, raw_speed, raw_accel):
    bus = FakeBus()
    monkeypatch.setattr(teleop.can, 'Bus', lambda **kwargs: bus)
    context = Context()
    rclpy.init(args=[], context=context, domain_id=229)
    node = None
    try:
        node = teleop.MiraboTeleop(
            context=context, use_global_arguments=False,
            parameter_overrides=[
                Parameter('mirabo_command_speed', value=speed),
                Parameter('mirabo_command_acceleration', value=accel),
                Parameter('require_origin', value=False),
                Parameter('origin_file', value=str(tmp_path / 'no-origin.json')),
            ],
        )
        assert node.command_speed_raw == raw_speed
        assert node.command_acceleration_raw == raw_accel
        clock = SimpleNamespace(now=100.0)
        monkeypatch.setattr(node, 'now_sec', lambda: clock.now)
        diagnostics = []
        node.diagnostics_pub = SimpleNamespace(
            publish=lambda msg: diagnostics.append(json.loads(msg.data))
        )
        sample(node, bus, 0.0, 0.0, 10.0, 20.0)
        assert node.set_parameters([Parameter('armed', value=True)])[0].successful
        node.control_step()
        assert [frame.arbitration_id for frame in bus.sent] == [0x668, 0x669]
        assert all(struct.unpack('>ihh', frame.data)[1:] ==
                   (raw_speed, raw_accel) for frame in bus.sent)
        assert diagnostics[-1]['command_speed_configured'] == speed
        assert diagnostics[-1]['command_acceleration_configured'] == accel
        assert diagnostics[-1]['command_speed_raw'] == raw_speed
        assert diagnostics[-1]['command_acceleration_raw'] == raw_accel
    finally:
        if node is not None:
            node.destroy_node()
        context.shutdown()


def test_invalid_command_profile_fails_before_opening_can(monkeypatch):
    bus_factory = Mock()
    monkeypatch.setattr(teleop.can, 'Bus', bus_factory)
    context = Context()
    rclpy.init(args=[], context=context, domain_id=228)
    try:
        with pytest.raises(ValueError, match='signed int16 source range'):
            teleop.MiraboTeleop(
                context=context, use_global_arguments=False,
                parameter_overrides=[
                    Parameter('mirabo_command_speed', value=327680),
                ],
            )
        bus_factory.assert_not_called()
    finally:
        context.shutdown()
