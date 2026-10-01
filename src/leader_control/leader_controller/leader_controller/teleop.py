#!/usr/bin/env python3
"""ROS 2 teleop node from calibrated leader joints to Mirabo CAN motors."""

from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time
from collections import deque
import json

import can
from ament_index_python.packages import get_package_share_directory
from leader_controller import mapping
from leader_controller.command_limiter import (
    active_constraints, limit_position, trace_limit_stages,
)
from leader_controller.filters import EmaFilter
from leader_controller.mirabo_can import (
    COMMAND_PAYLOAD_FORMAT, MiraboCanDriver, pack_position_command,
    unpack_feedback, validate_command_field,
)
from leader_controller.safety import FaultRecord, TeleopState
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from leader_state import origin as saved_origin
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml


def joint_delta_to_can_delta(delta_deg):
    """Convert output joint delta to CAN angle delta."""
    if mapping.CAN_ANGLE_IS_MOTOR_SHAFT:
        return delta_deg * mapping.GEAR_RATIO
    return delta_deg


@dataclass
class MotorFeedback:
    """Most recent feedback from one Mirabo motor."""

    angle_deg: float = math.nan
    fault: int = 0
    stamp_sec: float = -math.inf


class MiraboTeleop(Node):
    """Bridge /leader/joint_states to Mirabo position commands on can0."""

    def __init__(self, **kwargs):
        super().__init__('leader_mirabo_teleop', **kwargs)

        self.declare_parameter(
            'armed', False,
            ParameterDescriptor(
                description='Enable at runtime only after inputs are ready; '
                'startup overrides are ignored. False stops commands, '
                'but does not disable torque.'
            ),
            ignore_override=True,
        )
        self.declare_parameter(
            'origin_file', str(saved_origin.default_origin_file())
        )
        self.origin_file = self.get_parameter('origin_file').value
        self.origin = saved_origin.load_origin(self.origin_file)
        read_only = ParameterDescriptor(read_only=True)
        config_path = (Path(get_package_share_directory('leader_controller')) /
                       'config' / 'teleop.yaml')
        with config_path.open(encoding='utf-8') as config_file:
            defaults = yaml.safe_load(config_file)[
                'leader_mirabo_teleop']['ros__parameters']
        for name, value in defaults.items():
            self.declare_parameter(name, value, read_only)
        self.config = {
            name: self.get_parameter(name).value for name in defaults
        }
        for name in defaults:
            if name == 'require_origin':
                if not isinstance(self.config[name], bool):
                    raise ValueError('require_origin must be boolean')
                continue
            if name == 'can_interface':
                if not isinstance(self.config[name], str) or not self.config[name]:
                    raise ValueError('can_interface must be nonempty')
                continue
            value = self.config[name]
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value <= 0):
                raise ValueError(f'{name} must be positive and finite')
        if self.config['filter_alpha'] > 1.0:
            raise ValueError('filter_alpha must be at most 1')
        self.command_speed_raw = validate_command_field(
            self.config['mirabo_command_speed'], 'mirabo_command_speed',
        )
        self.command_acceleration_raw = validate_command_field(
            self.config['mirabo_command_acceleration'],
            'mirabo_command_acceleration',
        )

        try:
            self.driver = MiraboCanDriver(
                feedback_ids=(item.feedback_id for item in mapping.MIRABO_JOINTS),
                interface=self.config['can_interface'],
                bitrate=self.config['can_bitrate'],
            )
        except (can.CanError, OSError):
            super().destroy_node()
            raise

        self.feedback_by_id = {
            item.feedback_id: MotorFeedback()
            for item in mapping.MIRABO_JOINTS
        }
        self.leader_positions = {}
        self.leader_stamp_sec = -math.inf
        self.stage_stamps_ns = {}
        self.event_stamps_ns = {
            name: deque()
            for name in ('leader_rx', 'control', 'can_tx', 'feedback_68',
                         'feedback_69')
        }
        self.event_counts = {
            name: 0 for name in (
                'can_warning_frames', 'can_bus_off_frames',
                'can_malformed_frames', 'motor_fault_events',
                'can_receive_errors', 'command_errors',
            )
        }
        self.last_telemetry = {}

        self.baseline_leader = {}
        self.baseline_motor = {}
        self.last_command = {}
        self.last_command_time = {}
        self.last_command_velocity = {}
        self.target_filters = {
            item.motor_id: EmaFilter(self.config['filter_alpha'])
            for item in mapping.MIRABO_JOINTS
        }
        self.internal_armed = False
        self.last_fault_reason = ''
        self.fault = None
        self.last_sent_stamps = None
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.add_post_set_parameters_callback(self.parameters_changed)

        self.leader_sub = self.create_subscription(
            JointState,
            '/leader/joint_states',
            self.leader_callback,
            1,
        )
        self.status_pub = self.create_publisher(
            String,
            '/leader_controller/mirabo_status',
            10,
        )
        self.feedback_pub = self.create_publisher(
            JointState,
            '/leader_controller/mirabo_feedback',
            10,
        )
        self.reference_pub = self.create_publisher(
            JointState, '/leader_controller/reference', 10,
        )
        self.filtered_reference_pub = self.create_publisher(
            JointState, '/leader_controller/filtered_reference', 10,
        )
        self.command_pub = self.create_publisher(
            JointState, '/leader_controller/command', 10,
        )
        self.actual_pub = self.create_publisher(
            JointState, '/mirabo/joint_states', 10,
        )
        self.diagnostics_pub = self.create_publisher(
            String, '/leader_controller/diagnostics', 10,
        )
        self.clear_fault_srv = self.create_service(
            Trigger, '/leader_controller/clear_fault', self.clear_fault,
        )
        self.estop_srv = self.create_service(
            Trigger, '/leader_controller/estop', self.estop,
        )
        self.clear_estop_srv = self.create_service(
            Trigger, '/leader_controller/clear_estop', self.clear_estop,
        )

        period = 1.0 / self.config['control_rate_hz']
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.can_timer = self.create_timer(
            1.0 / self.config['can_read_rate_hz'],
            self.read_can_feedback,
            clock=self.steady_clock,
        )
        self.control_timer = self.create_timer(
            period,
            self.control_step,
            clock=self.steady_clock,
        )

        self.get_logger().info(
            "Mirabo teleop started disarmed. Set parameter 'armed' true "
            'after all inputs are ready.'
        )
        self.log_limiter_config()
        self.get_logger().info(
            'MIRABO COMMAND PROFILE: '
            f"command_speed_parameter={self.config['mirabo_command_speed']:g}; "
            f"command_acceleration_parameter={self.config['mirabo_command_acceleration']:g}; "
            f'encoded_speed_raw={self.command_speed_raw}; '
            f'encoded_acceleration_raw={self.command_acceleration_raw}; '
            f'CAN payload format={COMMAND_PAYLOAD_FORMAT}'
        )
        self.get_logger().info(
            f'CAN angle scale={joint_delta_to_can_delta(1.0):g}; '
            f'gear ratio={mapping.GEAR_RATIO:g}. '
            'Stopping commands does not disable torque.'
        )
        if self.origin is not None:
            self.get_logger().info(
                f'Mirabo software origin: {self.origin_file}'
            )
        elif self.config['require_origin']:
            self.get_logger().warning(
                f'Mirabo origin missing: {self.origin_file}; arming disabled'
            )
        self.get_logger().warning(
            'No absolute angle limits are configured; confirm mechanical travel.'
        )

    def log_limiter_config(self):
        """Show the loaded limits and the nominal per-cycle step ceiling."""
        step_velocity_cap = (
            self.config['max_step_deg'] * self.config['control_rate_hz']
        )
        required_step = (
            self.config['max_velocity_deg_s'] / self.config['control_rate_hz']
        )
        timer_period_sec = self.control_timer.timer_period_ns * 1e-9
        self.get_logger().info(
            'ACTIVE TELEOP PROFILE: '
            f"control_rate_hz={self.config['control_rate_hz']:g}; "
            f'actual_timer_period_sec={timer_period_sec:g}; '
            f"limiter_vmax_0x68={self.config['max_velocity_deg_s']:g}; "
            f"limiter_vmax_0x69={self.config['max_velocity_deg_s']:g}; "
            f"limiter_amax_0x68={self.config['max_acceleration_deg_s2']:g}; "
            f"limiter_amax_0x69={self.config['max_acceleration_deg_s2']:g}; "
            f"max_step_deg={self.config['max_step_deg']:g}; "
            f'effective_step_velocity_cap={step_velocity_cap:g} deg/s; '
            f"filter_alpha={self.config['filter_alpha']:g}"
        )
        self.get_logger().info(
            f"configured_control_rate_hz={self.config['control_rate_hz']:g}; "
            f'timer_period_sec={timer_period_sec:g}; '
            f"max_velocity_deg_s={self.config['max_velocity_deg_s']:g}; "
            f"max_acceleration_deg_s2="
            f"{self.config['max_acceleration_deg_s2']:g}; "
            f"max_step_deg={self.config['max_step_deg']:g}; "
            f'effective_step_velocity_cap={step_velocity_cap:g} deg/s '
            '(nominal); '
            f'required_step_for_vmax={required_step:g} deg; '
            f"filter_alpha={self.config['filter_alpha']:g}"
        )
        for item in mapping.MIRABO_JOINTS:
            self.get_logger().info(
                f'Mirabo limiter 0x{item.motor_id:02X}: '
                f"vmax={self.config['max_velocity_deg_s']:g} deg/s, "
                f"amax={self.config['max_acceleration_deg_s2']:g} deg/s^2, "
                f"max_step={self.config['max_step_deg']:g} deg, "
                f"control_rate={self.config['control_rate_hz']:g} Hz, "
                f'step_velocity_cap={step_velocity_cap:g} deg/s; '
                'limits are passed from loaded config on each command'
            )
        if self.config['max_step_deg'] < required_step:
            self.get_logger().warning(
                'max_step_deg is limiting effective velocity below configured '
                f"max_velocity_deg_s={self.config['max_velocity_deg_s']:g}: "
                f"max_step_deg={self.config['max_step_deg']:g} at "
                f"control_rate_hz={self.config['control_rate_hz']:g} "
                f'caps nominal speed at {step_velocity_cap:g} deg/s; '
                f'required_step_for_vmax={required_step:g} deg'
            )

    def validate_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name != 'armed':
                continue
            if parameter.type_ != Parameter.Type.BOOL:
                return SetParametersResult(
                    successful=False, reason='armed must be a bool'
                )
            if parameter.value:
                if self.fault is not None:
                    return SetParametersResult(
                        successful=False, reason='clear fault before arming'
                    )
                ready, reason = self.ready()
                if not ready:
                    return SetParametersResult(
                        successful=False, reason=reason
                    )
        return SetParametersResult(successful=True)

    def parameters_changed(self, parameters):
        for parameter in parameters:
            if parameter.name == 'armed':
                if not parameter.value:
                    self.internal_armed = False
                    self.last_sent_stamps = None
                    self.get_logger().info(
                        'Disarmed: commands stopped; torque is unchanged.'
                    )

    def leader_callback(self, msg):
        self.stage_stamps_ns['t_leader_rx'] = time.monotonic_ns()
        self.record_event('leader_rx', self.stage_stamps_ns['t_leader_rx'])
        names = list(msg.name)
        # An invalid message must not leave the previous sample armable.
        self.leader_positions = {}
        self.leader_stamp_sec = -math.inf
        if len(msg.position) != len(names):
            self.disarm('leader JointState has mismatched name/position',
                        'INVALID_LEADER_DATA')
            return
        if len(set(names)) != len(names):
            self.disarm('leader JointState has duplicate names',
                        'INVALID_LEADER_DATA')
            return

        positions = dict(zip(names, msg.position))
        for item in mapping.MIRABO_JOINTS:
            if item.leader_joint not in positions:
                self.disarm(
                    f'leader JointState missing {item.leader_joint}',
                    'INVALID_LEADER_DATA',
                )
                return
            if not math.isfinite(positions[item.leader_joint]):
                self.disarm(
                    f'leader JointState has invalid {item.leader_joint}',
                    'INVALID_LEADER_DATA',
                )
                return

        self.leader_positions = positions
        self.leader_stamp_sec = self.now_sec()

    def read_can_feedback(self):
        try:
            for event in self.driver.read_events():
                if event.kind == 'bus_off':
                    self.event_counts['can_bus_off_frames'] += 1
                    self.can_fault(
                        f'CAN BUS-OFF error frame 0x{event.arbitration_id:X}, '
                        f'data={event.detail}', 'CAN_BUS_OFF',
                    )
                    return
                if event.kind == 'error':
                    self.event_counts['can_warning_frames'] += 1
                    self.get_logger().warning(
                        f'Non-fatal CAN error frame '
                        f'0x{event.arbitration_id:X}, data={event.detail}',
                        throttle_duration_sec=2.0,
                    )
                    continue
                if event.kind == 'malformed':
                    self.event_counts['can_malformed_frames'] += 1
                    self.get_logger().warning(
                        f'Ignoring malformed CAN feedback '
                        f'0x{event.arbitration_id:X}: {event.detail}',
                        throttle_duration_sec=2.0,
                    )
                    continue
                feedback = self.feedback_by_id[event.arbitration_id]
                received_ns = time.monotonic_ns()
                feedback.angle_deg = event.angle_deg
                feedback.fault = event.fault
                feedback.stamp_sec = self.now_sec()
                key = f'feedback_{event.arbitration_id & 0xFF:02x}'
                self.stage_stamps_ns[
                    f't_feedback_rx_{event.arbitration_id & 0xFF:02x}'
                ] = received_ns
                self.record_event(key, received_ns)
                if event.fault:
                    self.event_counts['motor_fault_events'] += 1
                    item = self.mapping_for_feedback(event.arbitration_id)
                    self.disarm(
                        f'Mirabo 0x{item.motor_id:02X} fault=0x{event.fault:02X}',
                        f'MOTOR_{item.motor_id:02X}_FAULT',
                    )
                else:
                    self.publish_actual_sample(
                        self.mapping_for_feedback(event.arbitration_id),
                        event.angle_deg,
                    )
        except (can.CanError, OSError) as exc:
            self.event_counts['can_receive_errors'] += 1
            self.can_fault(f'CAN receive error: {exc}', 'CAN_SOCKET_ERROR')

    def control_step(self):
        control_ns = time.monotonic_ns()
        self.stage_stamps_ns['t_control_loop'] = control_ns
        self.record_event('control', control_ns)
        # Consume queued faults before considering either motor command.
        self.read_can_feedback()
        requested_armed = bool(self.get_parameter('armed').value)
        ready, reason = self.ready()

        if not requested_armed:
            self.publish_status(self.current_state(ready, reason),
                                self.last_fault_reason or reason)
            self.publish_feedback()
            self.publish_telemetry()
            return

        if not ready:
            code = ('LEADER_TIMEOUT' if 'leader/joint_states' in reason
                    else 'CAN_FEEDBACK_TIMEOUT' if 'timeout waiting for Mirabo'
                    in reason else 'INPUT_FAULT')
            self.disarm(reason, code)
            self.publish_status('FAULT')
            self.publish_feedback()
            self.publish_telemetry()
            return

        if not self.internal_armed:
            self.arm_from_current_positions()

        stamps = (self.leader_stamp_sec,) + tuple(
            self.feedback_by_id[item.feedback_id].stamp_sec
            for item in mapping.MIRABO_JOINTS
        )
        if self.last_sent_stamps is not None and any(
            current <= previous
            for current, previous in zip(stamps, self.last_sent_stamps)
        ):
            self.publish_status('ARMED', 'waiting for new input samples')
            self.publish_feedback()
            self.publish_telemetry()
            return

        try:
            commands = [
                (item, self.command_for_mapping(item))
                for item in mapping.MIRABO_JOINTS
            ]
            self.stage_stamps_ns['t_reference_ready'] = time.monotonic_ns()
            sent_at = self.now_sec()
            for item, command in commands:
                telemetry = self.last_telemetry[item.motor_id]
                telemetry['q_cmd_deg'] = command.position_deg
                telemetry['v_cmd_deg_s'] = command.velocity_deg_s
                self.send_position_command(item, command.position_deg)
                tx_ns = time.monotonic_ns()
                self.stage_stamps_ns['t_can_tx'] = tx_ns
                self.record_event('can_tx', tx_ns)
                self.last_command[item.motor_id] = command.position_deg
                self.last_command_velocity[item.motor_id] = command.velocity_deg_s
                self.last_command_time[item.motor_id] = sent_at
            self.last_sent_stamps = stamps
        except (can.CanError, OSError, ValueError) as exc:
            self.event_counts['command_errors'] += 1
            self.can_fault(f'CAN command error: {exc}', 'CAN_COMMAND_ERROR')

        self.publish_status('ARMED' if self.internal_armed else 'FAULT')
        self.publish_feedback()
        self.publish_telemetry()

    def ready(self):
        now = self.now_sec()
        if self.config['require_origin'] and self.origin is None:
            return False, f'origin file missing: {self.origin_file}'
        if self.driver.bus.state == can.BusState.ERROR:
            return False, f'CAN bus state is {self.driver.bus.state.name}'
        if now - self.leader_stamp_sec > self.config['leader_timeout_sec']:
            return False, 'timeout waiting for /leader/joint_states'

        for item in mapping.MIRABO_JOINTS:
            if item.leader_joint not in self.leader_positions:
                return False, f'missing leader {item.leader_joint}'
            feedback = self.feedback_by_id[item.feedback_id]
            if now - feedback.stamp_sec > self.config['feedback_timeout_sec']:
                return False, (
                    f'timeout waiting for Mirabo 0x{item.motor_id:02X}'
                )
            if feedback.fault:
                return False, (
                    f'Mirabo 0x{item.motor_id:02X} fault='
                    f'0x{feedback.fault:02X}'
                )
            if not math.isfinite(feedback.angle_deg):
                return False, f'missing Mirabo 0x{item.motor_id:02X} angle'
        return True, ''

    def arm_from_current_positions(self):
        self.baseline_leader = {}
        self.baseline_motor = {}
        self.last_command = {}
        self.last_command_time = {}
        self.last_command_velocity = {}
        armed_at = self.now_sec()

        for item in mapping.MIRABO_JOINTS:
            leader_rad = self.leader_positions[item.leader_joint]
            feedback = self.feedback_by_id[item.feedback_id]
            self.baseline_leader[item.leader_joint] = math.degrees(
                leader_rad
            )
            self.baseline_motor[item.motor_id] = feedback.angle_deg
            self.last_command[item.motor_id] = feedback.angle_deg
            self.last_command_time[item.motor_id] = armed_at
            self.last_command_velocity[item.motor_id] = 0.0
            self.target_filters[item.motor_id].reset(feedback.angle_deg)

        self.internal_armed = True
        self.last_sent_stamps = None
        self.get_logger().info(
            'Armed Mirabo teleop from current leader and motor positions.'
        )

    def command_for_mapping(self, item):
        self.last_telemetry[item.motor_id] = {}
        leader_ns = self.stage_stamps_ns.get('t_leader_rx', 0)
        target = mapping.target_can_degrees(
            item, self.leader_positions[item.leader_joint],
            self.baseline_leader[item.leader_joint],
            self.baseline_motor[item.motor_id],
        )
        self.last_telemetry[item.motor_id]['q_des_deg'] = target
        self.stage_stamps_ns['t_reference_ready'] = time.monotonic_ns()
        target = self.target_filters[item.motor_id].update(target)
        self.last_telemetry[item.motor_id]['q_filtered_deg'] = target
        self.stage_stamps_ns['t_filtered_ready'] = time.monotonic_ns()
        previous = self.last_command[item.motor_id]
        previous_velocity = self.last_command_velocity[item.motor_id]
        elapsed = self.now_sec() - self.last_command_time[item.motor_id]
        command = limit_position(
            target, previous, previous_velocity, elapsed,
            self.config['max_step_deg'],
            self.config['max_velocity_deg_s'],
            self.config['max_acceleration_deg_s2'],
        )
        self.last_telemetry[item.motor_id]['limiter_trace'] = (
            trace_limit_stages(
                target, previous, previous_velocity, elapsed,
                self.config['max_step_deg'],
                self.config['max_velocity_deg_s'],
                self.config['max_acceleration_deg_s2'], command,
            )
        )
        limit_flags = active_constraints(
            target, previous, previous_velocity, elapsed,
            self.config['max_step_deg'],
            self.config['max_velocity_deg_s'],
            self.config['max_acceleration_deg_s2'], command,
        )
        self.last_telemetry[item.motor_id].update(limit_flags)
        self.last_telemetry[item.motor_id]['active_limit'] = '+'.join(
            label for key, label in (
                ('velocity_limited', 'VELOCITY'),
                ('acceleration_limited', 'ACCELERATION'),
                ('step_limited', 'STEP'),
            ) if limit_flags[key]
        ) or 'NONE'
        self.last_telemetry[item.motor_id]['limit_control_ns'] = (
            self.stage_stamps_ns['t_control_loop']
        )
        self.last_telemetry[item.motor_id]['q_cmd_deg'] = command.position_deg
        self.last_telemetry[item.motor_id]['v_cmd_deg_s'] = command.velocity_deg_s
        self.stage_stamps_ns['t_command_ready'] = time.monotonic_ns()
        self.last_telemetry[item.motor_id]['t_leader_rx'] = leader_ns
        self.last_telemetry[item.motor_id]['t_reference_ready'] = (
            self.stage_stamps_ns['t_reference_ready']
        )
        self.last_telemetry[item.motor_id]['t_filtered_ready'] = (
            self.stage_stamps_ns['t_filtered_ready']
        )
        self.last_telemetry[item.motor_id]['t_command_ready'] = (
            self.stage_stamps_ns['t_command_ready']
        )
        return command

    def send_position_command(self, item, angle_deg):
        if not self.internal_armed or not self.get_parameter('armed').value:
            raise ValueError('cannot send a position command while disarmed')
        if not math.isfinite(angle_deg):
            raise ValueError('CAN position command must be finite')
        self.driver.send_position(
            item.command_id, angle_deg,
            self.config['mirabo_command_speed'],
            self.config['mirabo_command_acceleration'],
        )

    def publish_status(self, state, reason=''):
        if isinstance(state, TeleopState):
            state = state.value
        parts = [
            f'state={state}',
            f"armed_param={bool(self.get_parameter('armed').value)}",
            f'internal_armed={self.internal_armed}',
            f'leader_age_sec={self.now_sec() - self.leader_stamp_sec:.3f}',
            f'can_state={self.driver.bus.state.name}',
        ]
        if self.fault is not None:
            parts.extend((f'fault_code={self.fault.code}',
                          f'fault_time_sec={self.fault.stamp_sec:.3f}'))
        if reason or self.last_fault_reason:
            parts.append(f'reason={reason or self.last_fault_reason}')

        for item in mapping.MIRABO_JOINTS:
            feedback = self.feedback_by_id[item.feedback_id]
            parts.append(
                f'0x{item.motor_id:02X}='
                f'{feedback.angle_deg:.1f}CANdeg/'
                f'fault=0x{feedback.fault:02X}/'
                f'age_sec={self.now_sec() - feedback.stamp_sec:.3f}'
            )
            if item.motor_id in self.last_command:
                parts.append(
                    f'0x{item.motor_id:02X}_last_command='
                    f'{self.last_command[item.motor_id]:.1f}CANdeg'
                )

        msg = String()
        msg.data = '; '.join(parts)
        self.status_pub.publish(msg)

    def publish_feedback(self):
        now = self.now_sec()
        if any(
            now - feedback.stamp_sec > self.config['feedback_timeout_sec']
            or not math.isfinite(feedback.angle_deg)
            or feedback.fault
            for feedback in self.feedback_by_id.values()
        ):
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        offsets = (
            self.origin['mirabo_can_degrees'] if self.origin is not None else {}
        )
        msg.name = [
            f'mirabo_0x{item.motor_id:02X}'
            for item in mapping.MIRABO_JOINTS
        ]
        msg.position = [
            math.radians(
                (self.feedback_by_id[item.feedback_id].angle_deg
                 - offsets.get(f'0x{item.motor_id:02x}', 0.0))
                / joint_delta_to_can_delta(1.0)
            )
            for item in mapping.MIRABO_JOINTS
        ]
        self.feedback_pub.publish(msg)

    def publish_actual_sample(self, item, angle_deg):
        offsets = self.origin['mirabo_can_degrees'] if self.origin else {}
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'mirabo_{item.motor_id:02x}']
        msg.position = [math.radians(
            (angle_deg - offsets.get(f'0x{item.motor_id:02x}', 0.0)) /
            joint_delta_to_can_delta(1.0)
        )]
        self.actual_pub.publish(msg)

    def publish_telemetry(self):
        now_ns = time.monotonic_ns()
        now_sec = self.now_sec()
        stamp = self.get_clock().now().to_msg()
        names = [f'mirabo_{item.motor_id:02x}' for item in mapping.MIRABO_JOINTS]
        offsets = self.origin['mirabo_can_degrees'] if self.origin else {}

        def output_degrees(item, angle_deg):
            return (angle_deg - offsets.get(f'0x{item.motor_id:02x}', 0.0)) / \
                joint_delta_to_can_delta(1.0)

        actual_valid = all(
            now_sec - self.feedback_by_id[item.feedback_id].stamp_sec
            <= self.config['feedback_timeout_sec']
            and not self.feedback_by_id[item.feedback_id].fault
            and math.isfinite(self.feedback_by_id[item.feedback_id].angle_deg)
            for item in mapping.MIRABO_JOINTS
        )
        telemetry_valid = all(
            item.motor_id in self.last_telemetry
            and self.leader_positions.get(item.leader_joint) is not None
            and all(field in self.last_telemetry[item.motor_id] for field in
                    ('q_des_deg', 'q_filtered_deg', 'q_cmd_deg', 'v_cmd_deg_s'))
            for item in mapping.MIRABO_JOINTS
        )
        if telemetry_valid:
            for topic, field in (
                (self.reference_pub, 'q_des_deg'),
                (self.filtered_reference_pub, 'q_filtered_deg'),
                (self.command_pub, 'q_cmd_deg'),
            ):
                msg = JointState()
                msg.header.stamp = stamp
                msg.name = names
                msg.position = [
                    math.radians(output_degrees(
                        item, self.last_telemetry[item.motor_id][field]
                    )) for item in mapping.MIRABO_JOINTS
                ]
                if topic is self.command_pub:
                    msg.velocity = [
                        math.radians(self.last_telemetry[item.motor_id]
                                     ['v_cmd_deg_s'] /
                                     joint_delta_to_can_delta(1.0))
                        for item in mapping.MIRABO_JOINTS
                    ]
                topic.publish(msg)

        metrics = {}
        current_limit_flags = []
        for item in mapping.MIRABO_JOINTS:
            entry = self.last_telemetry.get(item.motor_id)
            if not entry or item.leader_joint not in self.leader_positions:
                continue
            limit_flags = {
                name: entry.get(name) if entry.get('limit_control_ns') ==
                self.stage_stamps_ns.get('t_control_loop') else None
                for name in ('velocity_limited', 'acceleration_limited',
                             'step_limited')
            }
            if all(value is not None for value in limit_flags.values()):
                current_limit_flags.append(limit_flags)
            q_actual = (math.radians(output_degrees(
                item, self.feedback_by_id[item.feedback_id].angle_deg
            )) if actual_valid else None)
            q_des = math.radians(output_degrees(item, entry['q_des_deg']))
            q_cmd = math.radians(output_degrees(item, entry['q_cmd_deg']))
            metrics[f'0x{item.motor_id:02x}'] = {
                'q_leader_rad': self.leader_positions[item.leader_joint],
                'q_des_rad': q_des,
                'q_filtered_rad': math.radians(output_degrees(
                    item, entry['q_filtered_deg'])),
                'q_cmd_rad': q_cmd,
                'v_cmd_rad_s': math.radians(
                    entry['v_cmd_deg_s'] / joint_delta_to_can_delta(1.0)),
                'q_actual_rad': q_actual,
                'reference_error_rad': (q_des - q_actual
                                        if q_actual is not None else None),
                'command_error_rad': (q_cmd - q_actual
                                      if q_actual is not None else None),
                'limiter_gap_rad': q_des - q_cmd,
                **limit_flags,
                'active_limit': (entry['active_limit']
                                 if limit_flags['velocity_limited'] is not None
                                 else None),
                'limiter_trace_can_units': (
                    entry.get('limiter_trace')
                    if limit_flags['velocity_limited'] is not None else None
                ),
                't_feedback_rx_ns': self.stage_stamps_ns.get(
                    f't_feedback_rx_{item.motor_id:02x}', 0),
                't_can_tx_ns': self.stage_stamps_ns.get('t_can_tx', 0),
            }
        actual_control_rate_hz = self.event_rate('control')
        data = {
            'monotonic_ns': now_ns,
            'configured_control_rate_hz': self.config['control_rate_hz'],
            'actual_control_rate_hz': actual_control_rate_hz,
            'timer_period_sec': self.control_timer.timer_period_ns * 1e-9,
            'configured_filter_alpha': self.config['filter_alpha'],
            'command_speed_configured': self.config['mirabo_command_speed'],
            'command_acceleration_configured': self.config[
                'mirabo_command_acceleration'],
            'command_speed_raw': self.command_speed_raw,
            'command_acceleration_raw': self.command_acceleration_raw,
            'configured_vmax': self.config['max_velocity_deg_s'],
            'configured_amax': self.config['max_acceleration_deg_s2'],
            'configured_max_step': self.config['max_step_deg'],
            'configured_max_velocity_deg_s': self.config['max_velocity_deg_s'],
            'configured_max_acceleration_deg_s2': self.config[
                'max_acceleration_deg_s2'],
            'configured_max_step_deg': self.config['max_step_deg'],
            'effective_step_velocity_cap': (
                self.config['max_step_deg'] * self.config['control_rate_hz']
            ),
            'effective_step_velocity_cap_actual': (
                self.config['max_step_deg'] * actual_control_rate_hz
            ),
            'required_step_for_vmax': (
                self.config['max_velocity_deg_s'] /
                self.config['control_rate_hz']
            ),
            'limiter_flags_valid': bool(current_limit_flags),
            'velocity_limited': any(entry['velocity_limited']
                                    for entry in current_limit_flags),
            'acceleration_limited': any(entry['acceleration_limited']
                                        for entry in current_limit_flags),
            'step_limited': any(entry['step_limited']
                                for entry in current_limit_flags),
            'can_state': self.driver.bus.state.name,
            'event_counts': dict(self.event_counts),
            'motor_fault_bytes': {
                f'0x{item.motor_id:02x}':
                self.feedback_by_id[item.feedback_id].fault
                for item in mapping.MIRABO_JOINTS
            },
            'timestamps_ns': dict(self.stage_stamps_ns),
            'rates_hz': {key: self.event_rate(key) for key in
                         self.event_stamps_ns},
            'joints': metrics,
        }
        diagnostic = String()
        diagnostic.data = json.dumps(data, separators=(',', ':'))
        self.diagnostics_pub.publish(diagnostic)

    def record_event(self, name, stamp_ns):
        stamps = self.event_stamps_ns[name]
        stamps.append(stamp_ns)
        cutoff = stamp_ns - 5_000_000_000
        while len(stamps) > 2 and stamps[0] < cutoff:
            stamps.popleft()

    def event_rate(self, name):
        stamps = self.event_stamps_ns[name]
        if len(stamps) < 2 or stamps[-1] <= stamps[0]:
            return 0.0
        return (len(stamps) - 1) * 1e9 / (stamps[-1] - stamps[0])

    def current_state(self, ready, reason):
        if self.fault is not None:
            return (TeleopState.ESTOP if self.fault.code == 'ESTOP'
                    else TeleopState.FAULT)
        if self.internal_armed:
            return TeleopState.ARMED
        if ready:
            return TeleopState.READY
        if 'origin' in reason.lower():
            return TeleopState.INIT
        if 'leader' in reason.lower():
            return TeleopState.WAIT_LEADER
        return TeleopState.WAIT_CAN

    def clear_fault(self, request, response):
        if self.fault is None:
            response.success = True
            response.message = 'No fault is latched'
            return response
        if self.fault.code == 'ESTOP':
            response.success = False
            response.message = 'Use clear_estop for manual E-stop'
            return response
        return self._clear_latch(response)

    def estop(self, request, response):
        self.disarm('manual E-stop; commands stopped, torque unchanged',
                    'ESTOP')
        response.success = True
        response.message = self.last_fault_reason
        return response

    def clear_estop(self, request, response):
        if self.fault is None or self.fault.code != 'ESTOP':
            response.success = False
            response.message = 'No manual E-stop is latched'
            return response
        return self._clear_latch(response)

    def _clear_latch(self, response):
        if self.get_parameter('armed').value or self.internal_armed:
            response.success = False
            response.message = 'Disarm before clearing a fault'
            return response
        ready, reason = self.ready()
        if not ready:
            response.success = False
            response.message = reason
            return response
        self.fault = None
        self.last_fault_reason = ''
        response.success = True
        response.message = 'Ready; arm explicitly to send commands'
        self.get_logger().info('Fault cleared; teleop is READY')
        return response

    def disarm(self, reason, code='TELEOP_FAULT'):
        if self.fault is not None and self.fault.code == 'ESTOP' and code != 'ESTOP':
            return
        if reason != self.last_fault_reason:
            self.get_logger().warning(f'Mirabo teleop disarmed: {reason}')
        self.internal_armed = False
        self.last_sent_stamps = None
        if self.get_parameter('armed').value:
            self.set_parameters([Parameter('armed', value=False)])
        self.last_fault_reason = reason
        self.fault = FaultRecord(code, reason, self.now_sec())

    def can_fault(self, reason, code='CAN_FAULT'):
        for feedback in self.feedback_by_id.values():
            feedback.stamp_sec = -math.inf
        self.disarm(reason, code)

    def mapping_for_feedback(self, feedback_id):
        for item in mapping.MIRABO_JOINTS:
            if item.feedback_id == feedback_id:
                return item
        raise KeyError(feedback_id)

    def now_sec(self):
        return time.monotonic()

    def destroy_node(self):
        try:
            self.driver.shutdown()
        finally:
            super().destroy_node()


def main(args=None):
    """Run only the Mirabo controller; use the launch file for the full chain."""
    command_args = list(sys.argv[1:] if args is None else args)
    if '--arm' in command_args:
        raise ValueError('--arm is disabled; arm explicitly after startup')
    rclpy.init(args=command_args)
    node = None
    try:
        node = MiraboTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
