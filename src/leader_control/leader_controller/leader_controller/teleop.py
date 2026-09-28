#!/usr/bin/env python3
"""ROS 2 teleop node from calibrated leader joints to Mirabo CAN motors."""

from dataclasses import dataclass
import math
import struct
import sys
import time

import can
from feetech_driver.node import FeetechNode
from leader_controller import mapping
from leader_state.state import LeaderState
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String


def clamp(value, minimum, maximum):
    """Clamp value to [minimum, maximum]."""
    return max(minimum, min(value, maximum))


def joint_delta_to_can_delta(delta_deg):
    """Convert output joint delta to CAN angle delta."""
    if mapping.CAN_ANGLE_IS_MOTOR_SHAFT:
        return delta_deg * mapping.GEAR_RATIO
    return delta_deg


def pack_position_command(angle_deg, speed, accel):
    """Pack Mirabo extended-CAN position command payload."""
    if not all(math.isfinite(value) for value in (angle_deg, speed, accel)):
        raise ValueError('CAN command values must be finite')
    try:
        # Match C++'s float argument, then its truncating integer conversion.
        cpp_angle = struct.unpack('>f', struct.pack('>f', angle_deg))[0]
        return struct.pack(
            '>ihh', int(cpp_angle * 10000.0),
            int(speed / 10.0), int(accel / 10.0),
        )
    except (struct.error, OverflowError) as exc:
        raise ValueError('CAN command exceeds its integer range') from exc


def unpack_feedback(data):
    """Unpack Mirabo feedback: angle in degrees and fault byte."""
    if len(data) != 8:
        raise ValueError('Mirabo feedback payload must contain 8 bytes')
    raw_angle = struct.unpack('>h', bytes(data[:2]))[0]
    return raw_angle / 10.0, int(data[7])


@dataclass
class MotorFeedback:
    """Most recent feedback from one Mirabo motor."""

    angle_deg: float = math.nan
    fault: int = 0
    stamp_sec: float = -math.inf


class MiraboTeleop(Node):
    """Bridge /leader/joint_states to Mirabo position commands on can0."""

    def __init__(self, arm_on_ready=False, **kwargs):
        super().__init__('leader_mirabo_teleop', **kwargs)

        self.startup_arm_pending = arm_on_ready

        self.declare_parameter(
            'armed', False,
            ParameterDescriptor(
                description='Enable at runtime only after inputs are ready; '
                'startup overrides are ignored. False stops commands, '
                'but does not disable torque.'
            ),
            ignore_override=True,
        )

        try:
            self.bus = can.Bus(
                interface='socketcan',
                channel=mapping.CAN_INTERFACE,
                bitrate=mapping.CAN_BITRATE,
                ignore_rx_error_frames=False,
                ignore_config=True,
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

        self.baseline_leader = {}
        self.baseline_motor = {}
        self.last_command = {}
        self.internal_armed = False
        self.last_fault_reason = ''
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

        period = 1.0 / mapping.CONTROL_RATE_HZ
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.can_timer = self.create_timer(
            0.005,
            self.read_can_feedback,
            clock=self.steady_clock,
        )
        self.control_timer = self.create_timer(
            period,
            self.control_step,
            clock=self.steady_clock,
        )

        self.get_logger().info(
            'Mirabo teleop started disarmed. '
            + ('--arm will enable when all inputs are ready.'
               if arm_on_ready else
               "Set parameter 'armed' true after all inputs are ready.")
        )
        self.get_logger().info(
            f'CAN angle scale={joint_delta_to_can_delta(1.0):g}; '
            f'gear ratio={mapping.GEAR_RATIO:g}. '
            'Stopping commands does not disable torque.'
        )
        self.get_logger().warning(
            'J2 sign uses the existing mapping; confirm on the real machine. '
            'CAN limits are software limits from the C++ implementation.'
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
                ready, reason = self.ready()
                if not ready:
                    return SetParametersResult(
                        successful=False, reason=reason
                    )
        return SetParametersResult(successful=True)

    def parameters_changed(self, parameters):
        for parameter in parameters:
            if parameter.name == 'armed':
                self.last_fault_reason = ''
                if not parameter.value:
                    self.startup_arm_pending = False
                    self.internal_armed = False
                    self.last_sent_stamps = None
                    self.get_logger().info(
                        'Disarmed: commands stopped; torque is unchanged.'
                    )

    def leader_callback(self, msg):
        names = list(msg.name)
        # An invalid message must not leave the previous sample armable.
        self.leader_positions = {}
        self.leader_stamp_sec = -math.inf
        if len(msg.position) != len(names):
            self.disarm('leader JointState has mismatched name/position')
            return
        if len(set(names)) != len(names):
            self.disarm('leader JointState has duplicate names')
            return

        positions = dict(zip(names, msg.position))
        for item in mapping.MIRABO_JOINTS:
            if item.leader_joint not in positions:
                self.disarm(
                    f'leader JointState missing {item.leader_joint}'
                )
                return
            if not math.isfinite(positions[item.leader_joint]):
                self.disarm(
                    f'leader JointState has invalid {item.leader_joint}'
                )
                return

        self.leader_positions = positions
        self.leader_stamp_sec = self.now_sec()

    def read_can_feedback(self):
        try:
            for _ in range(mapping.MAX_CAN_FRAMES_PER_READ):
                msg = self.bus.recv(timeout=0.0)
                if msg is None:
                    return
                # SocketCAN's Bus.state is always ACTIVE in python-can 4.3.1.
                # Kernel error frames and feedback timeouts are also required.
                if msg.is_error_frame:
                    self.can_fault(
                        f'CAN error frame 0x{msg.arbitration_id:X}, '
                        f'data={msg.data.hex()}'
                    )
                    continue
                if not msg.is_extended_id:
                    continue
                feedback = self.feedback_by_id.get(msg.arbitration_id)
                if feedback is None:
                    continue
                if msg.is_remote_frame or msg.is_fd or msg.dlc != 8:
                    feedback.stamp_sec = -math.inf
                    self.disarm(
                        f'invalid CAN feedback 0x{msg.arbitration_id:X}'
                    )
                    continue
                try:
                    angle_deg, fault = unpack_feedback(msg.data)
                except ValueError as exc:
                    feedback.stamp_sec = -math.inf
                    self.disarm(
                        f'invalid CAN feedback 0x{msg.arbitration_id:X}: {exc}'
                    )
                    continue
                feedback.angle_deg = angle_deg
                feedback.fault = fault
                feedback.stamp_sec = self.now_sec()
                if fault:
                    item = self.mapping_for_feedback(msg.arbitration_id)
                    self.disarm(
                        f'Mirabo 0x{item.motor_id:02X} fault=0x{fault:02X}'
                    )
            self.can_fault('CAN receive backlog exceeds per-read limit')
        except (can.CanError, OSError) as exc:
            self.can_fault(f'CAN receive error: {exc}')

    def control_step(self):
        # Consume queued faults before considering either motor command.
        self.read_can_feedback()
        requested_armed = bool(self.get_parameter('armed').value)
        ready, reason = self.ready()

        if self.startup_arm_pending and ready and not requested_armed:
            self.startup_arm_pending = False
            result = self.set_parameters([Parameter('armed', value=True)])[0]
            if result.successful:
                requested_armed = True
            else:
                self.disarm(f'cannot arm at startup: {result.reason}')

        if not requested_armed:
            state = 'FAULT' if self.last_fault_reason else (
                'READY' if ready else 'DISARMED'
            )
            self.publish_status(state, self.last_fault_reason or reason)
            self.publish_feedback()
            return

        if not ready:
            self.disarm(reason)
            self.publish_status('FAULT')
            self.publish_feedback()
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
            return

        try:
            commands = [
                (item, self.command_for_mapping(item))
                for item in mapping.MIRABO_JOINTS
            ]
            for item, command_deg in commands:
                self.send_position_command(item, command_deg)
                self.last_command[item.motor_id] = command_deg
            self.last_sent_stamps = stamps
        except (can.CanError, OSError, ValueError) as exc:
            self.can_fault(f'CAN command error: {exc}')

        self.publish_status('ARMED' if self.internal_armed else 'FAULT')
        self.publish_feedback()

    def ready(self):
        now = self.now_sec()
        if self.bus.state != can.BusState.ACTIVE:
            return False, f'CAN bus state is {self.bus.state.name}'
        if now - self.leader_stamp_sec > mapping.LEADER_TIMEOUT_SEC:
            return False, 'timeout waiting for /leader/joint_states'

        for item in mapping.MIRABO_JOINTS:
            if item.leader_joint not in self.leader_positions:
                return False, f'missing leader {item.leader_joint}'
            feedback = self.feedback_by_id[item.feedback_id]
            if now - feedback.stamp_sec > mapping.FEEDBACK_TIMEOUT_SEC:
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
            if not item.min_deg <= feedback.angle_deg <= item.max_deg:
                return False, (
                    f'Mirabo 0x{item.motor_id:02X} CAN angle '
                    f'{feedback.angle_deg:g} outside software limits '
                    f'[{item.min_deg:g}, {item.max_deg:g}]'
                )
        return True, ''

    def arm_from_current_positions(self):
        self.baseline_leader = {}
        self.baseline_motor = {}
        self.last_command = {}

        for item in mapping.MIRABO_JOINTS:
            leader_rad = self.leader_positions[item.leader_joint]
            feedback = self.feedback_by_id[item.feedback_id]
            self.baseline_leader[item.leader_joint] = math.degrees(
                leader_rad
            )
            self.baseline_motor[item.motor_id] = feedback.angle_deg
            self.last_command[item.motor_id] = feedback.angle_deg

        self.internal_armed = True
        self.last_sent_stamps = None
        self.last_fault_reason = ''
        self.get_logger().info(
            'Armed Mirabo teleop from current leader and motor positions.'
        )

    def command_for_mapping(self, item):
        leader_deg = math.degrees(
            self.leader_positions[item.leader_joint]
        )
        leader_delta = leader_deg - self.baseline_leader[item.leader_joint]
        target_delta = joint_delta_to_can_delta(
            leader_delta * item.sign
        )
        target = self.baseline_motor[item.motor_id] + target_delta
        target = clamp(target, item.min_deg, item.max_deg)

        previous = self.last_command[item.motor_id]
        max_step = mapping.MAX_STEP_DEG_PER_CYCLE
        target = clamp(target, previous - max_step, previous + max_step)
        return clamp(target, item.min_deg, item.max_deg)

    def send_position_command(self, item, angle_deg):
        if not self.internal_armed or not self.get_parameter('armed').value:
            raise ValueError('cannot send a position command while disarmed')
        if not math.isfinite(angle_deg) or not (
            item.min_deg <= angle_deg <= item.max_deg
        ):
            raise ValueError('CAN position command is outside software limits')
        data = pack_position_command(
            angle_deg,
            mapping.COMMAND_SPEED,
            mapping.COMMAND_ACCEL,
        )
        msg = can.Message(
            arbitration_id=item.command_id,
            data=data,
            is_extended_id=True,
        )
        self.bus.send(msg, timeout=0.0)

    def publish_status(self, state, reason=''):
        parts = [
            f'state={state}',
            f"armed_param={bool(self.get_parameter('armed').value)}",
            f'internal_armed={self.internal_armed}',
            f'leader_age_sec={self.now_sec() - self.leader_stamp_sec:.3f}',
        ]
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

        msg = String()
        msg.data = '; '.join(parts)
        self.status_pub.publish(msg)

    def publish_feedback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            f'mirabo_0x{item.motor_id:02X}'
            for item in mapping.MIRABO_JOINTS
        ]
        msg.position = [
            math.radians(
                self.feedback_by_id[item.feedback_id].angle_deg
                / joint_delta_to_can_delta(1.0)
            )
            for item in mapping.MIRABO_JOINTS
        ]
        self.feedback_pub.publish(msg)

    def disarm(self, reason):
        if reason != self.last_fault_reason:
            self.get_logger().warning(f'Mirabo teleop disarmed: {reason}')
        self.internal_armed = False
        self.startup_arm_pending = False
        self.last_sent_stamps = None
        if self.get_parameter('armed').value:
            self.set_parameters([Parameter('armed', value=False)])
        self.last_fault_reason = reason

    def can_fault(self, reason):
        for feedback in self.feedback_by_id.values():
            feedback.stamp_sec = -math.inf
        self.disarm(reason)

    def mapping_for_feedback(self, feedback_id):
        for item in mapping.MIRABO_JOINTS:
            if item.feedback_id == feedback_id:
                return item
        raise KeyError(feedback_id)

    def now_sec(self):
        return time.monotonic()

    def destroy_node(self):
        try:
            self.bus.shutdown()
        finally:
            super().destroy_node()


def main(args=None):
    """Run the Feetech, calibration and Mirabo nodes with one command."""
    command_args = list(sys.argv[1:] if args is None else args)
    arm_on_ready = '--arm' in command_args
    ros_args = [arg for arg in command_args if arg != '--arm']

    rclpy.init(args=ros_args)
    nodes = []
    executor = None
    try:
        nodes.append(FeetechNode())
        nodes.append(LeaderState())
        nodes.append(MiraboTeleop(arm_on_ready=arm_on_ready))

        executor = MultiThreadedExecutor(num_threads=3)
        for node in nodes:
            executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
