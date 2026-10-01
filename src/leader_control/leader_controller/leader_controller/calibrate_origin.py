#!/usr/bin/env python3
"""Capture the current leader and Mirabo pose as a persistent software zero."""

import math
import time

import can
from feetech_driver.node import FeetechNode
from leader_controller import mapping
from leader_controller.mirabo_can import unpack_feedback
from leader_state import origin as saved_origin
from leader_state.state import LeaderState
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState


CAPTURE_TIMEOUT_SEC = 10.0
SAMPLE_TIMEOUT_SEC = 0.5
MAX_SAMPLE_SKEW_SEC = 0.1


class OriginCapture(Node):
    """Wait for fresh, fault-free samples and write a software origin."""

    def __init__(self):
        """Open the CAN feedback reader without enabling motor commands."""
        super().__init__('leader_mirabo_origin_capture')
        self.declare_parameter(
            'origin_file', str(saved_origin.default_origin_file())
        )
        self.origin_file = self.get_parameter('origin_file').value
        self.previous_origin = saved_origin.load_origin(self.origin_file)
        self.leader_positions = None
        self.leader_stamp = -math.inf
        self.motor_angles = {}
        self.motor_stamps = {}
        self.done = False
        self.error = None

        try:
            self.bus = can.Bus(
                interface='socketcan', channel=mapping.CAN_INTERFACE,
                bitrate=mapping.CAN_BITRATE, ignore_rx_error_frames=False,
                ignore_config=True,
            )
        except (can.CanError, OSError):
            super().destroy_node()
            raise

        self.leader_sub = self.create_subscription(
            JointState, '/leader/joint_states', self.leader_callback, 1,
        )
        self.timer = self.create_timer(0.005, self.capture_step)
        self.get_logger().info(
            f'Capturing read-only origin to {self.origin_file}; '
            'no position or torque commands will be sent.'
        )

    def leader_callback(self, msg):
        """Keep the latest complete calibrated leader sample."""
        names = list(msg.name)
        if (len(names) != len(saved_origin.LEADER_JOINTS)
                or len(msg.position) != len(names)
                or set(names) != set(saved_origin.LEADER_JOINTS)
                or not all(math.isfinite(value) for value in msg.position)):
            self.leader_positions = None
            self.leader_stamp = -math.inf
            return
        self.leader_positions = dict(zip(names, msg.position))
        self.leader_stamp = time.monotonic()

    def capture_step(self):
        """Save one origin once all samples are fresh and close in time."""
        if self.done or self.error:
            return
        try:
            self.read_can()
            if self.error:
                return
            now = time.monotonic()
            if self.leader_positions is None or (
                now - self.leader_stamp > SAMPLE_TIMEOUT_SEC
            ):
                return
            if any(
                now - self.motor_stamps.get(item.motor_id, -math.inf)
                > SAMPLE_TIMEOUT_SEC
                for item in mapping.MIRABO_JOINTS
            ):
                return
            stamps = [self.leader_stamp] + [
                self.motor_stamps[item.motor_id]
                for item in mapping.MIRABO_JOINTS
            ]
            if max(stamps) - min(stamps) > MAX_SAMPLE_SKEW_SEC:
                return
            previous_leader = (
                self.previous_origin['leader_radians']
                if self.previous_origin else {}
            )
            data = {
                'version': 1,
                'leader_radians': {
                    joint: self.leader_positions[joint]
                    + previous_leader.get(joint, 0.0)
                    for joint in saved_origin.LEADER_JOINTS
                },
                'mirabo_can_degrees': {
                    f'0x{item.motor_id:02x}': self.motor_angles[item.motor_id]
                    for item in mapping.MIRABO_JOINTS
                },
            }
            saved_origin.save_origin(self.origin_file, data)
            self.done = True
            self.get_logger().info(
                f'Origin saved to {self.origin_file}: '
                f"leader={data['leader_radians']}; "
                f"Mirabo CAN degrees={data['mirabo_can_degrees']}"
            )
        except (can.CanError, OSError, ValueError) as exc:
            self.error = str(exc)

    def read_can(self):
        """Read motor angles while rejecting malformed and fault frames."""
        feedback_ids = {
            item.feedback_id: item.motor_id
            for item in mapping.MIRABO_JOINTS
        }
        for _ in range(mapping.MAX_CAN_FRAMES_PER_READ):
            frame = self.bus.recv(timeout=0.0)
            if frame is None:
                return
            if frame.is_error_frame:
                if frame.arbitration_id & 0x40:
                    raise ValueError(
                        f'CAN BUS-OFF error frame 0x{frame.arbitration_id:X}'
                    )
                continue
            if not frame.is_extended_id:
                continue
            motor_id = feedback_ids.get(frame.arbitration_id)
            if motor_id is None:
                continue
            if frame.is_remote_frame or frame.is_fd or frame.dlc != 8:
                continue
            try:
                angle, fault = unpack_feedback(frame.data)
            except ValueError:
                continue
            if fault:
                raise ValueError(
                    f'Mirabo 0x{motor_id:02X} fault=0x{fault:02X}'
                )
            self.motor_angles[motor_id] = angle
            self.motor_stamps[motor_id] = time.monotonic()
        return

    def destroy_node(self):
        """Close the CAN socket without changing motor torque."""
        try:
            self.bus.shutdown()
        finally:
            super().destroy_node()


def main(args=None):
    """Run the read-only capture chain once, then exit."""
    rclpy.init(args=args)
    nodes = []
    executor = None
    try:
        nodes.append(FeetechNode(read_only=True))
        nodes.append(LeaderState())
        capture = OriginCapture()
        nodes.append(capture)
        executor = MultiThreadedExecutor(num_threads=3)
        for node in nodes:
            executor.add_node(node)
        deadline = time.monotonic() + CAPTURE_TIMEOUT_SEC
        while (rclpy.ok() and not capture.done and not capture.error
               and time.monotonic() < deadline):
            executor.spin_once(timeout_sec=0.05)
        if capture.error:
            raise RuntimeError(f'Origin calibration failed: {capture.error}')
        if not capture.done:
            raise RuntimeError('Origin calibration timed out waiting for '
                               'leader and both Mirabo feedback streams')
    finally:
        if executor is not None:
            executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
