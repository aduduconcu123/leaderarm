import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64

from feetech_driver.motor import STS3250
from feetech_driver.protocol import FeetechProtocol


JOINT_TO_MOTOR = {
    "joint_1": 1,
    "joint_2": 3,
    "joint_3": 2,
}


class FeetechNode(Node):
    def __init__(self):
        super().__init__("feetech_node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 1000000)
        self.declare_parameter("publish_rate", 20.0)

        # Start with reading only while calibration is unfinished.
        self.declare_parameter("enable_commands", False)

        port = self.get_parameter("port").value
        baudrate = self.get_parameter("baudrate").value
        rate = float(self.get_parameter("publish_rate").value)
        self.enable_commands = self.get_parameter(
            "enable_commands"
        ).value

        if not math.isfinite(rate) or rate <= 0:
            raise ValueError("publish_rate must be positive")

        # One serial connection for all three motors.
        self.protocol = FeetechProtocol(
            port=port,
            baudrate=baudrate,
            timeout=0.2,
        )

        self.motors = {}
        self.command_subscriptions = []

        try:
            for joint_name, motor_id in JOINT_TO_MOTOR.items():
                motor = STS3250(
                    motor_id=motor_id,
                    protocol=self.protocol,
                )

                if not motor.ping():
                    raise RuntimeError(
                        f"{joint_name}: ID {motor_id} did not respond"
                    )

                self.motors[joint_name] = motor
                self.get_logger().info(
                    f"{joint_name} -> motor ID {motor_id}: connected"
                )

            self.state_pub = self.create_publisher(
                JointState,
                "/feetech/joint_states",
                10,
            )

            if self.enable_commands:
                for joint_name in self.motors:
                    self.command_subscriptions.append(
                        self.create_subscription(
                            Float64,
                            f"/feetech/{joint_name}/position_command",
                            lambda msg, name=joint_name:
                                self.position_callback(name, msg),
                            10,
                        )
                    )

                    self.command_subscriptions.append(
                        self.create_subscription(
                            Bool,
                            f"/feetech/{joint_name}/torque_enable",
                            lambda msg, name=joint_name:
                                self.torque_callback(name, msg),
                            10,
                        )
                    )

            self.timer = self.create_timer(
                1.0 / rate,
                self.publish_state,
            )

        except Exception:
            self.protocol.close()
            raise

        self.get_logger().info(
            f"Reading 3 motors at {rate:.1f} Hz; "
            f"commands_enabled={self.enable_commands}"
        )

    def publish_state(self):
        positions = []

        for joint_name, motor in self.motors.items():
            try:
                raw = motor.read_raw_position()

                if raw is None or not 0 <= raw <= 4095:
                    raise ValueError(f"Invalid encoder value: {raw}")

                # Absolute encoder angle; not calibrated joint angle.
                angle_rad = raw * math.tau / 4096.0
                positions.append(angle_rad)

            except Exception as exc:
                self.get_logger().warning(
                    f"{joint_name}, ID {motor.motor_id}: {exc}",
                    throttle_duration_sec=2.0,
                )

                # Do not publish a partial or fabricated joint state.
                return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.motors)
        msg.position = positions

        # Leave velocity and effort empty:
        # raw speed/load are not ROS rad/s and N.m.
        self.state_pub.publish(msg)

    def position_callback(self, joint_name, msg):
        # Command is an absolute encoder angle in radians.
        # Mechanical limits must be checked before enabling commands.
        angle_rad = float(msg.data)

        if (
            not math.isfinite(angle_rad)
            or not 0.0 <= angle_rad < math.tau
        ):
            self.get_logger().error(
                f"{joint_name}: angle must be in [0, 2*pi) radians"
            )
            return

        raw = min(
            4095,
            int(round(angle_rad * 4096.0 / math.tau)),
        )

        try:
            self.motors[joint_name].set_position(raw)
        except Exception as exc:
            self.get_logger().error(
                f"{joint_name}: position command failed: {exc}"
            )

    def torque_callback(self, joint_name, msg):
        motor = self.motors[joint_name]

        try:
            if msg.data:
                motor.enable_torque()
            else:
                motor.disable_torque()

            self.get_logger().info(
                f"{joint_name}: torque={msg.data}"
            )

        except Exception as exc:
            self.get_logger().error(
                f"{joint_name}: torque command failed: {exc}"
            )

    def destroy_node(self):
        # Close communication without changing motor torque.
        # Exiting this node is not an emergency stop.
        try:
            self.protocol.close()
        finally:
            return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = FeetechNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()