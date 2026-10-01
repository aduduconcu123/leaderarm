import math
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState

from feetech_driver import calibration as feetech_calibration
from feetech_driver.calibration import CalibrationManager, encoder_offset
from leader_state import origin as saved_origin


JOINT_TO_MOTOR = {
    "joint_1": 1,
    "joint_2": 3,
    "joint_3": 2,
}
JOINT_TO_CALIBRATION_NAME = {
    "joint_1": "J1",
    "joint_2": "J2",
    "joint_3": "J3",
}
ENCODER_COUNTS = 4096


def default_calibration_file():
    driver_root = Path(feetech_calibration.__file__).resolve().parent.parent
    if (driver_root / "tools" / "calibrate_keyboard.py").is_file():
        return driver_root / "calibration" / "calibration.json"
    return (
        Path(get_package_share_directory("feetech_driver"))
        / "calibration"
        / "calibration.json"
    )


class LeaderState(Node):
    def __init__(self):
        super().__init__("leader_state")

        self.joint_names = list(JOINT_TO_MOTOR)
        self.declare_parameter(
            "calibration_file", str(default_calibration_file())
        )
        calibration_file = Path(
            self.get_parameter("calibration_file").value
        ).expanduser()
        self.calibration = self._load_calibration(calibration_file)
        self.declare_parameter(
            "origin_file", str(saved_origin.default_origin_file())
        )
        self.origin_file = Path(
            self.get_parameter("origin_file").value
        ).expanduser()
        self.origin = saved_origin.load_origin(self.origin_file)

        self.subscription = self.create_subscription(
            JointState,
            "/feetech/joint_states",
            self.joint_state_callback,
            10,
        )
        self.publisher = self.create_publisher(
            JointState,
            "/leader/joint_states",
            10,
        )

        self.get_logger().info(
            f"Leader state started with calibration: {calibration_file}"
        )
        if self.origin is not None:
            self.get_logger().info(f"Leader software origin: {self.origin_file}")

    def _load_calibration(self, calibration_file):
        manager = CalibrationManager(calibration_file)
        try:
            if not manager.load():
                raise RuntimeError(
                    f"Calibration file not found: {calibration_file}"
                )
        except Exception as exc:
            raise RuntimeError(
                f"Cannot load calibration file {calibration_file}: {exc}"
            ) from exc

        calibrations = {}
        for joint, motor_id in JOINT_TO_MOTOR.items():
            calibration = manager.get_motor(motor_id)
            expected_name = JOINT_TO_CALIBRATION_NAME[joint]
            if calibration is None:
                raise RuntimeError(
                    f"Calibration missing for {joint} (motor ID {motor_id}) "
                    f"in {calibration_file}"
                )
            if calibration.joint != expected_name:
                raise RuntimeError(
                    f"Calibration ID {motor_id} is labeled "
                    f"{calibration.joint!r}; expected {expected_name!r} "
                    f"for {joint}"
                )
            if (
                not isinstance(calibration.zero_raw, int)
                or not 0 <= calibration.zero_raw < ENCODER_COUNTS
                or calibration.direction not in (-1, 1)
                or not isinstance(calibration.limit_min, int)
                or not 0 <= calibration.limit_min < ENCODER_COUNTS
                or not isinstance(calibration.limit_max, int)
                or not 0 <= calibration.limit_max < ENCODER_COUNTS
            ):
                raise RuntimeError(
                    f"Invalid calibration values for {joint} (motor ID {motor_id})"
                )

            low = encoder_offset(calibration.limit_min, calibration.zero_raw)
            high = encoder_offset(calibration.limit_max, calibration.zero_raw)
            if not -2048 < low <= 0 <= high < 2048 or low >= high:
                raise RuntimeError(
                    f"Invalid limits for {joint} (motor ID {motor_id}): "
                    "limits must bracket zero within half a revolution"
                )

            calibrations[joint] = (calibration, low, high)

        return calibrations

    def joint_state_callback(self, msg):
        if (
            len(msg.name) != len(self.joint_names)
            or len(msg.position) != len(msg.name)
            or len(set(msg.name)) != len(msg.name)
            or set(msg.name) != set(self.joint_names)
        ):
            self.get_logger().warning(
                "Dropping /feetech/joint_states: expected one position for "
                "each of joint_1, joint_2, joint_3",
                throttle_duration_sec=2.0,
            )
            return

        input_positions = dict(zip(msg.name, msg.position))
        output_positions = []
        for joint in self.joint_names:
            angle = input_positions[joint]
            if not math.isfinite(angle) or not 0.0 <= angle < math.tau:
                self.get_logger().warning(
                    f"Dropping /feetech/joint_states: invalid angle "
                    f"{angle!r} for {joint}",
                    throttle_duration_sec=2.0,
                )
                return

            raw = int(round(angle * ENCODER_COUNTS / math.tau)) % ENCODER_COUNTS
            calibration, low, high = self.calibration[joint]
            delta = encoder_offset(raw, calibration.zero_raw)
            if not low <= delta <= high:
                self.get_logger().warning(
                    f"Dropping /feetech/joint_states: {joint} raw={raw}, "
                    f"offset={delta} outside calibrated limits [{low}, {high}]",
                    throttle_duration_sec=2.0,
                )
                return

            output_positions.append(
                delta * calibration.direction * math.tau / ENCODER_COUNTS
                - (self.origin["leader_radians"][joint]
                   if self.origin is not None else 0.0)
            )

        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = list(self.joint_names)
        state.position = output_positions
        self.publisher.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LeaderState()
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
