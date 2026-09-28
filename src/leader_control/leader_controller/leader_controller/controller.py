#!/usr/bin/env python3

"""
Leader arm controller.

Responsibilities:
    1. Load motor calibration from calibration.json
    2. Convert Feetech RAW position -> joint angle
    3. Check joint limits
    4. Calculate forward kinematics
    5. Calculate COM positions
    6. Calculate gravity compensation torque
    7. Provide teleoperation utilities

Motor mapping:
    ID 1 -> J1
    ID 2 -> J2
    ID 3 -> J3

IMPORTANT:
    Calibration values are NOT hard-coded here.

    They are loaded from:
        calibration/calibration.json
"""

import math
from pathlib import Path

import numpy as np

from feetech_driver.calibration import CalibrationManager


# ============================================================
# CALIBRATION
# ============================================================

CALIBRATION_FILE = (
    Path(__file__).resolve().parent.parent
    / "calibration"
    / "calibration.json"
)

MOTOR_IDS = {
    "J1": 1,
    "J2": 2,
    "J3": 3,
}


# ============================================================
# MASTER GEOMETRY
# ============================================================

# mm

L1 = 300.0
L2 = 408.51
L3 = 346.50


# ============================================================
# MASS
# ============================================================

# kg

M_LINK2 = 0.14666
M_LINK3 = 0.15075
M_LINK4 = 0.07854

M_MOTOR = 0.0745

GRAVITY = 9.81


# ============================================================
# DH TRANSFORM
# ============================================================

def dh_transform(theta, d, a, alpha):
    """
    Standard DH transformation.

    theta : rad
    d     : m
    a     : m
    alpha : rad
    """

    ct = math.cos(theta)
    st = math.sin(theta)

    ca = math.cos(alpha)
    sa = math.sin(alpha)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,     1.0],
    ])


# ============================================================
# FORWARD KINEMATICS
# ============================================================

def forward_kinematics(q):
    """
    Forward kinematics.

    q = [q1, q2, q3] in radians.

    Returns:
        T01
        T02
        T03
    """

    q1, q2, q3 = q

    T01 = dh_transform(
        q1,
        0.0,
        L1 / 1000.0,
        math.radians(90.0),
    )

    T12 = dh_transform(
        q2,
        0.0,
        L2 / 1000.0,
        0.0,
    )

    T23 = dh_transform(
        q3,
        0.0,
        L3 / 1000.0,
        0.0,
    )

    T02 = T01 @ T12
    T03 = T02 @ T23

    return T01, T02, T03


def end_effector_position(q):
    """
    Return end-effector position in meters.
    """

    _, _, T03 = forward_kinematics(q)

    return T03[:3, 3]


# ============================================================
# CENTER OF MASS
# ============================================================

def center_of_mass_positions(q):
    """
    Return COM positions of Link 2, Link 3 and Link 4.

    Positions are in meters.

    NOTE:
        These are currently estimated COM positions.
        Replace with SolidWorks COM data when available.
    """

    q1, q2, q3 = q

    T01, T02, _ = forward_kinematics(q)

    # --------------------------------------------------------
    # Link 2 COM
    # --------------------------------------------------------

    T_com2 = dh_transform(
        q1,
        0.0,
        (L1 / 2.0) / 1000.0,
        math.radians(90.0),
    )

    p_com2 = T_com2[:3, 3]

    # --------------------------------------------------------
    # Link 3 COM
    # --------------------------------------------------------

    T_com3 = T01 @ dh_transform(
        q2,
        0.0,
        (L2 / 2.0) / 1000.0,
        0.0,
    )

    p_com3 = T_com3[:3, 3]

    # --------------------------------------------------------
    # Link 4 COM
    # --------------------------------------------------------

    T_com4 = T02 @ dh_transform(
        q3,
        0.0,
        (L3 / 2.0) / 1000.0,
        0.0,
    )

    p_com4 = T_com4[:3, 3]

    return (
        p_com2,
        p_com3,
        p_com4,
    )


# ============================================================
# NUMERICAL JACOBIAN
# ============================================================

def numerical_jacobian(
    position_function,
    q,
    epsilon=1e-6,
):
    """
    Calculate numerical translational Jacobian.

    Returns:
        Jv : 3 x n
    """

    q = np.asarray(
        q,
        dtype=float,
    )

    p0 = np.asarray(
        position_function(q)
    )

    J = np.zeros(
        (3, len(q))
    )

    for i in range(len(q)):

        q_plus = q.copy()
        q_minus = q.copy()

        q_plus[i] += epsilon
        q_minus[i] -= epsilon

        p_plus = np.asarray(
            position_function(q_plus)
        )

        p_minus = np.asarray(
            position_function(q_minus)
        )

        J[:, i] = (
            p_plus - p_minus
        ) / (2.0 * epsilon)

    return J


# ============================================================
# GRAVITY TORQUE
# ============================================================

def gravity_torque(q):
    """
    Calculate gravitational torque.

    q:
        [q1, q2, q3] in radians

    Returns:
        tau in Nm
    """

    masses = [
        M_LINK2,
        M_LINK3,
        M_LINK4,
    ]

    def com_position(index, q_input):

        positions = center_of_mass_positions(
            q_input
        )

        return positions[index]

    tau = np.zeros(3)

    # Gravity force
    Fg = np.array([
        0.0,
        0.0,
        -GRAVITY,
    ])

    for i, mass in enumerate(masses):

        Jv = numerical_jacobian(
            lambda qx: com_position(
                i,
                qx,
            ),
            q,
        )

        tau += (
            Jv.T
            @ (mass * Fg)
        )

    return tau


# ============================================================
# CALIBRATION CONTROLLER
# ============================================================

class LeaderController:
    """
    Main leader-arm controller.

    Loads calibration automatically from calibration.json.
    """

    def __init__(
        self,
        calibration_file=CALIBRATION_FILE,
    ):

        self.calibration_file = Path(
            calibration_file
        )

        self.calibration = CalibrationManager(
            self.calibration_file
        )

        if not self.calibration.load():

            raise RuntimeError(
                "Calibration file not found: "
                f"{self.calibration_file}"
            )

        # ----------------------------------------------------
        # Load calibration for all 3 joints
        # ----------------------------------------------------

        self.joint_calibration = {}

        for joint, motor_id in MOTOR_IDS.items():

            motor_calib = (
                self.calibration.get_motor(
                    motor_id
                )
            )

            if motor_calib is None:

                raise RuntimeError(
                    f"Calibration for {joint} "
                    f"(Motor ID {motor_id}) "
                    "not found."
                )

            self.joint_calibration[joint] = (
                motor_calib
            )

    # ========================================================
    # RAW -> JOINT ANGLE
    # ========================================================

    def raw_to_joint_angle(
        self,
        motor_id,
        raw,
    ):
        """
        Convert Feetech RAW position to joint angle.

        Returns:
            angle in radians
        """

        motor_calib = (
            self.calibration.get_motor(
                motor_id
            )
        )

        if motor_calib is None:

            raise RuntimeError(
                f"No calibration for motor "
                f"ID {motor_id}."
            )

        angle_deg = (
            motor_calib.raw_to_angle(raw)
        )

        return math.radians(
            angle_deg
        )

    # ========================================================
    # READ ALL JOINT ANGLES
    # ========================================================

    def raw_positions_to_q(
        self,
        raw_positions,
    ):
        """
        Convert all motor RAW positions to q.

        raw_positions:
            {
                1: raw_j1,
                2: raw_j2,
                3: raw_j3,
            }

        Returns:
            numpy array [q1, q2, q3]
            in radians.
        """

        q = np.zeros(3)

        for index, motor_id in enumerate(
            [1, 2, 3]
        ):

            if motor_id not in raw_positions:

                raise ValueError(
                    f"Missing RAW position "
                    f"for motor {motor_id}."
                )

            q[index] = (
                self.raw_to_joint_angle(
                    motor_id,
                    raw_positions[motor_id],
                )
            )

        return q

    # ========================================================
    # JOINT LIMIT CHECK
    # ========================================================

    def is_within_limit(
        self,
        motor_id,
        raw,
    ):
        """
        Check RAW position against
        calibration MIN/MAX.
        """

        motor_calib = (
            self.calibration.get_motor(
                motor_id
            )
        )

        if motor_calib is None:

            return False

        return (
            motor_calib.limit_min
            <= raw
            <= motor_calib.limit_max
        )

    # ========================================================
    # CHECK ALL MOTORS
    # ========================================================

    def check_raw_positions(
        self,
        raw_positions,
    ):
        """
        Check all three motors against
        calibration limits.

        Returns:
            True if all positions are safe.
        """

        for motor_id in [1, 2, 3]:

            raw = raw_positions.get(
                motor_id
            )

            if raw is None:

                return False

            if not self.is_within_limit(
                motor_id,
                raw,
            ):

                return False

        return True

    # ========================================================
    # GET JOINT ANGLES
    # ========================================================

    def get_joint_angles(
        self,
        raw_positions,
    ):
        """
        Return joint angles in degrees.

        Returns:
            {
                "J1": angle_deg,
                "J2": angle_deg,
                "J3": angle_deg,
            }
        """

        q = self.raw_positions_to_q(
            raw_positions
        )

        return {
            "J1": math.degrees(q[0]),
            "J2": math.degrees(q[1]),
            "J3": math.degrees(q[2]),
        }

    # ========================================================
    # GRAVITY COMPENSATION
    # ========================================================

    def get_gravity_torque(
        self,
        raw_positions,
    ):
        """
        Calculate gravity torque from
        actual motor RAW positions.

        Returns:
            tau in Nm
        """

        if not self.check_raw_positions(
            raw_positions
        ):

            raise ValueError(
                "Motor position is outside "
                "calibrated limits."
            )

        q = self.raw_positions_to_q(
            raw_positions
        )

        return gravity_torque(q)

    # ========================================================
    # GET CURRENT STATE
    # ========================================================

    def get_state(
        self,
        raw_positions,
    ):
        """
        Calculate complete leader state.

        Returns:
            dictionary containing:
                q_rad
                q_deg
                end_effector
                gravity_torque
        """

        if not self.check_raw_positions(
            raw_positions
        ):

            raise ValueError(
                "Motor position is outside "
                "calibrated limits."
            )

        q = self.raw_positions_to_q(
            raw_positions
        )

        position = end_effector_position(
            q
        )

        tau = gravity_torque(q)

        return {
            "q_rad": q,
            "q_deg": np.degrees(q),
            "end_effector": position,
            "gravity_torque": tau,
        }


# ============================================================
# TELEOP
# ============================================================

class TeleopController:
    """
    Simple Cartesian teleoperation controller.
    """

    def __init__(self):

        self.master_start = None

        self.follower_start = np.zeros(3)

        self.scale = 1.0

    def start(
        self,
        master_position,
        follower_position,
    ):

        self.master_start = np.array(
            master_position,
            dtype=float,
        )

        self.follower_start = np.array(
            follower_position,
            dtype=float,
        )

    def update(
        self,
        master_position,
    ):

        if self.master_start is None:

            raise RuntimeError(
                "Teleop has not been started."
            )

        master_position = np.array(
            master_position,
            dtype=float,
        )

        delta = (
            master_position
            - self.master_start
        )

        target = (
            self.follower_start
            + self.scale * delta
        )

        return target


# ============================================================
# TEST
# ============================================================

def main():

    print()
    print("==========================================")
    print(" LEADER CONTROLLER TEST")
    print("==========================================")
    print()

    # --------------------------------------------------------
    # Create controller
    # --------------------------------------------------------

    try:

        controller = LeaderController()

    except RuntimeError as error:

        print(f"ERROR: {error}")

        return

    print(
        f"Calibration file:"
    )

    print(
        f"  {controller.calibration_file}"
    )

    print()

    print("Calibration loaded:")
    print()

    for joint, motor_id in MOTOR_IDS.items():

        calib = (
            controller.joint_calibration[
                joint
            ]
        )

        print(
            f"{joint} "
            f"(Motor ID {motor_id}):"
        )

        print(
            f"  ZERO = {calib.zero_raw}"
        )

        print(
            f"  DIR  = {calib.direction:+d}"
        )

        print(
            f"  MIN  = {calib.limit_min}"
        )

        print(
            f"  MAX  = {calib.limit_max}"
        )

        print()

    # --------------------------------------------------------
    # Example RAW positions
    #
    # IMPORTANT:
    # These are ONLY test values.
    #
    # In the real controller these values will come
    # directly from Feetech motors.
    # --------------------------------------------------------

    raw_positions = {
        1: controller.joint_calibration[
            "J1"
        ].zero_raw,

        2: controller.joint_calibration[
            "J2"
        ].zero_raw,

        3: controller.joint_calibration[
            "J3"
        ].zero_raw,
    }

    # --------------------------------------------------------
    # Convert RAW -> q
    # --------------------------------------------------------

    q = controller.raw_positions_to_q(
        raw_positions
    )

    print("Current joint angles:")

    print(
        f"  J1 = {math.degrees(q[0]):+.2f} deg"
    )

    print(
        f"  J2 = {math.degrees(q[1]):+.2f} deg"
    )

    print(
        f"  J3 = {math.degrees(q[2]):+.2f} deg"
    )

    print()

    # --------------------------------------------------------
    # FK
    # --------------------------------------------------------

    position = end_effector_position(q)

    print("End-effector:")

    print(
        f"  X = {position[0] * 1000:.2f} mm"
    )

    print(
        f"  Y = {position[1] * 1000:.2f} mm"
    )

    print(
        f"  Z = {position[2] * 1000:.2f} mm"
    )

    print()

    # --------------------------------------------------------
    # COM
    # --------------------------------------------------------

    coms = center_of_mass_positions(q)

    print("COM:")

    for i, com in enumerate(
        coms,
        start=2,
    ):

        print(
            f"  Link {i}: "
            f"X={com[0] * 1000:.2f} "
            f"Y={com[1] * 1000:.2f} "
            f"Z={com[2] * 1000:.2f} mm"
        )

    print()

    # --------------------------------------------------------
    # Gravity
    # --------------------------------------------------------

    tau = gravity_torque(q)

    print("Gravity torque:")

    print(
        f"  J1 = {tau[0]:+.4f} Nm"
    )

    print(
        f"  J2 = {tau[1]:+.4f} Nm"
    )

    print(
        f"  J3 = {tau[2]:+.4f} Nm"
    )

    print()

    # --------------------------------------------------------
    # Teleop test
    # --------------------------------------------------------

    teleop = TeleopController()

    follower_start = np.array([
        0.300,
        0.000,
        0.400,
    ])

    teleop.start(
        position,
        follower_start,
    )

    target = teleop.update(
        position
    )

    print("Follower target:")

    print(
        f"  X = {target[0]:.4f} m"
    )

    print(
        f"  Y = {target[1]:.4f} m"
    )

    print(
        f"  Z = {target[2]:.4f} m"
    )

    print()

    print("==========================================")
    print(" TEST COMPLETE")
    print("==========================================")


if __name__ == "__main__":
    main()
