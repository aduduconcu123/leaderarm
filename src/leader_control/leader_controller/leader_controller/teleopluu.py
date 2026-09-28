#!/usr/bin/env python3

import math
import struct
import time

import can

from feetech_driver.motor import STS3250
from leader_controller.controller import LeaderController


# ============================================================
# CONFIG
# ============================================================

PORT = "/dev/ttyUSB0"
BAUDRATE = 1000000

MOTOR_IDS = {
    "J1": 1,
    "J2": 2,
    "J3": 3,
}


# ============================================================
# GIM6010-8 CONFIG
# ============================================================

CAN_INTERFACE = "can0"
CAN_BITRATE = 500000

GIM_NODE_ID = 2


# ------------------------------------------------------------
# Teleoperation mapping
# ------------------------------------------------------------

# Feetech J1 angle:
#
#       q_feetech
#            ↓
#       direction
#            ↓
#       scale
#            ↓
#       offset
#            ↓
#       q_gim
#
# Initially:
#
#       GIM = Feetech
#
# 1.0  = same direction
# -1.0 = reverse direction

GIM_DIRECTION = 1.0
GIM_SCALE = 1.0

# Zero offset of GIM in radians
GIM_OFFSET = 0.0


# ------------------------------------------------------------
# Safety limits
# ------------------------------------------------------------

# GIM allowed position
#
# Change these later after calibration.

GIM_MIN_ANGLE_DEG = -90.0
GIM_MAX_ANGLE_DEG = 90.0


# Maximum change of target angle per loop
#
# This prevents a sudden jump if the Feetech gives
# an unexpected value.

MAX_STEP_DEG = 5.0


# Control loop
CONTROL_RATE = 50.0


# ------------------------------------------------------------
# Feed Forward
# ------------------------------------------------------------

GIM_VEL_FF = 0
GIM_TORQUE_FF = 0


# ============================================================
# GIM6010-8 CLASS
# ============================================================

class GIM6010:

    CMD_SET_AXIS_STATE = 0x007
    CMD_GET_ENCODER = 0x009
    CMD_SET_CONTROLLER_MODE = 0x00B
    CMD_SET_INPUT_POS = 0x00C

    AXIS_STATE_CLOSED_LOOP = 8

    def __init__(
        self,
        channel="can0",
        bitrate=500000,
        node_id=2,
    ):

        self.channel = channel
        self.bitrate = bitrate
        self.node_id = node_id

        self.bus = None

        self.current_target = 0.0

    # --------------------------------------------------------
    # CAN ID
    # --------------------------------------------------------

    def can_id(self, cmd_id):

        return (self.node_id << 5) + cmd_id

    # --------------------------------------------------------
    # Connect
    # --------------------------------------------------------

    def connect(self):

        print()
        print("------------------------------------------")
        print(" Connecting GIM6010-8")
        print("------------------------------------------")

        self.bus = can.Bus(
            interface="socketcan",
            channel=self.channel,
        )

        print(
            f"CAN interface : {self.channel}"
        )

        print(
            f"CAN bitrate   : {self.bitrate}"
        )

        print(
            f"GIM node ID   : {self.node_id}"
        )

        print("GIM CAN: OK")
        print()

    # --------------------------------------------------------
    # Send CAN frame
    # --------------------------------------------------------

    def send(self, arbitration_id, data):

        if self.bus is None:

            raise RuntimeError(
                "GIM CAN bus is not connected."
            )

        if len(data) != 8:

            raise ValueError(
                "GIM CAN frame must contain 8 bytes."
            )

        message = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=False,
        )

        self.bus.send(message)

    # --------------------------------------------------------
    # Set controller mode
    # --------------------------------------------------------

    def set_position_mode(self):

        can_id = self.can_id(
            self.CMD_SET_CONTROLLER_MODE
        )

        data = bytes([
            0x03, 0x00, 0x00, 0x00,
            0x03, 0x00, 0x00, 0x00,
        ])

        self.send(
            can_id,
            data,
        )

        print(
            f"GIM position mode "
            f"(CAN 0x{can_id:03X})"
        )

    # --------------------------------------------------------
    # Closed loop
    # --------------------------------------------------------

    def enable(self):

        can_id = self.can_id(
            self.CMD_SET_AXIS_STATE
        )

        data = struct.pack(
            "<I",
            self.AXIS_STATE_CLOSED_LOOP,
        ) + bytes(4)

        self.send(
            can_id,
            data,
        )

        print(
            f"GIM closed-loop "
            f"(CAN 0x{can_id:03X})"
        )

    # --------------------------------------------------------
    # Disable / idle
    # --------------------------------------------------------

    def disable(self):

        can_id = self.can_id(
            self.CMD_SET_AXIS_STATE
        )

        data = struct.pack(
            "<I",
            1,
        ) + bytes(4)

        try:

            self.send(
                can_id,
                data,
            )

        except Exception:

            pass

    # --------------------------------------------------------
    # Send position
    # --------------------------------------------------------

    def set_position(
        self,
        position_rev,
        velocity_ff=0,
        torque_ff=0,
    ):

        can_id = self.can_id(
            self.CMD_SET_INPUT_POS
        )

        data = struct.pack(
            "<fHH",
            float(position_rev),
            int(velocity_ff),
            int(torque_ff),
        )

        self.send(
            can_id,
            data,
        )

        self.current_target = position_rev

    # --------------------------------------------------------
    # Close
    # --------------------------------------------------------

    def close(self):

        try:

            self.disable()

        except Exception:

            pass

        if self.bus is not None:

            try:

                self.bus.shutdown()

            except Exception:

                pass

            self.bus = None


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def clamp(
    value,
    minimum,
    maximum,
):

    return max(
        minimum,
        min(
            value,
            maximum,
        ),
    )


def radians_to_revolutions(angle_rad):

    return angle_rad / (
        2.0 * math.pi
    )


def revolutions_to_degrees(rev):

    return rev * 360.0


# ============================================================
# MAIN
# ============================================================

def main():

    print()
    print("==========================================")
    print(" LEADER → GIM TELEOP TEST")
    print("==========================================")
    print()

    controller = None
    motors = {}
    gim = None

    try:

        # ====================================================
        # LEADER CONTROLLER
        # ====================================================

        controller = LeaderController()

        print("LeaderController initialized.")
        print()

        # ====================================================
        # FEETECH
        # ====================================================

        for joint, motor_id in MOTOR_IDS.items():

            print(
                f"Connecting {joint} "
                f"(Motor ID {motor_id})..."
            )

            motor = STS3250(
                port=PORT,
                baudrate=BAUDRATE,
                motor_id=motor_id,
            )

            if not motor.ping():

                raise RuntimeError(
                    f"{joint} "
                    f"(ID {motor_id}) "
                    f"does not respond."
                )

            motors[joint] = motor

            print(
                f"  {joint}: OK"
            )

        print()
        print("All Feetech motors connected.")
        print()

        # ====================================================
        # FEETECH TORQUE OFF
        # ====================================================

        for motor in motors.values():

            motor.disable_torque()

        print("Feetech torque: OFF")
        print()

        # ====================================================
        # GIM6010-8
        # ====================================================

        gim = GIM6010(
            channel=CAN_INTERFACE,
            bitrate=CAN_BITRATE,
            node_id=GIM_NODE_ID,
        )

        gim.connect()

        # ----------------------------------------------------
        # Configure GIM
        # ----------------------------------------------------

        gim.set_position_mode()

        time.sleep(0.1)

        gim.enable()

        time.sleep(0.2)

        print()
        print("GIM ready.")
        print()

        # ====================================================
        # INITIAL POSITION
        # ====================================================

        print(
            "Reading initial Feetech position..."
        )

        raw_positions = {}

        for joint, motor in motors.items():

            raw = motor.read_raw_position()

            if raw is None:

                raise RuntimeError(
                    f"Cannot read {joint}"
                )

            raw_positions[
                MOTOR_IDS[joint]
            ] = raw

        controller.check_raw_positions(
            raw_positions
        )

        state = controller.get_state(
            raw_positions
        )

        q = state["q"]

        initial_angle = q[0]

        initial_angle_deg = (
            initial_angle
            * 180.0
            / math.pi
        )

        print(
            f"Initial J1: "
            f"{initial_angle_deg:+.2f} deg"
        )

        # ----------------------------------------------------
        # Initial GIM target
        # ----------------------------------------------------

        target_angle = (
            initial_angle
            * GIM_DIRECTION
            * GIM_SCALE
            + GIM_OFFSET
        )

        target_angle = clamp(
            target_angle,
            math.radians(
                GIM_MIN_ANGLE_DEG
            ),
            math.radians(
                GIM_MAX_ANGLE_DEG
            ),
        )

        target_rev = (
            radians_to_revolutions(
                target_angle
            )
        )

        gim.set_position(
            target_rev,
            GIM_VEL_FF,
            GIM_TORQUE_FF,
        )

        print(
            f"Initial GIM target: "
            f"{math.degrees(target_angle):+.2f} deg"
        )

        print()
        print("------------------------------------------")
        print(" TELEOP ACTIVE")
        print("------------------------------------------")
        print()
        print(
            "Move Feetech J1 slowly."
        )
        print(
            "GIM should follow J1."
        )
        print()
        print(
            "Press Ctrl+C to stop."
        )
        print()

        # ====================================================
        # CONTROL LOOP
        # ====================================================

        period = 1.0 / CONTROL_RATE

        last_time = time.monotonic()

        while True:

            loop_start = time.monotonic()

            # =================================================
            # READ ALL FEETECH
            # =================================================

            raw_positions = {}

            for joint, motor in motors.items():

                raw = motor.read_raw_position()

                if raw is None:

                    raise RuntimeError(
                        f"Cannot read {joint}"
                    )

                raw_positions[
                    MOTOR_IDS[joint]
                ] = raw

            # =================================================
            # CONTROLLER
            # =================================================

            controller.check_raw_positions(
                raw_positions
            )

            state = controller.get_state(
                raw_positions
            )

            q = state["q"]

            # Gravity torque is calculated but NOT applied.
            tau = state["gravity_torque"]

            # =================================================
            # J1 → GIM
            # =================================================

            leader_angle = q[0]

            target_angle = (
                leader_angle
                * GIM_DIRECTION
                * GIM_SCALE
                + GIM_OFFSET
            )

            # =================================================
            # LIMIT
            # =================================================

            min_angle = math.radians(
                GIM_MIN_ANGLE_DEG
            )

            max_angle = math.radians(
                GIM_MAX_ANGLE_DEG
            )

            target_angle = clamp(
                target_angle,
                min_angle,
                max_angle,
            )

            # =================================================
            # RATE LIMIT
            # =================================================

            max_step = math.radians(
                MAX_STEP_DEG
            )

            current_angle = (
                gim.current_target
                * 2.0
                * math.pi
            )

            difference = (
                target_angle
                - current_angle
            )

            if difference > max_step:

                target_angle = (
                    current_angle
                    + max_step
                )

            elif difference < -max_step:

                target_angle = (
                    current_angle
                    - max_step
                )

            # =================================================
            # RAD → REV
            # =================================================

            target_rev = (
                radians_to_revolutions(
                    target_angle
                )
            )

            # =================================================
            # SEND GIM POSITION
            # =================================================

            gim.set_position(
                target_rev,
                GIM_VEL_FF,
                GIM_TORQUE_FF,
            )

            # =================================================
            # PRINT
            # =================================================

            j1_deg = math.degrees(
                leader_angle
            )

            gim_deg = math.degrees(
                target_angle
            )

            print(
                "\r"
                f"J1 "
                f"{j1_deg:+7.2f}°  →  "
                f"GIM "
                f"{gim_deg:+7.2f}°   |   "
                f"RAW {raw_positions[1]:4d}   |   "
                f"τ "
                f"{tau[0]:+.3f} Nm",
                end="",
                flush=True,
            )

            # =================================================
            # LOOP RATE
            # =================================================

            elapsed = (
                time.monotonic()
                - loop_start
            )

            sleep_time = (
                period
                - elapsed
            )

            if sleep_time > 0:

                time.sleep(
                    sleep_time
                )

    except KeyboardInterrupt:

        print()
        print()
        print("Stopping teleop...")

    except Exception as e:

        print()
        print()
        print(
            f"ERROR: {e}"
        )

    finally:

        # ====================================================
        # SAFETY
        # ====================================================

        print()
        print(
            "Disabling GIM..."
        )

        if gim is not None:

            try:

                gim.disable()

            except Exception:

                pass

            try:

                gim.close()

            except Exception:

                pass

        # ====================================================
        # FEETECH
        # ====================================================

        for motor in motors.values():

            try:

                motor.disable_torque()

            except Exception:

                pass

            try:

                motor.close()

            except Exception:

                pass

        print(
            "Feetech torque: OFF"
        )

        print(
            "GIM: disabled"
        )

        print(
            "Motors closed."
        )

        print()


if __name__ == "__main__":

    main()