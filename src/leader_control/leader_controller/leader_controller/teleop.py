import math
import struct
import time

import can

from feetech_driver.motor import STS3250



PORT = "/dev/ttyUSB0"
BAUDRATE = 1000000

FEETECH_ID = 3



CAN_INTERFACE = "can0"
CAN_BITRATE = 500000

GIM_NODE_ID = 2




GIM_DIRECTION = 1.0

GIM_SCALE = 8.0

GIM_OFFSET = 0.0



GIM_MIN_ANGLE_DEG = -90.0
GIM_MAX_ANGLE_DEG = 90.0

MAX_STEP_DEG = 40.0

CONTROL_RATE = 50.0



class GIM6010:

    CMD_SET_AXIS_STATE = 0x007
    CMD_SET_INPUT_POS = 0x00C
    CMD_SET_CONTROLLER_MODE = 0x00B

    AXIS_STATE_IDLE = 1
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


    def can_id(self, cmd_id):
        return (self.node_id << 5) + cmd_id


    def connect(self):

        print()
        print("------------------------------------------")
        print(" Connecting GIM6010-8")
        print("------------------------------------------")

        self.bus = can.Bus(
            interface="socketcan",
            channel=self.channel,
        )

        print(f"CAN interface : {self.channel}")
        print(f"CAN bitrate   : {self.bitrate}")
        print(f"GIM node ID   : {self.node_id}")
        print("GIM CAN: OK")
        print()


    def send(self, arbitration_id, data):

        if self.bus is None:
            raise RuntimeError("GIM CAN bus is not connected.")

        if len(data) != 8:
            raise ValueError("CAN frame must contain 8 bytes.")

        msg = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=False,
        )

        self.bus.send(msg)


    def set_position_mode(self):

        can_id = self.can_id(
            self.CMD_SET_CONTROLLER_MODE
        )

        data = bytes([
            0x03, 0x00, 0x00, 0x00,
            0x03, 0x00, 0x00, 0x00,
        ])

        self.send(can_id, data)

        print(
            f"GIM position mode "
            f"(CAN 0x{can_id:03X})"
        )


    def enable(self):

        can_id = self.can_id(
            self.CMD_SET_AXIS_STATE
        )

        data = struct.pack(
            "<I",
            self.AXIS_STATE_CLOSED_LOOP,
        ) + bytes(4)

        self.send(can_id, data)

        print(
            f"GIM closed-loop "
            f"(CAN 0x{can_id:03X})"
        )


    def disable(self):

        can_id = self.can_id(
            self.CMD_SET_AXIS_STATE
        )

        data = struct.pack(
            "<I",
            self.AXIS_STATE_IDLE,
        ) + bytes(4)

        try:
            self.send(can_id, data)
        except Exception:
            pass


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

        self.send(can_id, data)

        self.current_target = position_rev


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



def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def raw_to_degrees(raw):

    return raw * 360.0 / 4095.0


def degrees_to_radians(deg):
    return math.radians(deg)


def radians_to_revolutions(rad):
    return rad / (2.0 * math.pi)



def main():

    print()
    print("==========================================")
    print(" FEETECH J1 → GIM6010-8 TELEOP TEST")
    print("==========================================")
    print()

    motor = None
    gim = None

    try:


        print(
            f"Connecting Feetech J1 "
            f"(ID {FEETECH_ID})..."
        )

        motor = STS3250(
            port=PORT,
            baudrate=BAUDRATE,
            motor_id=FEETECH_ID,
        )

        if not motor.ping():
            raise RuntimeError(
                "Feetech J1 does not respond."
            )

        print("Feetech J1: OK")

        motor.disable_torque()

        print("Feetech torque: OFF")
        print()


        gim = GIM6010(
            channel=CAN_INTERFACE,
            bitrate=CAN_BITRATE,
            node_id=GIM_NODE_ID,
        )

        gim.connect()

        gim.set_position_mode()

        time.sleep(0.1)

        gim.enable()

        time.sleep(0.2)

        print()
        print("GIM ready.")
        print()


        raw = motor.read_raw_position()

        if raw is None:
            raise RuntimeError(
                "Cannot read Feetech J1 position."
            )

        initial_deg = raw_to_degrees(raw)

        print(
            f"Initial RAW : {raw}"
        )

        print(
            f"Initial J1  : {initial_deg:.2f} deg"
        )


        zero_raw = raw
        previous_raw = raw
        accumulated_raw = 0

        print()
        print(
            "Current Feetech position = GIM zero."
        )
        print(
            "Move J1 slowly."
        )
        print()


        target_angle = 0.0

        gim.set_position(
            radians_to_revolutions(
                target_angle
            )
        )

        print("------------------------------------------")
        print(" TELEOP ACTIVE")
        print("------------------------------------------")
        print()
        print("J1 → GIM link : 1:1")
        print("Link range: -90° ... +90°")
        print("Press Ctrl+C to stop.")
        print()


        period = 1.0 / CONTROL_RATE

        while True:

            loop_start = time.monotonic()


            raw = motor.read_raw_position()

            if raw is None:
                raise RuntimeError(
                    "Cannot read Feetech J1."
                )


            delta_raw = raw - previous_raw

            if delta_raw > 2048:
                delta_raw -= 4096
            elif delta_raw < -2048:
                delta_raw += 4096

            accumulated_raw += delta_raw
            previous_raw = raw

            leader_deg = (
                accumulated_raw
                * 360.0
                / 4095.0
            )

            output_deg = leader_deg * GIM_DIRECTION

            output_deg = clamp(
                output_deg,
                GIM_MIN_ANGLE_DEG,
                GIM_MAX_ANGLE_DEG,
            )

            target_deg = output_deg * GIM_SCALE

            current_deg = gim.current_target * 360.0
            difference = target_deg - current_deg

            if difference > MAX_STEP_DEG:
                target_deg = current_deg + MAX_STEP_DEG
            elif difference < -MAX_STEP_DEG:
                target_deg = current_deg - MAX_STEP_DEG

            target_rad = degrees_to_radians(
                target_deg
            )

            target_rev = (
                radians_to_revolutions(
                    target_rad
                )
            )

            gim.set_position(
                target_rev,
                0,
                0,
            )


            print(
                "\r"
                f"RAW {raw:4d} | "
                f"J1 {leader_deg:+7.2f}° → "
                f"LINK {output_deg:+7.2f}° → "
                f"GIM motor {target_deg:+7.2f}°",
                end="",
                flush=True,
            )


            elapsed = (
                time.monotonic()
                - loop_start
            )

            sleep_time = (
                period
                - elapsed
            )

            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:

        print()
        print()
        print("Stopping teleop...")

    except Exception as e:

        print()
        print()
        print(f"ERROR: {e}")

    finally:

        print()
        print("Disabling GIM...")

        if gim is not None:

            try:
                gim.disable()
            except Exception:
                pass

            try:
                gim.close()
            except Exception:
                pass

        if motor is not None:

            try:
                motor.disable_torque()
            except Exception:
                pass

            try:
                motor.close()
            except Exception:
                pass

        print("Feetech torque: OFF")
        print("GIM: disabled")
        print("Motors closed.")
        print()


if __name__ == "__main__":
    main()
