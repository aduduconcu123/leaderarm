#!/usr/bin/env python3

import serial
import time

PORT = "/dev/ttyUSB0"
BAUDRATE = 1000000
MOTOR_ID = 3

TORQUE_ENABLE = 40
GOAL_POSITION = 42
PRESENT_POSITION = 56


def checksum(values):
    return (~sum(values)) & 0xFF


def write_register(ser, address, data):
    length = 3 + len(data)
    instruction = 0x03

    packet_data = [
        MOTOR_ID,
        length,
        instruction,
        address,
        *data
    ]

    packet = bytes([
        0xFF,
        0xFF,
        *packet_data,
        checksum(packet_data)
    ])

    print(f"TX WRITE: {packet.hex(' ')}")

    ser.write(packet)
    response = ser.read(20)

    print(
        "RX WRITE:",
        response.hex(" ") if response else "NO RESPONSE"
    )

    return response


def read_register(ser, address, size):
    length = 4
    instruction = 0x02

    packet_data = [
        MOTOR_ID,
        length,
        instruction,
        address,
        size
    ]

    packet = bytes([
        0xFF,
        0xFF,
        *packet_data,
        checksum(packet_data)
    ])

    print(f"TX READ : {packet.hex(' ')}")

    ser.write(packet)
    response = ser.read(20)

    print(
        "RX READ :",
        response.hex(" ") if response else "NO RESPONSE"
    )

    if len(response) < 6 + size:
        return None

    value = 0

    for i in range(size):
        value |= response[5 + i] << (8 * i)

    return value


def main():

    print("=" * 55)
    print("STS3250 POSITION MOVE TEST")
    print("=" * 55)

    ser = serial.Serial(
        PORT,
        BAUDRATE,
        timeout=0.3
    )

    # --------------------------------------------------
    # Read current position
    # --------------------------------------------------

    current_position = read_register(
        ser,
        PRESENT_POSITION,
        2
    )

    if current_position is None:
        print("ERROR: Cannot read current position.")
        ser.close()
        return

    print(f"\nCurrent position: {current_position}")

    # --------------------------------------------------
    # Read torque state
    # --------------------------------------------------

    torque = read_register(
        ser,
        TORQUE_ENABLE,
        1
    )

    if torque is None:
        print("ERROR: Cannot read torque state.")
        ser.close()
        return

    print(f"Torque state: {torque}")

    # --------------------------------------------------
    # Calculate small movement
    # --------------------------------------------------

    delta = 50
    target_position = current_position + delta

    if target_position > 4095:
        target_position = current_position - delta

    print(f"Target position: {target_position}")
    print(
        f"Movement: "
        f"{abs(target_position - current_position)} counts "
        f"(~{abs(target_position - current_position) * 360 / 4096:.2f} deg)"
    )

    print()
    input("Press ENTER to start the movement...")

    # --------------------------------------------------
    # Enable torque if necessary
    # --------------------------------------------------

    if torque == 0:
        print("\nEnabling torque...")
        write_register(
            ser,
            TORQUE_ENABLE,
            [1]
        )
        time.sleep(0.2)

    # --------------------------------------------------
    # Move to target
    # --------------------------------------------------

    print("\nMoving to target...")

    goal_low = target_position & 0xFF
    goal_high = (target_position >> 8) & 0xFF

    write_register(
        ser,
        GOAL_POSITION,
        [goal_low, goal_high]
    )

    time.sleep(1.0)

    position_after_move = read_register(
        ser,
        PRESENT_POSITION,
        2
    )

    print(
        f"\nPosition after move: "
        f"{position_after_move}"
    )

    # --------------------------------------------------
    # Return to original position
    # --------------------------------------------------

    print("\nReturning to original position...")

    original_low = current_position & 0xFF
    original_high = (current_position >> 8) & 0xFF

    write_register(
        ser,
        GOAL_POSITION,
        [original_low, original_high]
    )

    time.sleep(1.0)

    final_position = read_register(
        ser,
        PRESENT_POSITION,
        2
    )

    print(
        f"\nFinal position: "
        f"{final_position}"
    )

    # --------------------------------------------------

    if position_after_move is not None:
        print("\nMovement command sent successfully.")

    if final_position is not None:
        error = abs(final_position - current_position)

        print(
            f"Return error: "
            f"{error} counts "
            f"({error * 360 / 4096:.2f} deg)"
        )

    ser.close()

    print("\nTEST FINISHED")


if __name__ == "__main__":
    main()
