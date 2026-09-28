#!/usr/bin/env python3

import serial

PORT = "/dev/ttyUSB0"
BAUDRATE = 1000000
MOTOR_ID = 3

PRESENT_POSITION = 56


def calculate_checksum(motor_id, length, instruction, address, data_length):
    return (~(
        motor_id +
        length +
        instruction +
        address +
        data_length
    )) & 0xFF


def main():
    print("=" * 50)
    print("STS3250 READ POSITION TEST")
    print("=" * 50)

    ser = serial.Serial(
        PORT,
        BAUDRATE,
        timeout=0.2
    )

    instruction = 0x02
    data_length = 2
    length = 4

    checksum = calculate_checksum(
        MOTOR_ID,
        length,
        instruction,
        PRESENT_POSITION,
        data_length
    )

    packet = bytes([
        0xFF,
        0xFF,
        MOTOR_ID,
        length,
        instruction,
        PRESENT_POSITION,
        data_length,
        checksum
    ])

    print(f"Motor ID : {MOTOR_ID}")
    print(f"TX       : {packet.hex(' ')}")

    ser.write(packet)

    response = ser.read(20)

    print(
        f"RX       : "
        f"{response.hex(' ') if response else 'NO RESPONSE'}"
    )

    if len(response) >= 8:

        position = (
            response[5] |
            (response[6] << 8)
        )

        print(f"Raw position : {position}")
        print("RESULT       : PASS")

    else:
        print("RESULT       : FAIL")

    ser.close()


if __name__ == "__main__":
    main()
