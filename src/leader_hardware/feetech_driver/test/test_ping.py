#!/usr/bin/env python3

import serial

PORT = "/dev/ttyUSB0"
BAUDRATE = 1000000
MOTOR_ID = 3


def calculate_checksum(motor_id, length, instruction):
    return (~(
        motor_id +
        length +
        instruction
    )) & 0xFF


def main():
    print("=" * 50)
    print("STS3250 PING TEST")
    print("=" * 50)

    ser = serial.Serial(
        PORT,
        BAUDRATE,
        timeout=0.2
    )

    length = 2
    instruction = 0x01

    checksum = calculate_checksum(
        MOTOR_ID,
        length,
        instruction
    )

    packet = bytes([
        0xFF,
        0xFF,
        MOTOR_ID,
        length,
        instruction,
        checksum
    ])

    print(f"Motor ID : {MOTOR_ID}")
    print(f"TX       : {packet.hex(' ')}")

    ser.write(packet)

    response = ser.read(20)

    if response:
        print(f"RX       : {response.hex(' ')}")
        print("RESULT   : PASS")
    else:
        print("RX       : NO RESPONSE")
        print("RESULT   : FAIL")

    ser.close()


if __name__ == "__main__":
    main()
