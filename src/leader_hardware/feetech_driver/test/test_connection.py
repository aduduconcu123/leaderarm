#!/usr/bin/env python3

import serial

PORT = "/dev/ttyUSB0"
BAUDRATE = 1000000


def main():
    print("=" * 50)
    print("FEETECH CONNECTION TEST")
    print("=" * 50)

    try:
        ser = serial.Serial(
            PORT,
            BAUDRATE,
            timeout=0.2
        )

        print(f"Port     : {ser.port}")
        print(f"Baudrate : {ser.baudrate}")
        print(f"Opened   : {ser.is_open}")

        ser.close()

        print("RESULT   : PASS")

    except Exception as e:
        print("RESULT   : FAIL")
        print(f"ERROR    : {e}")


if __name__ == "__main__":
    main()
