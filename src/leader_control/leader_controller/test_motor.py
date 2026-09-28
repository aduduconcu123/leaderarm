from feetech_driver.motor import STS3250
import time


motor = STS3250(
    port="/dev/ttyUSB0",
    baudrate=1000000,
    motor_id=3,
)

print("=== FEETECH STS3250 TEST ===")

try:
    print("Ping:", motor.ping())
    print("Torque enabled:", motor.is_torque_enabled())

    while True:
        print(
            f"position={motor.read_position():.2f}, "
            f"velocity={motor.read_velocity():.2f}, "
            f"load={motor.read_load():.2f}, "
            f"current={motor.read_current():.2f}, "
            f"temperature={motor.read_temperature():.2f}, "
            f"voltage={motor.read_voltage():.2f}"
        )

        time.sleep(0.2)

except KeyboardInterrupt:
    print("\nStopped.")

finally:
    motor.close()
