#!/usr/bin/env/python3
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=0x5F)

pca.frequency = 50

SERVO_RANGES = {
    0: (-45, 50),
    1: (-90, 90),
    2: (-90, 90),
}

def set_angle(servo_id, angle):
    if servo_id not in servos:
        raise ValueError("Servo: 0,1,2")
    else:
        s = servo.Servo(pca.channels[servo_id], min_pulse=500, max_pulse=2400, actuation_range=180)
        s.angle = angle


def to_servo_angle(user_angle):
    if user_angle < -90 or user_angle > 90:
        raise ValueError("Angle: -90..90")
    return user_angle + 90

if __name__ == "__main__":
    servos = [0, 1, 2]
    print("Commande: <servo> <angle> (ex: 1 30), quit")
    print("Plage angle servo 0: -45..50")
    print("Plage angle servos 1/2: -90..90")

    while True:
        cmd = input("> ").strip().lower()
        if cmd in {"quit", "q", "exit"}:
            break

        parts = cmd.split()
        if len(parts) != 2:
            print("Ex: 1 30")
            continue

        try:
            servo_id = int(parts[0])
            user_angle = float(parts[1])
            if servo_id not in servos:
                raise ValueError("Servo: 0,1,2")
            min_angle, max_angle = SERVO_RANGES[servo_id]
            if user_angle < min_angle or user_angle > max_angle:
                raise ValueError(f"Angle servo {servo_id}: {min_angle}..{max_angle}")
            set_angle(servo_id, to_servo_angle(user_angle))
            print(f"OK servo {servo_id} -> {user_angle}")
        except ValueError as e:
            print(e)