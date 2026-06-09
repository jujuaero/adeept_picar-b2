#!/usr/bin/env python3
# coding: utf-8
# Tache 4 : Moteur DC

import time
import os
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor, servo

M1_IN1 = 15
M1_IN2 = 14
M2_IN1 = 12
M2_IN2 = 13

SERVO_DIR_CH = 0
CENTER_ANGLE = 97.5

pwm       = None
motor1    = None
motor2    = None
servo_dir = None


def setup():
    global pwm, motor1, motor2, servo_dir
    i2c = busio.I2C(SCL, SDA)
    pwm = PCA9685(i2c, address=0x5f)
    pwm.frequency = 50

    motor1 = motor.DCMotor(pwm.channels[M1_IN1], pwm.channels[M1_IN2])
    motor1.decay_mode = motor.SLOW_DECAY

    motor2 = motor.DCMotor(pwm.channels[M2_IN1], pwm.channels[M2_IN2])
    motor2.decay_mode = motor.SLOW_DECAY

    servo_dir = servo.Servo(
        pwm.channels[SERVO_DIR_CH],
        min_pulse=500, max_pulse=2400, actuation_range=180
    )
    servo_dir.angle = CENTER_ANGLE


def stop():
    motor1.throttle = 0
    motor2.throttle = 0


def drive(speed_pct, direction):
    if direction == 0:
        stop()
        return
    speed_pct = max(0, min(100, speed_pct))
    throttle = (speed_pct / 100.0) * direction
    motor1.throttle =  throttle
    motor2.throttle = -throttle


def drive_ramp(speed_pct, direction, ramp_time=1.0):
    steps = 50
    delay = ramp_time / steps
    for i in range(1, steps + 1):
        drive(speed_pct * i / steps, direction)
        time.sleep(delay)


def _save_center(angle):
    filepath = os.path.abspath(__file__)
    with open(filepath, "r") as f:
        lines = f.readlines()
    for i, line in enumerate(lines):
        if line.startswith("CENTER_ANGLE"):
            lines[i] = "CENTER_ANGLE = " + str(angle) + "\n"
            break
    with open(filepath, "w") as f:
        f.writelines(lines)
    print("  -> CENTER_ANGLE = " + str(angle) + " ecrit dans 04_motor.py")


def calibrate_servo():
    angle = CENTER_ANGLE
    servo_dir.angle = angle
    print("")
    print("=== Etalonnage servo de direction ===")
    print("  +  : +5 deg    -  : -5 deg")
    print("  ++ : +1 deg    -- : -1 deg")
    print("  c  : enregistrer comme centre")
    print("  q  : quitter")

    center = CENTER_ANGLE
    while True:
        cmd = input("  angle=" + str(angle) + " > ").strip()
        if cmd == "q":
            break
        elif cmd == "+":
            angle = min(180, angle + 5)
        elif cmd == "-":
            angle = max(0, angle - 5)
        elif cmd == "++":
            angle = min(180, angle + 1)
        elif cmd == "--":
            angle = max(0, angle - 1)
        elif cmd == "c":
            center = angle
            _save_center(center)
            continue
        elif cmd.lstrip("-").isdigit():
            angle = max(0, min(180, int(cmd)))
        else:
            print("  Commande inconnue")
            continue
        servo_dir.angle = angle

    print("Centre=" + str(center) + "  Position finale=" + str(angle))
    return center


def destroy():
    stop()
    pwm.deinit()


if __name__ == "__main__":
    setup()
    print("=== Tache 4 - Moteur DC ===")
    print("  f             : avant 25%")
    print("  b             : arriere 25%")
    print("  s             : stop")
    print("  rf            : rampe avant  (0->100% en 1s)")
    print("  rb            : rampe arriere(0->100% en 1s)")
    print("  d <v> <d> <t> : drive_ramp(vitesse 0-100, sens +1/-1, rampe s)")
    print("  c             : etalonnage servo direction")
    print("  q             : quitter")

    try:
        while True:
            cmd = input("\n> ").strip().lower()

            if cmd == "f":
                drive(25, 1)
                print("-> Avant 25%")

            elif cmd == "b":
                drive(25, -1)
                print("-> Arriere 25%")

            elif cmd == "s":
                stop()
                print("-> Stop")

            elif cmd == "rf":
                print("-> Rampe avant 0->100% en 1s")
                drive_ramp(100, 1, ramp_time=1.0)
                print("   Rampe terminee")

            elif cmd == "rb":
                print("-> Rampe arriere 0->100% en 1s")
                drive_ramp(100, -1, ramp_time=1.0)
                print("   Rampe terminee")

            elif cmd.startswith("d "):
                parts = cmd.split()
                if len(parts) == 4:
                    v = int(parts[1])
                    d = int(parts[2])
                    t = float(parts[3])
                    sens = "avant" if d == 1 else "arriere"
                    print("-> drive_ramp(" + str(v) + "%, " + sens + ", " + str(t) + "s)")
                    drive_ramp(v, d, ramp_time=t)
                else:
                    print("Usage : d <vitesse 0-100> <sens 1/-1> <rampe_secondes>")

            elif cmd == "c":
                calibrate_servo()

            elif cmd == "q":
                break

            else:
                print("Commande inconnue")

    except KeyboardInterrupt:
        pass
    finally:
        destroy()
        print("Programme termine.")
