#!/usr/bin/env python3
# Tâche 4 : Moteur DC
# Fonctions : drive(), drive_ramp(), calibrate_servo() + commande manuelle

import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor, servo

# --- Canaux PCA9685 (même config que move.py) ---
M1_IN1 = 15
M1_IN2 = 14
M2_IN1 = 12
M2_IN2 = 13

SERVO_DIR_CH = 0  # servo de direction roues avant

pwm    = None
motor1 = None
motor2 = None
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
    servo_dir.angle = 90  # position neutre (tout droit)


def stop():
    motor1.throttle = 0
    motor2.throttle = 0


def drive(speed_pct, direction):
    """
    Pilote le moteur à vitesse et sens donnés.
    speed_pct : 0-100 (%)
    direction  : 1=avant, -1=arrière, 0=stop
    """
    if direction == 0:
        stop()
        return
    speed_pct = max(0, min(100, speed_pct))
    throttle = (speed_pct / 100.0) * direction
    motor1.throttle =  throttle   # M1 et M2 en sens opposés
    motor2.throttle = -throttle   # car montés en miroir sur le même axle


def drive_ramp(speed_pct, direction, ramp_time=1.0):
    """
    Monte progressivement de 0 à speed_pct sur ramp_time secondes,
    puis maintient la vitesse cible.
    speed_pct : 0-100 (%)
    direction  : 1=avant, -1=arrière
    ramp_time  : durée de la rampe en secondes
    """
    steps = 50
    delay = ramp_time / steps
    for i in range(1, steps + 1):
        drive(speed_pct * i / steps, direction)
        time.sleep(delay)


def calibrate_servo():
    """
    Étalonnage interactif du servo de direction.
    Permet de trouver le centre (tout droit) et les butées gauche/droite.
    """
    angle = 90
    servo_dir.angle = angle
    print("\n=== Étalonnage servo de direction ===")
    print("  +  : +5°    -  : -5°")
    print("  ++ : +1°    -- : -1°")
    print("  <nombre> : aller directement à cet angle (0-180)")
    print("  q  : quitter et afficher le centre retenu")

    center = 90
    while True:
        cmd = input(f"  angle={angle}° > ").strip()
        if cmd == 'q':
            break
        elif cmd == '+':
            angle = min(180, angle + 5)
        elif cmd == '-':
            angle = max(0,   angle - 5)
        elif cmd == '++':
            angle = min(180, angle + 1)
        elif cmd == '--':
            angle = max(0,   angle - 1)
        elif cmd == 'c':
            center = angle
            print(f"  → Centre enregistré à {center}°")
            continue
        elif cmd.lstrip('-').isdigit():
            angle = max(0, min(180, int(cmd)))
        else:
            print("  Commande inconnue")
            continue
        servo_dir.angle = angle

    print(f"  Centre retenu : {center}° | Position finale : {angle}°")
    return center


def destroy():
    stop()
    pwm.deinit()


# ─────────────────────────────────────────────
if __name__ == '__main__':
    setup()
    print("=== Tâche 4 — Moteur DC ===")
    print("  f        : avant  25%")
    print("  b        : arrière 25%")
    print("  s        : stop")
    print("  rf       : rampe avant  (0→100% en 1s)")
    print("  rb       : rampe arrière(0→100% en 1s)")
    print("  d <v> <d> <t> : drive_ramp(vitesse, sens +1/-1, rampe_s)")
    print("  c        : étalonnage servo de direction")
    print("  q        : quitter")

    try:
        while True:
            cmd = input("\n> ").strip().lower()

            if cmd == 'f':
                drive(25, 1)
                print("→ Avant 25%")

            elif cmd == 'b':
                drive(25, -1)
                print("→ Arrière 25%")

            elif cmd == 's':
                stop()
                print("→ Stop")

            elif cmd == 'rf':
                print("→ Rampe avant 0→100% en 1s")
                drive_ramp(100, 1, ramp_time=1.0)
                print("  Rampe terminée — moteur en marche")

            elif cmd == 'rb':
                print("→ Rampe arrière 0→100% en 1s")
                drive_ramp(100, -1, ramp_time=1.0)
                print("  Rampe terminée — moteur en marche")

            elif cmd.startswith('d '):
                parts = cmd.split()
                if len(parts) == 4:
                    v, d, t = int(parts[1]), int(parts[2]), float(parts[3])
                    print(f"→ drive_ramp({v}%, {'avant' if d==1 else 'arrière'}, rampe={t}s)")
                    drive_ramp(v, d, ramp_time=t)
                else:
                    print("Usage : d <vitesse 0-100> <sens 1/-1> <rampe_secondes>")

            elif cmd == 'c':
                calibrate_servo()

            elif cmd == 'q':
                break

            else:
                print("Commande inconnue")

    except KeyboardInterrupt:
        pass
    finally:
        destroy()
        print("Programme terminé.")
