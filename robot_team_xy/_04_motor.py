#!/usr/bin/env python3
# coding: utf-8
# Tache 4 : Moteur DC (Simplifié)

import time
from adafruit_motor import motor
from _03_servo import pca, set_angle, to_servo_angle

# Broches des moteurs
M1_IN1 = 15
M1_IN2 = 14
M2_IN1 = 12
M2_IN2 = 13

SERVO_DIR_CH = 0
CENTER_ANGLE = 0 # Angle utilisateur défini par défaut (0 = centre exact)

motor1 = None
motor2 = None

def setup():
    global motor1, motor2
    
    # La carte PCA est déjà initialisée via l'import de _03_servo
    motor1 = motor.DCMotor(pca.channels[M1_IN1], pca.channels[M1_IN2])
    motor1.decay_mode = motor.SLOW_DECAY

    motor2 = motor.DCMotor(pca.channels[M2_IN1], pca.channels[M2_IN2])
    motor2.decay_mode = motor.SLOW_DECAY

    # Placement du servo au centre avec les fonctions de _03_servo
    set_angle(SERVO_DIR_CH, to_servo_angle(CENTER_ANGLE))

def stop():
    motor1.throttle = 0
    motor2.throttle = 0

def drive(speed_pct, direction):
    if direction == 0:
        stop()
        return
    
    speed_pct = max(0, min(100, speed_pct))
    throttle = (speed_pct / 100.0) * direction
    
    motor1.throttle = throttle
    motor2.throttle = -throttle

def drive_ramp(speed_pct, direction, ramp_time=1.0):
    steps = 50
    delay = ramp_time / steps
    for i in range(1, steps + 1):
        drive(speed_pct * i / steps, direction)
        time.sleep(delay)

def destroy():
    stop()
    # pca.deinit() # Optionnel si on souhaite tout désactiver à la fin

if __name__ == "__main__":
    setup()
    print("=== Tache 4 - Moteur DC ===")
    print("  f  : avant 25%")
    print("  b  : arriere 25%")
    print("  s  : stop")
    print("  rf : rampe avant (1s)")
    print("  rb : rampe arriere (1s)")
    print("  q  : quitter")

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
                drive_ramp(100, 1)
                print("-> Rampe avant terminée")
            elif cmd == "rb":
                drive_ramp(100, -1)
                print("-> Rampe arriere terminée")
            elif cmd == "q":
                break
            else:
                print("Commande inconnue")

    except KeyboardInterrupt:
        pass
    finally:
        destroy()
        print("Programme terminé.")