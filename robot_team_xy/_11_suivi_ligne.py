#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Tache 11 : Suivi de ligne

import time
import sys
import select
from gpiozero import DistanceSensor, InputDevice
from threading import Thread

from _04_motor import *
from _03_servo import *
from _11_lineTracking import *

channel = 0
# --- Capteur ultrason et Obstacles ---
sensor = DistanceSensor(echo=24, trigger=23, max_distance=2, queue_len=1)
OBSTACLE_DIST = 20    # cm
SPEED = 35            # % de vitesse de base

# --- Capteurs de ligne ---
# Correspondances des broches GPIO pour Adeept PiCar-B
# (Généralement croisés dans l'exemple de base)
PIN_LEFT = 17
PIN_MIDDLE = 27
PIN_RIGHT = 22

left_sensor = InputDevice(pin=PIN_LEFT)
middle_sensor = InputDevice(pin=PIN_MIDDLE)
right_sensor = InputDevice(pin=PIN_RIGHT)

# --- Etats ---
STOPPED = 0
RUNNING = 1
state = STOPPED

# Dernière direction pour pouvoir retrouver la ligne
last_steer = CENTER_ANGLE

def checkdist():
    d = sensor.distance
    if d is None:
        return 999
    return d * 100

def follow_line():
    global last_steer
    # Lecture des capteurs : 
    # Pour ce type de module infrarouge CNY70 / TCRT5000 :
    # Noir (ligne) = 1 (ou True)
    # Blanc (sol) = 0 (ou False)
    # Parfois c'est l'inverse selon le calibrage, on suppose ici Noir=1
    # NB: left_sensor.value renvoie 1 ou 0
    l = left_sensor.value
    m = middle_sensor.value
    r = right_sensor.value

    # Angles calculés autour du centre (par ex: 6°)
    angle = CENTER_ANGLE

    if l == 0 and m == 1 and r == 0:
        # Centré
        angle = CENTER_ANGLE
    elif l == 1 and m == 0 and r == 0:
        # Déviation à gauche
        angle = CENTER_ANGLE - 40
    elif l == 1 and m == 1 and r == 0:
        # Légère déviation à gauche
        angle = CENTER_ANGLE - 25
    elif l == 0 and m == 0 and r == 1:
        # Déviation à droite
        angle = CENTER_ANGLE + 40
    elif l == 0 and m == 1 and r == 1:
        # Légère déviation à droite
        angle = CENTER_ANGLE + 25
    elif l == 1 and m == 1 and r == 1:
        # Intersection droite -> on avance tout droit
        angle = CENTER_ANGLE
    elif l == 0 and m == 0 and r == 0:
        # Ligne perdue -> on maintient le dernier braquage prononcé
        angle = last_steer
    
    last_steer = angle
    set_angle(channel, angle)
    drive(SPEED, 1)

def stop_robot(reason="manuel"):
    global state
    stop()
    set_angle(channel, CENTER_ANGLE)
    state = STOPPED
    print("-> Arret (%s)" % reason)

def start_move():
    global state
    state = RUNNING
    set_angle(channel, CENTER_ANGLE)
    print("-> Suivi de ligne demarre")

def read_cmd():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip()
    return None

def destroy():
    stop()
    if hasattr(motor_drv, 'pwm') and pwm:
        pwm.deinit()
    sensor.close()

if __name__ == "__main__":
    setup()
    
    print("=== Tache 11 - Suivi de ligne ===")
    print("  M : demarrer le suivi")
    print("  A : arret immediat")
    print("  Ctrl-C : quitter")
    print()

    try:
        while True:
            cmd = read_cmd()

            if cmd in ("M", "m"):
                if state == STOPPED:
                    start_move()

            elif cmd in ("A", "a"):
                if state != STOPPED:
                    stop_robot(reason="manuel")

            if state == RUNNING:
                dist = checkdist()

                if dist < OBSTACLE_DIST:
                    stop()
                    if state == RUNNING:
                        print("-> STOP  : obstacle a %.1f cm" % dist)
                        state = STOPPED
                else:
                    follow_line()

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nFin de programme par Ctrl-C")
    finally:
        destroy()
        print("Nettoyage final realise")