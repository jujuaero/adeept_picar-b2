#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Tache 9 : Marche avant et arret si obstacle

import time
import sys
import select
import importlib
from gpiozero import DistanceSensor, LED
from spi_ws2812 import Adeept_SPI_LedPixel
from robot_team_xy._01_LedAvant import * as led
from 05_ultrasond import * as ultra


# Import dynamique des moteurs car le fichier commence par un chiffre
motor_drv = importlib.import_module("04_motor")

# --- Capteur ultrason ---
sensor = DistanceSensor(echo=24, trigger=23, max_distance=2, queue_len=1)

# --- LEDs WS2812 ---
leds = Adeept_SPI_LedPixel(12, 60)

# --- Phares ---
phare_g = LED(9)
phare_d = LED(25)

# --- Parametres ---
SPEED         = 40   # % vitesse (reduite pour les tests)
OBSTACLE_DIST = 20   # cm
WARNING_DIST  = 40   # cm - seuil d'alerte avant arret
RAMP_TIME     = 0.5  # secondes

# --- Etats ---
STOPPED  = 0
RUNNING  = 1
OBSTACLE = 2

state         = STOPPED
_blink        = False
_last_blink   = 0.0
_warned       = False

def checkdist():
    d = sensor.distance
    if d is None:
        return 999
    return d * 100

def _set_leds(r, g, b, phares_on=False):
    leds.set_all_led_color(r, g, b)
    if phares_on:
        phare_g.on()
        phare_d.on()
    else:
        phare_g.off()
        phare_d.off()

def update_blink():
    global _blink, _last_blink
    now = time.time()
    if now - _last_blink >= 0.4:
        _blink = not _blink
        _last_blink = now
        if _blink:
            _set_leds(255, 0, 0, True)   # rouge + phares
        else:
            _set_leds(0, 0, 0, False)

def start_move():
    global state, _warned
    _warned = False
    _set_leds(0, 255, 0, False)   # vert au depart
    state = RUNNING
    
    # On gère manuellement la rampe ici pour intégrer la verification ultrason
    steps = 50
    delay = RAMP_TIME / steps
    hit_obstacle = False
    for i in range(1, steps + 1):
        if checkdist() < OBSTACLE_DIST:
            motor_drv.stop()
            hit_obstacle = True
            break
        motor_drv.drive(SPEED * i / steps, 1)
        time.sleep(delay)
        
    if hit_obstacle:
        obstacle_stop()
    else:
        print("-> Marche avant %d%%" % SPEED)

def stop_robot(reason="manuel"):
    global state
    motor_drv.stop()
    state = STOPPED
    print("-> Arret (%s)" % reason)

def obstacle_stop():
    global state, _last_blink
    motor_drv.stop()
    state = OBSTACLE
    _last_blink = 0.0
    time.sleep(0.15)
    stable = 0
    prev = checkdist()
    while stable < 3:
        time.sleep(0.05)
        curr = checkdist()
        if abs(curr - prev) < 0.08:
            stable += 1
        else:
            stable = 0
        prev = curr
    print("-> ARRET  : distance finale %.1f cm" % curr)
    print("-> OBSTACLE detecte ! Feux de detresse actives")

def read_cmd():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip()
    return None

def destroy():
    motor_drv.stop()
    if hasattr(motor_drv, 'pwm') and motor_drv.pwm:
        motor_drv.pwm.deinit()
    _set_leds(0, 0, 0, False)
    sensor.close()

if __name__ == "__main__":
    motor_drv.setup()
    leds.daemon = True
    leds.start()

    print("=== Tache 9 - Marche avant et arret obstacle ===")
    print("  M : demarrer en marche avant")
    print("  A : arret immediat")
    print("  Ctrl-C : quitter")
    print()

    try:
        while True:
            cmd = read_cmd()

            if cmd in ("M", "m"):
                if state in (STOPPED, OBSTACLE):
                    start_move()

            elif cmd in ("A", "a"):
                if state != STOPPED:
                    stop_robot(reason="manuel")

            if state == RUNNING:
                dist = checkdist()
                if dist < OBSTACLE_DIST:
                    print("-> STOP  : obstacle a %.1f cm" % dist)
                    obstacle_stop()
                elif dist < WARNING_DIST and not _warned:
                    _warned = True
                    _set_leds(255, 80, 0, True)    # orange + phares
                    print("-> ALERTE: obstacle a %.1f cm" % dist)
                elif dist >= WARNING_DIST and _warned:
                    _warned = False
                    _set_leds(0, 255, 0, False)    # retour vert

            elif state == OBSTACLE:
                update_blink()

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nFin de programme par Ctrl-C")
    finally:
        destroy()
        print("Nettoyage final realise")
