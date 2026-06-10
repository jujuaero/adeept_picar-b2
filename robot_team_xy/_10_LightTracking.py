#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Tache 10 : Suivi de source lumineuse et obstacle
import time, sys, select
from threading import Thread

import _01_LedAvant as led_av
import _02_LedWS2812 as led_ws
import _09_lightTracking as light_drv
from _03_servo import *
from _09_ObstacleDetect import *
from _04_motor import *

# --- Capteur de lumiere (ADS7830) ---
adc = light_drv.ADS7830()

# --- Parametres ---
STEER_ANGLE   = 30
SPEED         = 40
REVERSE_SPEED = 25
REVERSE_TIME  = 2.0
LIGHT_BASE    = 127
LIGHT_THRESH  = 15
channel       = 0

# --- Etats ---
STOPPED = 0
RUNNING = 1

state   = STOPPED

def _set_leds(couleur, phares_on=False):
    for i in range(14):
        led_ws.set_led(i, couleur)
    if phares_on:
        led_av.light_on(led_av.LEDS[1])
        led_av.light_on(led_av.LEDS[2])
    else:
        led_av.light_off(led_av.LEDS[1])
        led_av.light_off(led_av.LEDS[2])


"""def _blink_red(duration):
    t_end = time.time() + duration
    blink = False
    t_last = 0.0
    while time.time() < t_end:
        now = time.time()
        if now - t_last >= 0.4:
            blink = not blink
            t_last = now
            if blink:
                _set_leds("R", True)
            else:
                _set_leds("N", False)
        time.sleep(0.02)"""


def track_light():
    val = adc.analogRead(1)
    if val < LIGHT_BASE - LIGHT_THRESH:
        set_angle(channel, to_servo_angle(CENTER_ANGLE + STEER_ANGLE))
    elif val > LIGHT_BASE + LIGHT_THRESH:
        set_angle(channel, to_servo_angle(CENTER_ANGLE - STEER_ANGLE))
    else:
        set_angle(channel, to_servo_angle(CENTER_ANGLE))
    drive(SPEED, 1)


def stop_robot(reason="manuel"):
    global state
    stop()
    set_angle(channel, to_servo_angle(CENTER_ANGLE))
    state = STOPPED
    _set_leds("N", False)
    print("-> Arret (%s)" % reason)


def start_move():
    _set_leds("G", False)
    global state
    state = RUNNING
    print("-> Suivi lumiere demarre")


def read_cmd():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip()
    return None


def destroy():
    stop()
    _set_leds("N", False)
    led_av.set_all_switch_off()


if __name__ == "__main__":
    setup()
    led_av.switchSetup()
    led_av.set_all_switch_off()

    print("=== Tache 10 - Suivi lumiere et obstacle ===")
    print("  M : demarrer")
    print("  A : arret immediat")
    print("  Ctrl-C : quitter")
    print()

    try:
        Running = Thread(target=arretUrgence, args=(STOP_DIST, WARNING_DIST), daemon=True)
        Running.start()
        while True:
            cmd = read_cmd()
            if cmd in ("M", "m"):
                if Running.is_alive() and state == STOPPED:
                    start_move()

            elif cmd in ("A", "a"):
                if state != STOPPED:
                    stop_robot(reason="manuel")

            if state == RUNNING:
                if Running.is_alive():
                    track_light()
                else:
                    state = STOPPED
                    stop_robot(reason="manuel")
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nFin de programme par Ctrl-C")
    finally:
        destroy()
        print("Nettoyage final realise")