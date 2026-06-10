#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Tache 10 : Suivi de source lumineuse et obstacle
import time
import sys
import select
from gpiozero import TonalBuzzer

import _04_motor as motor_drv
import _05_ultrason as ultra_drv
import _01_LedAvant as led_av
import _02_LedWS2812 as led_ws
import _09_lightTracking as light_drv
from _03_servo import *

# --- Capteur de lumiere (ADS7830) ---
adc = light_drv.ADS7830()

# --- Buzzer ---
buzzer = TonalBuzzer(18)

# --- Parametres ---
STEER_ANGLE   = 30
SPEED         = 40
REVERSE_SPEED = 25
REVERSE_TIME  = 2.0
OBSTACLE_DIST = 200   # mm
WARNING_DIST  = 400   # mm
LIGHT_BASE    = 127
LIGHT_THRESH  = 15
channel       = 0


# --- Etats ---
STOPPED = 0
RUNNING = 1

state   = STOPPED
_warned = False


def _set_leds(couleur, phares_on=False):
    for i in range(14):
        led_ws.set_led(i, couleur)
    if phares_on:
        led_av.light_on(led_av.LEDS[1])
        led_av.light_on(led_av.LEDS[2])
    else:
        led_av.light_off(led_av.LEDS[1])
        led_av.light_off(led_av.LEDS[2])


def _blink_red(duration):
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
        time.sleep(0.02)


def track_light():
    val = adc.analogRead(1)
    if val < LIGHT_BASE - LIGHT_THRESH:
        set_angle(channel, motor_drv.CENTER_ANGLE + STEER_ANGLE)
    elif val > LIGHT_BASE + LIGHT_THRESH:
        set_angle(channel, motor_drv.CENTER_ANGLE - STEER_ANGLE)
    else:
        set_angle(channel, motor_drv.CENTER_ANGLE)
    motor_drv.drive(SPEED, 1)


def obstacle_recovery():
    global state, _warned

    print("-> Feux de detresse (1s)")
    _blink_red(1.0)

    # Recul avec bip bip
    print("-> Recul ~30 cm avec bip bip")
    set_angle(channel, motor_drv.CENTER_ANGLE)
    motor_drv.drive(REVERSE_SPEED, -1)
    t_end = time.time() + REVERSE_TIME
    bip = False
    t_bip = time.time()
    while time.time() < t_end:
        _set_leds("R", True)
        if time.time() - t_bip >= 0.5:
            bip = not bip
            t_bip = time.time()
            buzzer.play("A4") if bip else buzzer.stop()
        time.sleep(0.05)

    motor_drv.stop()
    buzzer.stop()
    _set_leds("N", False)

    # Pause 2 secondes
    print("-> Pause 2s")
    time.sleep(2.0)

    # Reprise
    _warned = False
    _set_leds("G", False)
    state = RUNNING
    print("-> Reprise suivi lumiere")


def stop_robot(reason="manuel"):
    global state
    motor_drv.stop()
    set_angle(channel, motor_drv.CENTER_ANGLE)
    state = STOPPED
    _set_leds("N", False)
    print("-> Arret (%s)" % reason)


def start_move():
    global state, _warned
    _warned = False
    _set_leds("G", False)
    state = RUNNING
    print("-> Suivi lumiere demarre")


def read_cmd():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip()
    return None


def destroy():
    motor_drv.stop()
    motor_drv.destroy()
    buzzer.stop()
    _set_leds("N", False)
    led_av.set_all_switch_off()


if __name__ == "__main__":
    motor_drv.setup()
    led_av.switchSetup()
    led_av.set_all_switch_off()

    print("=== Tache 10 - Suivi lumiere et obstacle ===")
    print("  M : demarrer")
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
                dist = ultra_drv.checkdist()

                if dist < OBSTACLE_DIST:
                    motor_drv.stop()
                    print("-> STOP  : obstacle a %.0f mm" % dist)
                    obstacle_recovery()

                elif dist < WARNING_DIST and not _warned:
                    _warned = True
                    _set_leds("R", True)
                    print("-> ALERTE: obstacle a %.0f mm" % dist)

                elif dist >= WARNING_DIST and _warned:
                    _warned = False
                    _set_leds("G", False)

                if state == RUNNING:
                    track_light()

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nFin de programme par Ctrl-C")
    finally:
        destroy()
        print("Nettoyage final realise")
