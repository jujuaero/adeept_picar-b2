# Tache 9 : Marche avant et arret si obstacle

import time
import sys
import select
import importlib.util
import os
from gpiozero import DistanceSensor, LED

# --- Import 04_motor (nom invalide comme module Python direct) ---
_dir = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    "motor", os.path.join(_dir, "04_motor.py")
)
mot = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mot)

from spi_ws2812 import Adeept_SPI_LedPixel

# --- Capteur ultrason ---
sensor = DistanceSensor(echo=24, trigger=23, max_distance=2)

# --- LEDs WS2812 (8 LEDs, luminosite 60) ---
leds = Adeept_SPI_LedPixel(8, 60)

# --- Phares (LED GPIO) ---
phare_g = LED(9)
phare_d = LED(25)

# --- Parametres ---
SPEED         = 40    # % vitesse (reduite pour les tests)
OBSTACLE_DIST = 20    # cm
RAMP_TIME     = 0.5   # secondes

# --- Etats ---
STOPPED  = 0
RUNNING  = 1
OBSTACLE = 2

state       = STOPPED
_blink      = False
_last_blink = 0.0


def checkdist():
    return sensor.distance * 100


def _set_hazard(on):
    if on:
        leds.set_all_led_color(255, 80, 0)
        phare_g.on()
        phare_d.on()
    else:
        leds.set_all_led_color(0, 0, 0)
        phare_g.off()
        phare_d.off()


def update_blink():
    global _blink, _last_blink
    now = time.time()
    if now - _last_blink >= 0.4:
        _blink = not _blink
        _last_blink = now
        _set_hazard(_blink)


def start_move():
    global state
    _set_hazard(False)
    mot.drive_ramp(SPEED, 1, ramp_time=RAMP_TIME)
    state = RUNNING
    print("-> Marche avant %d%%" % SPEED)


def stop_robot(reason="manuel"):
    global state
    mot.stop()
    state = STOPPED
    print("-> Arret (%s)" % reason)


def obstacle_stop():
    global state, _last_blink
    mot.stop()
    state = OBSTACLE
    _last_blink = 0.0
    print("-> OBSTACLE detecte ! Feux de detresse actives")


def read_cmd():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip()
    return None


def destroy():
    mot.destroy()
    _set_hazard(False)
    sensor.close()


if __name__ == "__main__":
    mot.setup()
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
                    obstacle_stop()

            elif state == OBSTACLE:
                update_blink()

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nFin de programme par Ctrl-C")
    finally:
        destroy()
        print("Nettoyage final realise")
