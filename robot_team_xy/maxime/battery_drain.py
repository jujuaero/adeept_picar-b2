#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Decharge RAPIDE de la batterie, pour passer vite d'un niveau de charge a
# l'autre entre deux series de calib_vitesse.py.
#
# Tape sur tous les consommateurs a la fois :
#   - 14 LED WS2812 en BLANC PLEIN (le gros courant, ~0.8 A continu)
#   - les 3 servos qui balayent (tirent du courant en bougeant)
#   - les moteurs en alternance avant/arriere (sens inverse periodique pour
#     ne pas s'enfuir ni rester cale)
#
# Affiche la tension qui descend. Robot SUR UN SUPPORT (roues en l'air) de
# preference, sinon il avance/recule sur place. Ctrl-C pour arreter.

import time
from spi_ws2812 import Adeept_SPI_LedPixel
from _03_servo import set_angle, to_servo_angle
from _04_motor import setup, stop, drive
import battery

LED_COUNT   = 14
MOTOR_PCT   = 70
FLIP_PERIOD = 2.5      # s : inversion du sens moteur

def _safe_servo(ch, ang):
    try:
        set_angle(ch, to_servo_angle(ang))
    except Exception:
        pass

def main():
    setup()
    led = Adeept_SPI_LedPixel(LED_COUNT, 255, 'GRB')
    leds_on = led.check_spi_state() != 0
    if leds_on:
        led.set_all_led_color(255, 255, 255)     # blanc plein = courant max
    else:
        print("(SPI/LED indisponible -> decharge via moteurs + servos seulement)")

    print("DECHARGE en cours (Ctrl-C pour arreter). Robot sur support conseille.")
    t0 = last_flip = last_print = time.time()
    direction = 1
    sweep_hi = True

    try:
        while True:
            now = time.time()

            # moteurs : on inverse le sens periodiquement (bref stop pour ne pas
            # encaisser un pic de courant a l'inversion)
            if now - last_flip >= FLIP_PERIOD:
                stop(); time.sleep(0.15)
                direction *= -1
                last_flip = now
            drive(MOTOR_PCT, direction)

            # servos : balayage continu entre deux positions
            a = 40 if sweep_hi else -40
            _safe_servo(0, max(-45, min(50, a)))    # direction (plage -45..50)
            _safe_servo(1, a * 2)                   # -80..80
            _safe_servo(2, a * 2)
            sweep_hi = not sweep_hi

            # tension toutes les 5 s
            if now - last_print >= 5.0:
                v = battery.read_voltage()
                if v is not None:
                    print("\r  t=%4.0fs   %.2f V  (~%.0f%%)        "
                          % (now - t0, v, battery.percentage(v)), end='', flush=True)
                last_print = now

            time.sleep(0.4)
    except KeyboardInterrupt:
        pass
    finally:
        stop()
        for ch in (0, 1, 2):
            _safe_servo(ch, 0)
        if leds_on:
            try:
                led.set_all_led_color(0, 0, 0)
                led.led_close()
            except Exception:
                pass
        print("\nDecharge arretee.")

if __name__ == "__main__":
    main()
