#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Mesure DIRECTE du rayon de virage : roues a plein braquage, le robot
# tourne en rond. Tu mesures le diametre du cercle trace au sol.
#   R = diametre / 2     puis    STEER_MAX_WHEEL_DEG = atan(L / R)
#
# Vitesse = SPEED_AVOID (22%), comme les arcs d'evitement reels, pour que
# le sous-virage mesure corresponde aux conditions de la mission.
# Ctrl-C pour arreter.

import time
from _03_servo import set_angle, to_servo_angle
from _04_motor import setup, stop, drive, CENTER_ANGLE

STEER_AMOUNT = 38      # plein braquage de la mission
SPEED        = 22      # % : SPEED_AVOID, la vraie vitesse de virage en evitement
DIRECTION    = 1       # +38 = braque a gauche (materiel inverse) ; peu importe ici

def main():
    setup()
    set_angle(0, to_servo_angle(CENTER_ANGLE + STEER_AMOUNT))
    time.sleep(0.4)                       # laisse le servo se placer
    print("Robot en cercle. Chrono affiche ci-dessous (Ctrl-C pour arreter).")
    print("PROTOCOLE : repere un point de depart. Quand le robot le repasse,")
    print("note le temps affiche pour 3 TOURS complets, puis mesure le DIAMETRE.")
    t0 = time.time()
    drive(SPEED, DIRECTION)
    try:
        while True:
            print("\r  t = %.1f s" % (time.time() - t0), end='', flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        stop()
        set_angle(0, to_servo_angle(CENTER_ANGLE))
        print("\nStop. Donne-moi : temps pour 3 tours + diametre du cercle.")

if __name__ == "__main__":
    main()
