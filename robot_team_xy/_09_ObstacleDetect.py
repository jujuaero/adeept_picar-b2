#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Tache 9 : Marche avant et arret si obstacle

from threading import Thread
from _01_LedAvant import * 
from _05_ultrason import * 
from _02_LedWS2812 import *
from _04_motor import *



# --- Parametres ---
SPEED  = 40   # % vitesse (reduite pour les tests)
distance= 2000# mm
WARNING_DIST  = 800   # mm - seuil d'alerte avant arret
STOP_DIST = 400
RAMP_TIME     = 0.5  # secondes


def arretUrgence(stop_distance,warning_distance):
    while True:
        distance = checkdist()
        if distance < stop_distance :
            stop()
            warning()
            print("Obstacle detecte a %.2f mm - Arret!" % distance)
            return
        elif stop_distance <= distance < warning_distance:
            print("Obstacle detecte a %.2f mm - Attention!" % distance)



if __name__ == "__main__":
    setup()
    switchSetup()
    print("=== Tache 9 - Marche avant et arret obstacle ===")
    print("  M : demarrer en marche avant")
    print("  A : arret immediat")
    print("  Ctrl-C : quitter")
    print()
    cmd = input("Commande : ").strip().upper()
    try:
        Running = Thread(target=arretUrgence, args=(STOP_DIST, WARNING_DIST), daemon=True)
        while True:
            if cmd == "M":
                Running.start()
                drive_ramp(SPEED, 1,RAMP_TIME)
                print("Marche avant...")
                cmd ="waiting"
            if not Running.is_alive():
                cmd = input("Commande : ").strip().upper()
    except KeyboardInterrupt:
        print("\nFin de programme par Ctrl-C")
    finally:
        destroy()
        set_all_switch_off()
        print("Nettoyage final realise")
