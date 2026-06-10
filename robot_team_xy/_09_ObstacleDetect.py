#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Tache 9 : Marche avant et arret si obstacle

from threading import Thread
from spi_ws2812 import Adeept_SPI_LedPixel
from _01_LedAvant import * 
from _05_ultrason import * 
from _02_LedWS2812 import *
from _04_motor import *



# --- Parametres ---
SPEED  = 40   # % vitesse (reduite pour les tests)
Distance_Obstacle = 2000# mm
WARNING_DIST  = 400   # mm - seuil d'alerte avant arret
STOP_DIST = 200
RAMP_TIME     = 0.5  # secondes



if __name__ == "__main__":
    print("=== Tache 9 - Marche avant et arret obstacle ===")
    print("  M : demarrer en marche avant")
    print("  A : arret immediat")
    print("  Ctrl-C : quitter")
    print()
    cmd = input("Commande : ").strip().upper()
    try:
        Thread(target=checkdist, daemon=True).start()
        while True:
            if Distance_Obstacle < STOP_DIST:
                stop()
                print("Obstacle detecte a %.2f mm - Arret!" % Distance_Obstacle)
                set_all_switch_off()
                cmd = input("Commande : ").strip().upper()
            if STOP_DIST < Distance_Obstacle <= WARNING_DIST:
                print("Obstacle detecte a %.2f mm - Attention!" % Distance_Obstacle)
                cmd = input("Commande : ").strip().upper()
            if cmd == "M":
                drive_ramp(SPEED, 1, RAMP_TIME)
                print("Marche avant...")
    except KeyboardInterrupt:
        print("\nFin de programme par Ctrl-C")
    finally:
        destroy()
        print("Nettoyage final realise")
