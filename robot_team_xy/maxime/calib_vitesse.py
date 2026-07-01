#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Mesure la vitesse STABILISEE (hors accel/frein) en ligne droite, par la
# methode des deux durees : on roule T_court puis T_long sur la meme ligne,
# et on soustrait. L'accel de depart et le frein de fin sont identiques dans
# les deux runs -> ils s'annulent dans la difference.
#
#   v_stable = (dist_long - dist_court) / (T_long - T_court)
#
# A CHAQUE run, on lit aussi la TENSION batterie (robot a l'arret) et on
# accumule (tension, %, v_stable) dans calib_speed_data.csv. En relancant ce
# script a plusieurs niveaux de charge (max, 3/4, mid, 1/4, bas), on construit
# le modele vitesse = f(throttle, tension).
#
# A lancer sur le Pi, longue ligne droite degagee. ENTREE pour chaque etape.

import time
import os
import csv
from _03_servo import set_angle, to_servo_angle
from _04_motor import setup, stop, drive, CENTER_ANGLE
import battery

STEER_CH = 0
T_COURT  = 2.0
T_LONG   = 4.0
SPEEDS   = [22, 25, 30]                   # vraies vitesses mission (cruise/avoid)
DATA_CSV = os.path.join(os.path.dirname(__file__), "calib_speed_data.csv")

def run_straight(secs, spd):
    set_angle(STEER_CH, to_servo_angle(CENTER_ANGLE))   # roues droites
    time.sleep(0.3)
    for i in (3, 2, 1):
        print('  %d...' % i, end='', flush=True); time.sleep(1.0)
    print(' GO')
    drive(spd, 1)
    time.sleep(secs)
    stop()

def ask(prompt):
    raw = input(prompt).strip().replace(',', '.')
    try:
        return float(raw)
    except ValueError:
        return None

def get_voltage():
    """Tension batterie (robot a l'arret). Si ADC indispo -> saisie manuelle."""
    v = battery.read_voltage()
    if v is None:
        v = ask("  ADC indispo. Tension batterie mesuree (V) : ")
        return v
    print("  tension lue : %.2f V  (~%.0f %%)" % (v, battery.percentage(v)))
    return v

def append_rows(rows):
    new_file = not os.path.exists(DATA_CSV)
    with open(DATA_CSV, "a", newline="") as f:
        w = csv.writer(f)
        if new_file:
            w.writerow(["voltage_V", "pct_throttle", "v_stable_mm_s"])
        for v_batt, spd, v_stable in rows:
            w.writerow(["%.2f" % v_batt, spd, "%.0f" % v_stable])

def main():
    setup()
    set_angle(STEER_CH, to_servo_angle(CENTER_ANGLE))
    print("=== Calibration vitesse vs batterie (methode 2 durees) ===")
    print("Ligne droite degagee. Marque le depart (meme repere a chaque run).")
    print("Donnees ajoutees a : %s\n" % os.path.basename(DATA_CSV))
    rows = []
    for spd in SPEEDS:
        print("--- %d%% ---" % spd)
        v_batt = get_voltage()                 # robot a l'arret -> tension au repos
        input("ENTREE : run COURT %.1fs a %d%% ..." % (T_COURT, spd))
        run_straight(T_COURT, spd)
        d_court = ask("  distance mesuree (mm) : ")
        input("ENTREE : run LONG %.1fs a %d%% (repars du MEME point) ..." % (T_LONG, spd))
        run_straight(T_LONG, spd)
        d_long = ask("  distance mesuree (mm) : ")
        if None not in (v_batt, d_court, d_long):
            v_stable = (d_long - d_court) / (T_LONG - T_COURT)
            rows.append((v_batt, spd, v_stable))
            print("  -> v_stable = %.0f mm/s @ %.2f V\n" % (v_stable, v_batt))
        else:
            print("  -> mesure incomplete, non enregistree\n")

    if rows:
        append_rows(rows)
        print("=================== ENREGISTRE ===================")
        for v_batt, spd, v_stable in rows:
            print("  %d%% : %.0f mm/s  @ %.2f V" % (spd, v_stable, v_batt))
        print("Ajoute a %s" % os.path.basename(DATA_CSV))
        print("Relance a une autre charge (3/4, mid, 1/4, bas) pour completer le modele.")
    else:
        print("Aucune donnee enregistree.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrompu.")
    finally:
        stop()
