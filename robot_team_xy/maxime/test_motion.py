#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# TEST STANDALONE : compare le mouvement REEL du robot a ce que le modele
# d'ego-motion PREDIT (celui de _12_MissionBObstacle).
#
# N'importe rien du programme principal et n'ecrit aucun fichier.
# A lancer sur le Pi, robot au sol avec de l'espace devant.
# Resume final affiche dans le terminal : modele vs mesure + erreur.

import time
import math

from _03_servo import set_angle, to_servo_angle
from _04_motor import setup, stop, drive, CENTER_ANGLE

# --- constantes COPIEES du modele actuel (_12) pour rester isole ---
# /!\ A garder synchronisees avec _12_MissionBObstacle.py (etalonnage 2026-06-29)
STEER_CH            = 0
STEER_AMOUNT        = 38
STEER_MAX_WHEEL_DEG = 20.0
ROBOT_LENGTH_MM     = 120.0
SPEED_PROFILE_MM_S  = ((30.0, 250.0), (40.0, 315.0), (50.0, 380.0))

# --- plan de test (modifie librement) ---
STRAIGHT_TESTS = [(30, 3.0), (40, 3.0)]      # (vitesse %, duree s)
TURN_TESTS     = [(38, 2.5), (-38, 2.5)]     # (offset braquage, duree s)
TURN_SPEED     = 35


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))

# Reproduction exacte des formules de _12 -------------------------------
def model_speed_mm_s(pct):
    sign = -1.0 if pct < 0 else 1.0
    pct = abs(float(pct))
    P = SPEED_PROFILE_MM_S
    if pct <= P[0][0]:
        p0, v0 = P[0]; p1, v1 = P[1]
    elif pct >= P[-1][0]:
        p0, v0 = P[-2]; p1, v1 = P[-1]
    else:
        p0, v0 = P[0]; p1, v1 = P[1]
        for i in range(len(P) - 1):
            p0, v0 = P[i]; p1, v1 = P[i + 1]
            if p0 <= pct <= p1:
                break
    r = (pct - p0) / max(1e-6, p1 - p0)
    return sign * (v0 + r * (v1 - v0))

def model_curv_per_mm(steer_off):
    delta = _clamp(steer_off / float(STEER_AMOUNT), -1.0, 1.0)
    if abs(delta) < 0.05:
        return 0.0
    wheel = math.radians(delta * STEER_MAX_WHEEL_DEG)
    return -(1.0 / ROBOT_LENGTH_MM) * math.tan(wheel)

# --- helpers materiel ---------------------------------------------------
def steer_off(off):
    set_angle(STEER_CH, to_servo_angle(CENTER_ANGLE + off))

def ask(prompt):
    raw = input(prompt).strip().lower().replace(',', '.')
    if raw in ('s', 'skip', ''):
        return None
    try:
        return float(raw)
    except ValueError:
        print("  (nombre invalide -> saute)")
        return None

def countdown(n=3):
    for i in range(n, 0, -1):
        print('  %d...' % i, end='', flush=True)
        time.sleep(1.0)
    print(' GO')

def run(secs, speed, direction=1, off=0):
    steer_off(off)
    time.sleep(0.25)                 # laisse le servo se placer
    t0 = time.perf_counter()
    drive(speed, direction)
    while time.perf_counter() - t0 < secs:
        time.sleep(0.005)
    stop()
    steer_off(0)


results = []   # liste de dict pour le resume

def main():
    setup()
    steer_off(0)
    print("=== TEST mouvement reel vs modele (standalone) ===")
    print("Robot au sol, espace devant. ENTREE vide ou 's' = sauter un test.\n")

    # ---- avance en ligne droite ----
    for spd, T in STRAIGHT_TESTS:
        input("ENTREE pour : tout droit %.1fs a %d%% ..." % (T, spd))
        countdown()
        run(T, spd)
        meas = ask("  distance reelle mesuree (mm) : ")
        pred = model_speed_mm_s(spd) * T
        results.append({'kind': 'straight', 'label': 'Avance %d%% %.1fs' % (spd, T),
                        'spd': spd, 'T': T, 'pred': pred, 'meas': meas})

    # ---- braquage ----
    for off, T in TURN_TESTS:
        side = 'droite' if off > 0 else 'gauche'
        input("ENTREE pour : braquage %+d (%s) %.1fs a %d%% ..." % (off, side, T, TURN_SPEED))
        countdown()
        run(T, TURN_SPEED, off=off)
        meas = ask("  angle reel tourne (deg, positif) : ")
        ds = model_speed_mm_s(TURN_SPEED) * T
        pred = abs(math.degrees(model_curv_per_mm(off) * ds))
        results.append({'kind': 'turn', 'label': 'Braquage %+d %.1fs' % (off, T),
                        'off': off, 'T': T, 'ds': ds, 'pred': pred, 'meas': meas})

    summary()


def summary():
    print("\n=================== RESUME ===================")
    print("%-20s %10s %10s %12s" % ('test', 'modele', 'mesure', 'erreur'))
    print("-" * 56)
    for r in results:
        if r['meas'] is None:
            print("%-20s %10.1f %10s %12s" % (r['label'], r['pred'], '-', '(saute)'))
            continue
        err = r['meas'] - r['pred']
        if r['kind'] == 'straight':
            pct = 100.0 * err / max(1.0, r['pred'])
            print("%-20s %8.0fmm %8.0fmm %+7.0fmm %+.0f%%"
                  % (r['label'], r['pred'], r['meas'], err, pct))
        else:
            print("%-20s %8.1fd %8.1fd %+9.1fd" % (r['label'], r['pred'], r['meas'], err))

    # valeurs reelles a reporter dans _12 si on veut recaler le modele
    print("\nValeurs reelles implicites :")
    for r in results:
        if r['meas'] is None:
            continue
        if r['kind'] == 'straight':
            v_real = r['meas'] / r['T']
            v_mod  = model_speed_mm_s(r['spd'])
            print("  %d%% : reel %.0f mm/s  (modele %.0f mm/s)"
                  % (r['spd'], v_real, v_mod))
        else:
            ang = math.radians(r['meas'])
            if ang > 1e-4 and r['ds'] > 1e-3:
                R = r['ds'] / ang
                print("  braquage %+d : rayon reel ~%.0f mm  (corde non mesuree, base sur ds modele)"
                      % (r['off'], R))
    print("==============================================")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrompu.")
    finally:
        stop()
