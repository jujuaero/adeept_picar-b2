#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Parcours de validation : execute une suite de mouvements, puis affiche le
# point d'arrivee theorique (vol d'oiseau) predit par le modele calibre de _12.
# Tu mesures la distance reelle depart->arrivee au sol et tu compares.
#
# Le parcours est joue EN CONTINU (comme la croisiere de la mission) : une
# seule accel au depart, une seule decel a la fin, le braquage change a la
# volee entre les segments. Evite de payer la taxe accel/decel a chaque bout.

import time
import math
from _03_servo import set_angle, to_servo_angle
from _04_motor import setup, stop, drive, CENTER_ANGLE

# --- constantes calibrees (identiques a _12_MissionBObstacle.py) ---
SPEED_PROFILE_MM_S  = ((30.0, 250.0), (40.0, 315.0), (50.0, 380.0))
STEER_AMOUNT        = 38
STEER_MAX_WHEEL_DEG = 20.0
ROBOT_LENGTH_MM     = 120.0
TURN_SCRUB_FULL     = 0.82     # patinage : vitesse d'arc/droite a plein braquage
STEER_CH            = 0
# /!\ Materiel inverse : un offset POSITIF (CENTER+38) braque physiquement a
# GAUCHE (le servo sonar est inverse pareil, donc la mission _12 reste correcte).
# Ici on veut predire le mouvement REEL -> +offset = gauche.

# --- PARCOURS : (offset_braquage, vitesse%, duree_s) ---
# offset 0 = tout droit, +38 = plein droite, -38 = plein gauche
PATH = [
    (0,  30, 3.0),    # tout droit
    (38, 30, 2.5),    # un seul virage (~90 deg) -> pas d'annulation sensible
    (0,  30, 3.0),    # tout droit
]

# ---------------------------------------------------------------- modele
def speed_mm_s(pct):
    sign = -1.0 if pct < 0 else 1.0
    pct = abs(float(pct)); P = SPEED_PROFILE_MM_S
    if pct <= P[0][0]:
        p0, v0 = P[0]; p1, v1 = P[1]
    elif pct >= P[-1][0]:
        p0, v0 = P[-2]; p1, v1 = P[-1]
    else:
        for i in range(len(P) - 1):
            p0, v0 = P[i]; p1, v1 = P[i + 1]
            if p0 <= pct <= p1:
                break
    r = (pct - p0) / max(1e-6, p1 - p0)
    return sign * (v0 + r * (v1 - v0))

def curvature_per_mm(off):
    delta = max(-1.0, min(1.0, off / float(STEER_AMOUNT)))
    if abs(delta) < 0.05:
        return 0.0
    wheel = math.radians(delta * STEER_MAX_WHEEL_DEG)
    return math.tan(wheel) / ROBOT_LENGTH_MM   # +offset = gauche (cf. note materiel)

def turn_scrub(off):
    delta = max(-1.0, min(1.0, off / float(STEER_AMOUNT)))
    return 1.0 - (1.0 - TURN_SCRUB_FULL) * abs(delta)

def simulate(path):
    x = y = 0.0; phi = math.pi / 2.0; path_len = 0.0; DT = 0.005
    for off, spd, dur in path:
        v = speed_mm_s(spd) * turn_scrub(off); k = curvature_per_mm(off); t = 0.0
        while t < dur:
            step = min(DT, dur - t); ds = v * step; path_len += abs(ds)
            if abs(k) < 1e-9:
                x += ds * math.cos(phi); y += ds * math.sin(phi)
            else:
                phin = phi + k * ds
                x += (math.sin(phin) - math.sin(phi)) / k
                y += -(math.cos(phin) - math.cos(phi)) / k
                phi = phin
            t += step
    return x, y, math.degrees(phi - math.pi / 2.0), path_len

# ---------------------------------------------------------------- execution
def run_path(path):
    setup()
    # Pre-place le braquage du 1er segment AVANT de rouler (robot a l'arret),
    # puis on ne s'arrete plus : changements de braquage a la volee.
    set_angle(STEER_CH, to_servo_angle(CENTER_ANGLE + path[0][0]))
    time.sleep(0.5)
    print("Parcours dans 3s... place un repere sous un point du robot MAINTENANT.")
    time.sleep(3.0)
    for i, (off, spd, dur) in enumerate(path, 1):
        kind = "tout droit" if off == 0 else ("droite" if off > 0 else "gauche")
        set_angle(STEER_CH, to_servo_angle(CENTER_ANGLE + off))   # a la volee
        drive(spd, 1)
        print("  %d/%d  %-10s  %d%%  %.1fs" % (i, len(path), kind, spd, dur))
        time.sleep(dur)
    stop()
    set_angle(STEER_CH, to_servo_angle(CENTER_ANGLE))

def main():
    x, y, dhead, plen = simulate(PATH)
    crow = math.hypot(x, y)
    try:
        run_path(PATH)
    except KeyboardInterrupt:
        stop()
        print("\nInterrompu.")
        return
    finally:
        stop()
    print("\n=================== CIBLE THEORIQUE ===================")
    print("  Arrivee (repere depart) : x=%+.0f mm (cote), y=%+.0f mm (avant)" % (x, y))
    print("  Cap final tourne        : %+.1f deg" % dhead)
    print("  Longueur parcourue      : %.0f mm" % plen)
    print("  >>> VOL D'OISEAU theorique : %.0f mm  (%.2f m) <<<" % (crow, crow / 1000.0))
    print("  Mesure la distance reelle entre les 2 reperes et compare.")
    print("======================================================")

if __name__ == "__main__":
    main()
