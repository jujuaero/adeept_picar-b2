#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Analyse de l'ESPACE DEVANT par balayage sonar (robuste au milieu charge).
#
# On ne cherche PAS a "mesurer un objet isole" (vision de labo). On regarde le
# profil bloque / libre et on en tire ce dont l'evitement a besoin :
#   - la MENACE : le secteur bloque le plus proche (ou ca bloque, a quelle
#     distance, sur quelle largeur laterale -> de combien il faut se decaler)
#   - les PASSAGES : les trouees libres, leur ouverture laterale reelle, et si
#     le robot (largeur ROBOT_WIDTH_MM) peut passer
#   - la direction conseillee = vers la meilleure trouee
#
# Le faisceau large du sonar GONFLE les zones bloquees -> les passages sont
# sous-estimes : c'est PRUDENT (donc safe) pour de l'evitement.

import time
import math
from _03_servo import set_angle
from _05_ultrason import checkdist

US_CH        = 1
US_FORWARD   = 100
US_RIGHT     = 58
US_LEFT      = 142
DEG_PER_UNIT = 45.0 / 42.0

SCAN_STEP    = 1
SETTLE       = 0.05
SAMPLES      = 3
VALID_MIN    = 30
VALID_MAX    = 1900          # >= : pas d'echo = libre/loin
DANGER_MM    = 700           # en deca, l'angle est "bloque" (menace dans le chemin)
ROBOT_WIDTH_MM = 170         # largeur du robot
SAFETY_MM    = 60            # marge de chaque cote
MIN_GAP_MM   = ROBOT_WIDTH_MM + 2 * SAFETY_MM   # ouverture mini pour passer

def _raw_to_rad(raw):
    return math.radians(-90 - (raw - US_FORWARD) * DEG_PER_UNIT)

def to_xy(raw, dist):
    rad = _raw_to_rad(raw)
    return math.cos(rad) * dist, -math.sin(rad) * dist   # (x lateral +droite, y avant)

def deg_off(raw):
    return (raw - US_FORWARD) * DEG_PER_UNIT             # + = droite

def measure_dist():
    vals = []
    for _ in range(SAMPLES):
        d = checkdist()
        if d is not None and VALID_MIN <= d < VALID_MAX:
            vals.append(float(d))
        time.sleep(0.012)
    if not vals:
        return None                # pas d'echo -> libre
    vals.sort()
    return vals[len(vals) // 2]

def scan():
    out = []
    lo, hi = min(US_RIGHT, US_LEFT), max(US_RIGHT, US_LEFT)
    for raw in range(lo, hi + 1, SCAN_STEP):
        set_angle(US_CH, raw)
        time.sleep(SETTLE)
        out.append((raw, measure_dist()))
    set_angle(US_CH, US_FORWARD)
    return out

def classify(readings):
    """-> liste de zones (kind, pings) ou kind in {'block','free'}, contigues."""
    zones = []
    for raw, d in readings:
        blocked = (d is not None) and (d < DANGER_MM)
        kind = 'block' if blocked else 'free'
        if zones and zones[-1][0] == kind:
            zones[-1][1].append((raw, d))
        else:
            zones.append((kind, [(raw, d)]))
    return zones

def lateral_opening(left_edge, right_edge):
    """Distance physique entre deux pings (les flancs d'une trouee)."""
    (rl, dl), (rr, dr) = left_edge, right_edge
    xl, yl = to_xy(rl, dl)
    xr, yr = to_xy(rr, dr)
    return math.hypot(xr - xl, yr - yl)

def report(readings):
    zones = [z for z in classify(readings) if len(z[1]) >= 2]
    if not zones:
        print("Rien d'exploitable.")
        return

    blocks = [z[1] for z in zones if z[0] == 'block']
    print("\n=================== ESPACE DEVANT ===================")

    # --- menace : bloc le plus proche ---
    if blocks:
        nearest = min(blocks, key=lambda b: min(d for _, d in b if d))
        ds = [d for _, d in nearest if d]
        a1, a2 = nearest[0], nearest[-1]
        lat_w = lateral_opening(a1, a2)                       # largeur laterale du bloc
        c_off = (deg_off(a1[0]) + deg_off(a2[0])) / 2.0
        print("  MENACE la plus proche :")
        print("    distance       : %.0f mm" % min(ds))
        print("    secteur         : %.0f deg -> %.0f deg (centre %+.0f deg)"
              % (deg_off(a1[0]), deg_off(a2[0]), c_off))
        print("    largeur laterale: ~%.0f mm  (de combien se decaler pour la longer)" % lat_w)
    else:
        print("  Aucune menace sous %d mm : voie libre devant." % DANGER_MM)

    # --- passages (trouees libres) ---
    print("  ----------------------------------------------------")
    print("  PASSAGES (trouees libres, ouverture >= %d mm pour passer) :" % MIN_GAP_MM)
    full = classify(readings)
    any_gap = False
    best = None
    for i, (kind, pings) in enumerate(full):
        if kind != 'free' or len(pings) < 2:
            continue
        # flancs : dernier ping du bloc a gauche / premier du bloc a droite
        left = full[i-1][1][-1] if i > 0 and full[i-1][1][-1][1] else pings[0]
        right = full[i+1][1][0] if i+1 < len(full) and full[i+1][1][0][1] else pings[-1]
        # si la trouee touche un bord de scan sans bloc -> consideree ouverte
        open_left  = (i == 0)
        open_right = (i == len(full) - 1)
        if open_left or open_right:
            opening = 9999
        else:
            opening = lateral_opening(left, right)
        c_off = (deg_off(pings[0][0]) + deg_off(pings[-1][0])) / 2.0
        ok = opening >= MIN_GAP_MM
        any_gap = any_gap or ok
        tag = "OK" if ok else "trop etroit"
        shown = ">%d" % MIN_GAP_MM if opening == 9999 else "%.0f" % opening
        print("    centre %+4.0f deg | ouverture ~%s mm | %s" % (c_off, shown, tag))
        if ok and (best is None or abs(c_off) < abs(best[0])):
            best = (c_off, opening)

    print("  ----------------------------------------------------")
    if best:
        print("  >>> CONSEIL : viser la trouee a %+.0f deg <<<" % best[0])
    elif not any_gap:
        print("  >>> Aucun passage assez large : reculer / demi-tour <<<")
    print("=====================================================")

def main():
    print("Scan de l'espace devant...")
    report(scan())

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        set_angle(US_CH, US_FORWARD)
        print("\nInterrompu.")
