#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Mesure la LARGEUR d'un obstacle par balayage du sonar.
#
# Logique :
#   1. On balaye la tete US sur tout l'avant.
#   2. SEGMENTATION PAR CONTINUITE : tant que la distance varie peu d'un pas a
#      l'autre (ex. 500 -> 520), c'est le MEME objet ; un gros saut (520 -> 1200)
#      = bord de l'objet / autre objet.
#   3. On garde le segment le plus PROCHE (l'obstacle devant).
#   4. LARGEUR par les BORDS MI-MONTEE : le bord de l'objet n'est pas le premier
#      echo (qui tombe ~un demi-cone trop tot), mais l'angle ou la distance
#      franchit le seuil (fond + d_face)/2, interpole entre deux pas. Ca place le
#      bord au centre de la zone floue plutot qu'a son bord exterieur, donc sans
#      dependre d'un BEAM_DEG a calibrer par objet.
#         largeur = 2 * d_face * sin(span_bords / 2)
#
# A lancer robot immobile face a l'objet. Pose un objet de largeur CONNUE pour
# verifier. La ligne "1er/dernier echo" reste affichee pour comparaison.

import time
import math
from _03_servo import set_angle
from _05_ultrason import checkdist

US_CH       = 1
US_FORWARD  = 100
US_RIGHT    = 58
US_LEFT     = 142
DEG_PER_UNIT = 45.0 / 42.0      # conversion unite servo -> degres (cf. _12)

SCAN_STEP   = 1          # pas de balayage (unites servo) : fin pour les bords
SETTLE      = 0.05       # s : laisse la tete se placer
SAMPLES     = 3          # mediane par angle
VALID_MAX   = 1900       # ~2000 = pas d'echo
VALID_MIN   = 30
JUMP_MM     = 150        # saut de distance qui marque un bord / autre objet
MIN_PINGS   = 3          # un segment doit avoir au moins ca de pings
FACE_TOL    = 40         # mm : FACE avant (sert a la mesure de COMPARAISON)
BEAM_DEG    = 60.0       # cone efficace (pour la mesure de COMPARAISON 1er/dernier echo)
NO_ECHO_MM  = 2000.0     # niveau "fond" attribue a un pas sans echo valide
DEBUG       = True       # logge chaque angle -> distance pendant le balayage

def measure_dist():
    vals = []
    for _ in range(SAMPLES):
        d = checkdist()
        if d is not None and VALID_MIN <= d < VALID_MAX:
            vals.append(float(d))
        time.sleep(0.012)
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2]

def scan():
    """Balaye et renvoie la liste sequentielle (raw, dist) des echos valides."""
    readings = []
    lo, hi = min(US_RIGHT, US_LEFT), max(US_RIGHT, US_LEFT)
    if DEBUG:
        print("  raw |  angle |  distance")
        print("  ----+--------+----------")
    for raw in range(lo, hi + 1, SCAN_STEP):
        set_angle(US_CH, raw)
        time.sleep(SETTLE)
        d = measure_dist()
        readings.append((raw, d))      # d peut etre None (pas d'echo)
        if DEBUG:
            off = (raw - US_FORWARD) * DEG_PER_UNIT
            ds = "%6.0f mm" % d if d is not None else "  --- (pas d'echo)"
            print("  %3d | %+5.1f | %s" % (raw, off, ds))
    set_angle(US_CH, US_FORWARD)
    return readings

def segment(readings):
    """Decoupe par continuite : un saut > JUMP_MM (ou un trou d'echo) ferme le
    segment courant.

    Chaque segment retenu est un dict :
        {'pts': [(raw, dist), ...], 'open_lo': bool, 'open_hi': bool}
    open_lo / open_hi valent True quand ce bord du segment N'A PAS ete vu : le
    segment bute sur la limite du balayage (l'objet continue au-dela). Ils valent
    False quand un VRAI bord ferme le segment (saut de distance ou perte d'echo).
    Sans bord reel des deux cotes, on ne peut pas mesurer une largeur."""
    segments = []
    pts = []
    start_idx = 0

    def flush(end_open):
        if len(pts) >= MIN_PINGS:
            segments.append({
                'pts': list(pts),
                'open_lo': (start_idx == 0),   # commence sur la limite de scan ?
                'open_hi': end_open,           # fini sur la limite de scan ?
            })

    for i, (raw, d) in enumerate(readings):
        if d is None:                          # perte d'echo = vrai bord
            flush(False)
            pts = []
            start_idx = i + 1
            continue
        if pts and abs(d - pts[-1][1]) > JUMP_MM:   # saut = vrai bord
            flush(False)
            pts = []
            start_idx = i
        if not pts:
            start_idx = i
        pts.append((raw, d))
    flush(True)                                # fin de balayage = bord non vu
    return segments

def width_of(seg):
    """COMPARAISON : ancienne largeur (loi des cosinus sur 1er/dernier echo),
    brute et corrigee d'un cone constant BEAM_DEG. Gardee pour reference."""
    raw1, d1 = seg[0]
    raw2, d2 = seg[-1]
    dtheta = math.radians(abs(raw2 - raw1) * DEG_PER_UNIT)
    def chord(dth):
        return math.sqrt(max(0.0, d1 * d1 + d2 * d2 - 2 * d1 * d2 * math.cos(dth)))
    w_raw = chord(dtheta)
    dtheta_corr = max(0.0, dtheta - math.radians(BEAM_DEG))
    w_corr = chord(dtheta_corr)
    return w_raw, w_corr, math.degrees(dtheta)

def _dval(d):
    """Distance numerique d'un ping : le fond (pas d'echo / hors plage) devient
    NO_ECHO_MM pour que la mi-montee ait un niveau haut coherent."""
    return float(d) if (d is not None and VALID_MIN <= d < VALID_MAX) else NO_ECHO_MM

def edge_raw(readings, i_center, d_face, step):
    """Bord de l'objet par MI-MONTEE, du cote 'step' (-1 = vers index bas, +1 =
    vers index haut).

    On part du ping le plus proche (centre de la face) et on s'eloigne jusqu'a
    retrouver le fond. Le bord est l'angle (raw) ou la distance franchit
    (fond + d_face)/2, interpole lineairement entre les deux pas qui l'encadrent
    -> resolution sous le pas servo.

    Renvoie None si le fond n'est jamais atteint de ce cote : l'objet deborde du
    champ balaye, le bord n'est pas vu (largeur non mesurable)."""
    n = len(readings)
    seq = []                       # (raw, dval) du centre vers l'exterieur
    i = i_center
    while 0 <= i < n:
        raw, d = readings[i]
        dv = _dval(d)
        seq.append((raw, dv))
        if i != i_center and dv > d_face + JUMP_MM:   # ressorti sur le fond
            break
        i += step

    if seq[-1][1] <= d_face + JUMP_MM:
        return None                # jamais ressorti : bord hors champ

    bg = seq[-1][1]
    thresh = (bg + d_face) / 2.0
    for k in range(1, len(seq)):
        raw_in, dv_in = seq[k - 1]
        raw_out, dv_out = seq[k]
        if dv_out >= thresh > dv_in:            # la distance franchit le seuil
            frac = (thresh - dv_in) / (dv_out - dv_in)
            return raw_in + frac * (raw_out - raw_in)
    return seq[-1][0]

def offset_deg(raw):
    return (raw - US_FORWARD) * DEG_PER_UNIT

def lateral(raw, d):
    """Position laterale (mm) du ping : >0 d'un cote, <0 de l'autre."""
    return d * math.sin(math.radians(offset_deg(raw)))

def report(segments, readings):
    if not segments:
        print("Aucun obstacle detecte.")
        return
    nearest = min(segments, key=lambda s: min(d for _, d in s['pts']))
    pts = nearest['pts']
    d_face = min(d for _, d in pts)

    # --- BORDS MI-MONTEE (methode retenue) ---
    raw_center = min(pts, key=lambda rd: rd[1])[0]
    index_of = {raw: i for i, (raw, _) in enumerate(readings)}
    i_center = index_of[raw_center]
    raw_lo = edge_raw(readings, i_center, d_face, -1)
    raw_hi = edge_raw(readings, i_center, d_face, +1)
    measurable = raw_lo is not None and raw_hi is not None

    # --- COMPARAISON : ancienne methode 1er/dernier echo sur la FACE ---
    face = [(raw, d) for raw, d in pts if d <= d_face + FACE_TOL]
    w_raw, w_corr, span_old = width_of(face)

    print("\n=================== OBSTACLE LE PLUS PROCHE ===================")
    print("  pings segment      : %d" % len(pts))
    print("  distance face      : %.0f mm  (point le plus proche)" % d_face)
    print("  ----------------------------------------------------------")
    if measurable:
        span_deg = abs(raw_hi - raw_lo) * DEG_PER_UNIT
        width_hr = 2.0 * d_face * math.sin(math.radians(span_deg) / 2.0)
        center_off = offset_deg((raw_lo + raw_hi) / 2.0)
        print("  bords mi-montee    : raw %.1f  /  raw %.1f" % (raw_lo, raw_hi))
        print("  angles bords       : %+.1f deg  /  %+.1f deg" %
              (offset_deg(raw_lo), offset_deg(raw_hi)))
        print("  span mi-montee     : %.1f deg" % span_deg)
        print("  centre             : %+.0f deg (0 = pile devant)" % center_off)
        print("  ----------------------------------------------------------")
        print("  LARGEUR (mi-montee): %.0f mm    <== mesure retenue" % width_hr)
        print("  comparaison 1er/dernier echo : brut %.0f mm / corr-faisc %.0f mm (span %.0f deg)" %
              (w_raw, w_corr, span_old))
    else:
        missing = []
        if raw_lo is None:
            missing.append("cote index bas (vers US_RIGHT)")
        if raw_hi is None:
            missing.append("cote index haut (vers US_LEFT)")
        xs = [lateral(raw, d) for raw, d in face]
        w_lat = (max(xs) - min(xs)) if xs else 0.0
        print("  /!\\ BORD(S) NON VU(S) : %s" % ", ".join(missing))
        print("      -> objet plus large que le champ balaye, ou face plate / mur")
        print("         (la distance grimpe hors-axe sans redescendre au fond).")
        print("      borne basse de largeur (vue) : >= %.0f mm" % w_lat)
    print("===============================================================")
    if len(segments) > 1:
        print("  (%d segments detectes au total ; on a pris le plus proche.)" % len(segments))

def main():
    print("Scan de l'obstacle... (robot immobile face a l'objet)")
    readings = scan()
    segments = segment(readings)
    report(segments, readings)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        set_angle(US_CH, US_FORWARD)
        print("\nInterrompu.")
