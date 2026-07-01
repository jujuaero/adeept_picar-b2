#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# TEST autonome : reperer un CYLINDRE (bouteille / gros cylindre) par fusion
# sonar + camera, robuste au fond charge. NE TOUCHE PAS a la mission.
#
# Idee :
#   - le SONAR donne la distance D (et confirme un vrai objet 3D ; la ligne au
#     sol, plate, ne renvoie pas d'echo -> ignoree).
#   - a distance D, un cylindre de diametre connu occupe une largeur en PIXELS
#     previsible. On cherche donc dans l'image une PAIRE de bords verticaux
#     espacee de cette largeur -> ca rejette le decor (mauvaise largeur) et le
#     fond lointain (on ne regarde que la bande BASSE = proche).
#   - la camera donne alors l'ANGLE precis (gauche/droite) que le sonar, avec son
#     cone de 60 deg, ne peut pas donner.
#
# Sortie : angles/ largeur estimee dans le terminal + 'cyl_debug.jpg' (annotee)
# et 'cyl_edges.jpg' (bords verticaux) a regarder pour regler.

# A REGLER (en haut) : HFOV_DEG, BAND_TOP_FRAC, DIAM_MIN/MAX_MM, seuils de bords.

import os
import sys
import time
import math
import numpy as np
import cv2
from picamera2 import Picamera2

# Les modules materiel (_03_servo, _05_ultrason) sont dans le dossier parent
# (robot_team_xy). On l'ajoute au chemin pour pouvoir lancer ce script depuis
# un sous-dossier (ex. maxime/).
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _03_servo import set_angle
from _05_ultrason import checkdist

# ----- a regler -----
IMG_W, IMG_H   = 640, 480
HFOV_DEG       = 62.0        # champ horizontal camera (Pi cam v2 ~62, v1 ~54) -> calibrer
BAND_TOP_FRAC  = 0.45        # on n'analyse que sous cette fraction de hauteur (le proche)
DIAM_MIN_MM    = 40.0        # plus petit cylindre attendu (bouteille fine)
DIAM_MAX_MM    = 220.0       # plus gros cylindre attendu
EDGE_BLUR      = 5           # flou avant Sobel (impair) : lisse le bruit
PEAK_MIN_FRAC  = 0.35        # un bord vertical doit valoir au moins ça * le plus fort
PEAK_MIN_SEP   = 8           # px : ecart mini entre deux pics distincts
SMOOTH_W       = 9           # px : lissage du profil de colonnes (impair)

US_CH, US_FORWARD    = 1, 100
VALID_MIN, VALID_MAX = 30, 1900


def capture():
    cam = Picamera2()
    cfg = cam.preview_configuration
    cfg.size = (IMG_W, IMG_H)
    cfg.format = 'RGB888'        # array en ordre BGR (compatible cv2)
    cam.configure("preview")
    cam.start()
    time.sleep(1.0)              # laisse expo / balance des blancs se stabiliser
    frame = cam.capture_array()
    cam.stop(); cam.close()
    return frame


def sonar_distance():
    set_angle(US_CH, US_FORWARD)
    time.sleep(0.1)
    vals = []
    for _ in range(5):
        d = checkdist()
        if d is not None and VALID_MIN <= d < VALID_MAX:
            vals.append(float(d))
        time.sleep(0.02)
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2]


def px_to_angle(px):
    """Colonne pixel -> angle horizontal (deg), + = droite."""
    return (px - IMG_W / 2.0) / IMG_W * HFOV_DEG


def vertical_edge_profile(frame):
    """Renvoie (profil, band_top, edges_vis).

    profil[x] = force des bords VERTICAUX de la colonne x, cumulee sur la bande
    basse de l'image (le proche). Un bord vertical = variation horizontale forte
    -> Sobel en x.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if EDGE_BLUR >= 3:
        gray = cv2.GaussianBlur(gray, (EDGE_BLUR, EDGE_BLUR), 0)
    sx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    mag = np.abs(sx)

    band_top = int(BAND_TOP_FRAC * IMG_H)
    band = mag[band_top:, :]
    profile = band.sum(axis=0)

    if SMOOTH_W >= 3:
        k = np.ones(SMOOTH_W, dtype=np.float32) / SMOOTH_W
        profile = np.convolve(profile, k, mode='same')

    edges_vis = np.zeros((IMG_H, IMG_W), np.uint8)
    m = mag / (mag.max() + 1e-6) * 255.0
    edges_vis[band_top:, :] = m[band_top:, :].astype(np.uint8)
    return profile, band_top, edges_vis


def find_peaks(profile):
    """Colonnes ou le profil fait un maximum local net -> bords verticaux candidats."""
    thr = PEAK_MIN_FRAC * float(profile.max() + 1e-6)
    peaks = []
    for x in range(1, len(profile) - 1):
        v = profile[x]
        if v >= thr and v >= profile[x - 1] and v >= profile[x + 1]:
            if peaks and x - peaks[-1][0] < PEAK_MIN_SEP:
                if v > peaks[-1][1]:         # garde le plus fort du groupe serre
                    peaks[-1] = (x, v)
            else:
                peaks.append((x, v))
    return peaks


def px_width_range(dist_mm):
    """Largeur en pixels attendue d'un cylindre de diametre [MIN..MAX] a distance D.
    Sans distance : plage large par defaut."""
    if dist_mm is None:
        return 20, int(0.9 * IMG_W)
    hfov = math.radians(HFOV_DEG)
    def px_of(diam):
        ang = 2.0 * math.atan((diam / 2.0) / dist_mm)   # largeur angulaire
        return ang / hfov * IMG_W
    lo = px_of(DIAM_MIN_MM)
    hi = px_of(DIAM_MAX_MM)
    return int(max(6, lo * 0.8)), int(hi * 1.25)


def best_pair(peaks, wpx_lo, wpx_hi):
    """Meilleure paire (gauche, droite) de bords espacee dans la plage de largeur
    attendue, maximisant la force cumulee des deux bords."""
    best, best_score = None, -1.0
    for i in range(len(peaks)):
        xl, vl = peaks[i]
        for j in range(i + 1, len(peaks)):
            xr, vr = peaks[j]
            gap = xr - xl
            if gap < wpx_lo or gap > wpx_hi:
                continue
            score = vl + vr
            if score > best_score:
                best_score, best = score, (xl, xr, vl, vr)
    return best


def main():
    print("Distance sonar + capture camera...")
    dist = sonar_distance()
    frame = capture()

    profile, band_top, edges_vis = vertical_edge_profile(frame)
    peaks = find_peaks(profile)
    wpx_lo, wpx_hi = px_width_range(dist)
    pair = best_pair(peaks, wpx_lo, wpx_hi)

    annotated = frame.copy()
    cv2.line(annotated, (0, band_top), (IMG_W, band_top), (120, 120, 120), 1)
    for x, _v in peaks:
        cv2.line(annotated, (x, band_top), (x, IMG_H), (0, 200, 255), 1)

    msg_d = "sonar: %.0f mm" % dist if dist else "sonar: (pas d'echo)"
    print("\n=================== CYLINDRE (fusion) ===================")
    print("  %s" % msg_d)
    print("  largeur attendue : %d..%d px" % (wpx_lo, wpx_hi))
    print("  bords candidats  : %d" % len(peaks))

    if pair is None:
        print("  -> aucune paire de bords a la bonne largeur.")
        print("     (regle BAND_TOP_FRAC / PEAK_MIN_FRAC / HFOV_DEG, regarde cyl_edges.jpg)")
    else:
        xl, xr, vl, vr = pair
        ang_l, ang_r = px_to_angle(xl), px_to_angle(xr)
        center = (ang_l + ang_r) / 2.0
        cv2.rectangle(annotated, (xl, band_top), (xr, IMG_H - 1), (0, 0, 255), 2)
        print("  bords retenus    : x=%d..%d px  ->  %+.1f .. %+.1f deg" % (xl, xr, ang_l, ang_r))
        print("  centre           : %+.1f deg (0 = pile devant)" % center)
        if dist:
            width = dist * (math.tan(math.radians(ang_r)) - math.tan(math.radians(ang_l)))
            print("  LARGEUR estimee  : %.0f mm" % width)
            cv2.putText(annotated, "%.0f mm  %+.0f deg" % (width, center),
                        (xl, max(20, band_top - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    print("=========================================================")

    cv2.imwrite("cyl_edges.jpg", edges_vis)
    cv2.imwrite("cyl_debug.jpg", annotated)
    print("\nImages : cyl_debug.jpg (annotee) + cyl_edges.jpg (bords verticaux).")
    print("Regarde-les devant un cylindre, puis on ajuste.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        set_angle(US_CH, US_FORWARD)
        print("\nInterrompu.")
