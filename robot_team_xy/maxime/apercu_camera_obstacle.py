#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# APERCU (a regler) : largeur d'obstacle par fusion sonar + camera.
#
# Principe (cf. discussion) :
#   - le SONAR donne la distance (et confirme que c'est un vrai objet 3D ;
#     une ligne noire plate au sol ne renvoie pas d'echo, donc ignoree)
#   - la CAMERA raffine les BORDS gauche/droite (resolution angulaire fine)
#   - segmentation par le SOL (on modelise le sol, pas l'obstacle, pour gerer
#     "tout et n'importe quoi"). Filtre de forme pour rejeter une ligne au sol.
#
# Sortie : largeur estimee (mm) + image annotee 'obstacle_debug.jpg' et masque
# 'obstacle_mask.jpg' a regarder pour juger/regler la segmentation.
#
# A REGLER (en haut) : HFOV_DEG (champ de vision camera), FLOOR_TOL (tolerance
# couleur sol), tailles morpho / filtres de forme.

import time
import math
import numpy as np
import cv2
from picamera2 import Picamera2
from _03_servo import set_angle
from _05_ultrason import checkdist

# ----- a regler -----
IMG_W, IMG_H = 640, 480
HFOV_DEG     = 62.0      # champ de vision horizontal (Pi cam v2 ~62, v1 ~54) -> a calibrer
FLOOR_TOL_H  = 12        # tolerance teinte (HSV H) autour du sol
FLOOR_TOL_S  = 60        # tolerance saturation
FLOOR_TOL_V  = 60        # tolerance valeur
MIN_AREA     = 1500      # px^2 : ignore les petits blobs (bruit)
MIN_H_FRAC   = 0.15      # un obstacle doit faire au moins 15% de la hauteur image
MAX_WH_RATIO = 6.0       # rejette les bandes horizontales (ligne au sol) trop larges/plates

US_CH, US_FORWARD = 1, 100
VALID_MIN, VALID_MAX = 30, 1900

def capture():
    cam = Picamera2()
    cfg = cam.preview_configuration
    cfg.size = (IMG_W, IMG_H)
    cfg.format = 'RGB888'      # array renvoye en ordre BGR (compatible cv2)
    cam.configure("preview")
    cam.start()
    time.sleep(1.0)            # laisse l'expo / balance des blancs se stabiliser
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

def floor_model(hsv):
    """Echantillonne le sol dans 2 coins bas (souvent du sol meme si un obstacle
    est centre) -> renvoie (low, high) HSV pour cv2.inRange."""
    h, w = hsv.shape[:2]
    patches = [hsv[h-40:h, 0:80], hsv[h-40:h, w-80:w]]
    samples = np.concatenate([p.reshape(-1, 3) for p in patches], axis=0)
    med = np.median(samples, axis=0)
    low = np.array([max(0, med[0]-FLOOR_TOL_H), max(0, med[1]-FLOOR_TOL_S), max(0, med[2]-FLOOR_TOL_V)])
    high = np.array([min(179, med[0]+FLOOR_TOL_H), 255, 255])
    return low.astype(np.uint8), high.astype(np.uint8), med

def find_obstacle(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    low, high, med = floor_model(hsv)
    floor = cv2.inRange(hsv, low, high)
    nonfloor = cv2.bitwise_not(floor)
    k = np.ones((5, 5), np.uint8)
    nonfloor = cv2.morphologyEx(nonfloor, cv2.MORPH_OPEN, k)
    nonfloor = cv2.morphologyEx(nonfloor, cv2.MORPH_CLOSE, k)

    cnts, _ = cv2.findContours(nonfloor, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for c in cnts:
        x, y, w, h = cv2.boundingRect(c)
        area = cv2.contourArea(c)
        touches_bottom = (y + h) >= IMG_H - 8
        tall_enough = h >= MIN_H_FRAC * IMG_H
        not_a_line = (w / max(1, h)) <= MAX_WH_RATIO     # rejette bande horizontale
        if area >= MIN_AREA and touches_bottom and tall_enough and not_a_line:
            if best is None or area > best[4]:
                best = (x, y, w, h, area)
    return best, nonfloor, med

def px_to_angle(px):
    """Colonne pixel -> angle horizontal (deg) depuis l'axe optique (+ = droite)."""
    return (px - IMG_W / 2.0) / IMG_W * HFOV_DEG

def main():
    print("Lecture sonar + capture camera...")
    dist = sonar_distance()
    frame = capture()
    best, mask, med = find_obstacle(frame)

    cv2.imwrite("obstacle_mask.jpg", mask)
    annotated = frame.copy()

    if best is None:
        print("Aucun obstacle segmente (regle FLOOR_TOL / eclairage ?).")
        print("Sol HSV median echantillonne :", med)
    else:
        x, y, w, h, area = best
        ang_l = px_to_angle(x)
        ang_r = px_to_angle(x + w)
        cv2.rectangle(annotated, (x, y), (x+w, y+h), (0, 0, 255), 2)
        msg_d = "sonar: %.0f mm" % dist if dist else "sonar: (pas d'echo)"
        print("\n=================== OBSTACLE (fusion) ===================")
        print("  %s" % msg_d)
        print("  bords image    : x=%d..%d px  ->  %.1f .. %.1f deg" % (x, x+w, ang_l, ang_r))
        if dist:
            # largeur d'un objet fronto-parallele a distance d : x_lat = d*tan(angle)
            width = dist * (math.tan(math.radians(ang_r)) - math.tan(math.radians(ang_l)))
            print("  LARGEUR estimee: %.0f mm  (camera bords + distance sonar)" % width)
            cv2.putText(annotated, "%.0f mm" % width, (x, max(20, y-8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            print("  (pas de distance sonar -> largeur angulaire seulement : %.1f deg)"
                  % (ang_r - ang_l))
        print("=========================================================")

    cv2.imwrite("obstacle_debug.jpg", annotated)
    print("\nImages ecrites : obstacle_debug.jpg (annotee) + obstacle_mask.jpg (masque).")
    print("Regarde-les pour juger la segmentation, puis on ajuste les seuils.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        set_angle(US_CH, US_FORWARD)
        print("\nInterrompu.")
