import cv2
import numpy as np
import os, time, sys
from collections import Counter

# Import des modules Adeept
from _01_LedAvant import *
from _04_motor import *
from _03_servo import *
from _05_ultrason import *

# --- CANAUX SERVOS ---
STEER_CH = 0      # Direction (plage -45..50)
HEAD_PAN_CH = 1   # Tete gauche/droite (le sonar est monte dessus)
HEAD_TILT_CH = 2  # Tete haut/bas

# Convention verifiee au rapporteur (etalonnage 2026-06-29) :
# +offset = GAUCHE, -offset = DROITE (pour les roues ET la tete).

# --- PARAMETRES DE NAVIGATION ---
SPEED_FORWARD = 20
DISTANCE_STOP = 30.0  # cm
TURN_TIME = 1.5       # Temps estime pour accomplir le virage a 90 degres

# --- PARAMETRES DETECTION FLECHE ---
# Fusion de deux approches : filtres de forme + vote multi-frames (version
# locale) et redressement minAreaRect de la fleche (version 7c95315a).
ROI_MARGIN_X = 0.15      # bande ignoree sur les bords (fraction de l'image)
ROI_MARGIN_Y = 0.15
ARROW_MIN_AREA_FRAC = 0.02   # taille du contour, en fraction de la ROI
ARROW_MAX_AREA_FRAC = 0.70
ARROW_MIN_ASPECT = 1.1       # apres redressement la fleche est horizontale
ARROW_MAX_ASPECT = 4.0       # (largeur/hauteur de la bounding box)
ARROW_MIN_SOLIDITY = 0.35    # aire/aire de l'enveloppe convexe
ARROW_MAX_SOLIDITY = 0.92    # un rectangle plein ~1.0 -> rejete
ARROW_MIN_VERTICES = 5       # une fleche approximee ~7 sommets
ARROW_MAX_VERTICES = 10
ARROW_MIN_OFFSET = 0.10      # decalage tete/centre bbox minimal (norme)
ARROW_MIN_PEAK_RATIO = 1.5   # hauteur de la tete / hauteur mediane (tige)
ARROW_FRAMES_PER_LOOK = 5    # frames analysees par tentative
ARROW_VOTES_NEEDED = 3       # majorite requise parmi ces frames

# La partie la plus HAUTE d'une fleche (colonne de pixels la plus
# remplie) est sa tete triangulaire : elle indique la direction.
# Mettre ARROW_SIGN = -1 pour inverser si besoin apres test terrain.
ARROW_SIGN = +1

# --- ETATS DU ROBOT ---
STATE_DRIVE = "DRIVE"
STATE_STOP_LOOK = "STOP_AND_LOOK"
STATE_MANEUVER = "MANEUVER"

# Affichage debug seulement si un ecran est disponible (sinon cv2.imshow
# plante en SSH). Forcable avec LABY_DEBUG=1 ou LABY_DEBUG=0.
_dbg = os.environ.get("LABY_DEBUG")
if _dbg is not None:
    SHOW_DEBUG = _dbg not in ("0", "false", "no")
else:
    SHOW_DEBUG = bool(os.environ.get("DISPLAY")) or os.name == "nt"

# Initialisation de la camera
vidcap = cv2.VideoCapture(0, cv2.CAP_V4L2)
vidcap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
vidcap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
vidcap.set(cv2.CAP_PROP_BUFFERSIZE, 1)


def apply_steering(angle):
    set_angle(STEER_CH, to_servo_angle(angle))


def set_head_pan(angle):
    set_angle(HEAD_PAN_CH, to_servo_angle(angle))


def flush_camera(n=4):
    # V4L2 bufferise plusieurs images : sans purge, on analyserait une
    # frame datant du moment ou le robot roulait encore (floue).
    for _ in range(n):
        vidcap.grab()


def get_distance_cm(samples=3):
    # Mediane de quelques lectures pour lisser le bruit du sonar
    vals = []
    for _ in range(samples):
        vals.append(checkdist() / 10.0)
        if samples > 1:
            time.sleep(0.03)
    return float(np.median(vals))


def detect_arrow_direction(frame, debug_frame=None):
    """Retourne 'LEFT', 'RIGHT' ou 'UNKNOWN'.

    Principe : fleche sombre sur fond clair. On binarise (Otsu), on filtre
    les contours par taille/forme, on REDRESSE la forme via minAreaRect
    (annule l'inclinaison de la fleche / le roulis camera, idee reprise
    de la version 7c95315a), puis on cherche la colonne la plus remplie
    du contour redresse : c'est la tete triangulaire (la partie la plus
    large d'une fleche), et son cote donne la direction.
    """
    h, w = frame.shape[:2]
    x0, y0 = int(w * ROI_MARGIN_X), int(h * ROI_MARGIN_Y)
    x1, y1 = int(w * (1 - ROI_MARGIN_X)), int(h * (1 - ROI_MARGIN_Y))
    roi = frame[y0:y1, x0:x1]
    roi_area = roi.shape[0] * roi.shape[1]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    # Otsu : seuil auto, robuste aux variations d'eclairage
    _, thresh = cv2.threshold(gray, 0, 255,
                              cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    # Ouverture morphologique pour supprimer le bruit poivre-et-sel
    kernel = np.ones((3, 3), np.uint8)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    # On examine les plus gros contours en premier
    for c in sorted(contours, key=cv2.contourArea, reverse=True)[:3]:
        area = cv2.contourArea(c)
        if area < ARROW_MIN_AREA_FRAC * roi_area:
            break  # les suivants sont encore plus petits
        if area > ARROW_MAX_AREA_FRAC * roi_area:
            continue  # trop gros : mur sombre, ombre globale...

        # Filtres invariants par rotation, AVANT redressement
        hull = cv2.convexHull(c)
        hull_area = cv2.contourArea(hull)
        if hull_area <= 0:
            continue
        solidity = area / hull_area
        if not (ARROW_MIN_SOLIDITY <= solidity <= ARROW_MAX_SOLIDITY):
            continue

        approx = cv2.approxPolyDP(c, 0.02 * cv2.arcLength(c, True), True)
        if not (ARROW_MIN_VERTICES <= len(approx) <= ARROW_MAX_VERTICES):
            continue

        # Redressement : on tourne le masque du contour pour mettre la
        # fleche a l'horizontale (tolere fleches inclinees / roulis camera)
        mask = np.zeros(thresh.shape, np.uint8)
        cv2.drawContours(mask, [c], -1, 255, -1)
        (rcx, rcy), (rw, rh), rangle = cv2.minAreaRect(c)
        if rw < rh:
            rangle += 90
        rot = cv2.getRotationMatrix2D((rcx, rcy), rangle, 1.0)
        mask = cv2.warpAffine(mask, rot, (mask.shape[1], mask.shape[0]))

        sub_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
        if not sub_contours:
            continue
        sc = max(sub_contours, key=cv2.contourArea)
        bx, by, bw, bh = cv2.boundingRect(sc)
        if bh == 0 or bw < bh:
            continue  # une fois redressee, la fleche doit etre horizontale
        aspect = bw / float(bh)
        if not (ARROW_MIN_ASPECT <= aspect <= ARROW_MAX_ASPECT):
            continue

        # Profil vertical : nombre de pixels actifs par colonne du masque
        # redresse ; le pic est la tete triangulaire
        cols = (mask[by:by + bh, bx:bx + bw] > 0).sum(axis=0).astype(float)
        # petit lissage pour tolerer le bruit
        k = max(3, bw // 20) | 1
        cols = np.convolve(cols, np.ones(k) / k, mode="same")

        median_h = float(np.median(cols[cols > 0]))
        if median_h <= 0 or cols.max() < ARROW_MIN_PEAK_RATIO * median_h:
            continue  # pas de "tete" nettement plus large que la tige

        # Position du pic (la tete) par rapport au centre : > 0 = a droite
        peak_x = int(np.argmax(cols))
        offset = (peak_x - bw / 2.0) / bw
        if abs(offset) < ARROW_MIN_OFFSET:
            continue  # tete trop centree pour trancher (losange, tache...)

        direction = "RIGHT" if offset * ARROW_SIGN > 0 else "LEFT"

        if debug_frame is not None:
            # bbox d'origine (avant redressement) pour l'affichage
            obx, oby, obw, obh = cv2.boundingRect(c)
            cv2.rectangle(debug_frame, (x0 + obx, y0 + oby),
                          (x0 + obx + obw, y0 + oby + obh), (0, 255, 0), 2)
            cv2.putText(debug_frame,
                        f"{direction} off={offset:+.2f} ang={rangle:.0f}",
                        (x0 + obx, y0 + oby - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        return direction

    if debug_frame is not None:
        cv2.rectangle(debug_frame, (x0, y0), (x1, y1), (0, 0, 255), 1)
    return "UNKNOWN"


def look_for_arrow():
    """Analyse plusieurs frames et vote. Retourne LEFT/RIGHT/UNKNOWN."""
    flush_camera()
    votes = []
    for _ in range(ARROW_FRAMES_PER_LOOK):
        ret, frame = vidcap.read()
        if not ret:
            continue
        d = detect_arrow_direction(frame)
        if d != "UNKNOWN":
            votes.append(d)
        time.sleep(0.05)
    if votes:
        winner, count = Counter(votes).most_common(1)[0]
        if count >= ARROW_VOTES_NEEDED:
            return winner
    return "UNKNOWN"


def execute_90_degree_turn(direction):
    print(f"-> Virage fluide vers : {direction}")

    steps = 15  # Nombre de micro-mouvements pour fluidifier le retour de la tete
    step_time = TURN_TIME / steps

    if direction == "RIGHT":
        steering_angle = CENTER_ANGLE - 40
        head_start_angle = CENTER_ANGLE - 40
    else:  # LEFT
        steering_angle = CENTER_ANGLE + 40
        head_start_angle = CENTER_ANGLE + 40

    # 1. Tourner la tete pour regarder le futur chemin
    set_head_pan(head_start_angle)
    time.sleep(0.3)

    # 2. Braquer les roues et lancer les moteurs
    apply_steering(steering_angle)
    drive(20, 1)

    # 3. Boucle de virage : on ramene la tete au centre pendant le temps imparti
    for i in range(steps):
        progress = (i + 1) / steps
        # Interpolation lineaire vers CENTER_ANGLE (bug corrige : l'ancienne
        # formule start + (start - CENTER) * progress ELOIGNAIT la tete du centre)
        current_head_angle = head_start_angle + \
            (CENTER_ANGLE - head_start_angle) * progress
        set_head_pan(current_head_angle)
        time.sleep(step_time)

    # 4. Fin du virage
    stop()
    apply_steering(CENTER_ANGLE)
    set_head_pan(CENTER_ANGLE)
    print("-> Fin de la manoeuvre")


def show_debug(frame, state, dist):
    global SHOW_DEBUG
    if not SHOW_DEBUG:
        return False
    try:
        cv2.putText(frame, f"Etat: {state}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, f"Dist: {dist:.1f} cm", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow("Debug Labyrinthe", frame)
        return cv2.waitKey(1) & 0xFF == ord('q')
    except cv2.error:
        print("(pas d'affichage disponible, debug desactive)")
        SHOW_DEBUG = False
        return False


if __name__ == '__main__':
    setup()
    switchSetup()

    # Camera de niveau (Tilt) et tete droite (Pan)
    set_angle(HEAD_TILT_CH, to_servo_angle(0))
    set_head_pan(CENTER_ANGLE)

    current_state = STATE_DRIVE
    direction = "UNKNOWN"

    print("=== Navigation Labyrinthe Continue ===")
    print(f"(debug video: {'ON' if SHOW_DEBUG else 'OFF'}, "
          f"forcer avec LABY_DEBUG=0/1)")

    try:
        if not vidcap.isOpened():
            print("Erreur Camera")
            sys.exit(1)
        flush_camera(8)  # warm-up : l'expo auto se stabilise

        while True:
            ret, frame = vidcap.read()
            if not ret:
                continue

            dist = get_distance_cm(1)

            # --- MACHINE D'ETAT ---
            if current_state == STATE_DRIVE:
                if dist > DISTANCE_STOP:
                    apply_steering(CENTER_ANGLE)
                    drive(SPEED_FORWARD, 1)
                else:
                    stop()
                    print(f"-> Mur a {dist:.1f} cm. Arret et analyse...")
                    current_state = STATE_STOP_LOOK
                    time.sleep(0.5)

            elif current_state == STATE_STOP_LOOK:
                direction = look_for_arrow()

                if direction != "UNKNOWN":
                    print(f"-> Fleche detectee : {direction}")
                    current_state = STATE_MANEUVER
                else:
                    print("-> Aucune fleche lisible. Ajustement...")
                    drive(35, -1)
                    time.sleep(0.3)
                    stop()

            elif current_state == STATE_MANEUVER:
                execute_90_degree_turn(direction)
                flush_camera()
                current_state = STATE_DRIVE

            # --- AFFICHAGE DEBUG ---
            if current_state == STATE_STOP_LOOK:
                detect_arrow_direction(frame, debug_frame=frame)
            if show_debug(frame, current_state, dist):
                break

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nFin Ctrl-C")
    finally:
        stop()
        set_all_switch_off()
        set_head_pan(CENTER_ANGLE)
        set_angle(HEAD_TILT_CH, to_servo_angle(CENTER_ANGLE))
        vidcap.release()
        cv2.destroyAllWindows()
