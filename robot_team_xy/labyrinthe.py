import cv2
import numpy as np
import time, sys, select

# Import des modules Adeept
from _01_LedAvant import *
from _04_motor import *
from _03_servo import *
from _05_ultrason import * 
channel = 0
head_pan_channel = 1  # Canal pour le mouvement gauche/droite de la tete

# --- PARAMETRES DE NAVIGATION ---
SPEED_FORWARD = 20
DISTANCE_STOP = 30.0  # cm
TURN_TIME = 1.5       # Temps estime pour accomplir le virage a 90 degres

# --- ETATS DU ROBOT ---
STATE_DRIVE = "DRIVE"
STATE_STOP_LOOK = "STOP_AND_LOOK"
STATE_MANEUVER = "MANEUVER"

current_state = STATE_DRIVE

# Initialisation de la camera
vidcap = cv2.VideoCapture(0, cv2.CAP_V4L2)
vidcap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
vidcap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

def apply_steering(angle):
    set_angle(channel, to_servo_angle(angle))

def detect_arrow_direction(frame):
    threshold = 1000

    # 1. Pretraitement pour isoler la forme blanche
    if len(frame.shape) == 3:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        gray = frame.copy()

    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, mask = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < threshold:
        return None

    # 2. Redresser numeriquement la fleche (annule le roulis de la camera)
    rect = cv2.minAreaRect(contour)
    (center_x, center_y), (width, height), angle = rect

    if width < height:
        angle += 90

    rot_matrix = cv2.getRotationMatrix2D((center_x, center_y), angle, 1.0)
    h_img, w_img = mask.shape[:2]
    straightened_mask = cv2.warpAffine(mask, rot_matrix, (w_img, h_img), flags=cv2.INTER_LINEAR)

    # 3. Recuperer la nouvelle forme redressee
    new_contours, _ = cv2.findContours(straightened_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not new_contours:
        return None

    clean_contour = max(new_contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(clean_contour)

    # Protection contre les artefacts
    if w == 0 or h == 0 or w < h:
        return None

    # 4. Test des tranches (15% a chaque extremite)
    slice_width = max(1, int(w * 0.15))
    left_slice = straightened_mask[y:y+h, x:x+slice_width]
    right_slice = straightened_mask[y:y+h, (x+w-slice_width):x+w]

    left_pixels = cv2.countNonZero(left_slice)
    right_pixels = cv2.countNonZero(right_slice)

    if left_pixels == 0 and right_pixels == 0:
        return None

    # La pointe a moins de pixels que la queue
    return "RIGHT" if right_pixels < left_pixels else "LEFT"

def execute_90_degree_turn(direction):
    print(f"-> Virage fluide vers : {direction}")
    
    steps = 15 # Nombre de micro-mouvements pour fluidifier le retour de la tete
    step_time = TURN_TIME / steps
    
    if direction == "RIGHT":
        steering_angle = CENTER_ANGLE - 40
        # Tete braquee a droite (environ 70 degres d'amplitude max pour eviter de forcer)
        head_start_angle = CENTER_ANGLE - 40 
    elif direction == "LEFT":
        steering_angle = CENTER_ANGLE + 40
        # Tete braquee a gauche
        head_start_angle = CENTER_ANGLE + 40 
        
    # 1. Tourner la tete a 90 degres pour regarder le futur chemin
    set_angle(head_pan_channel, to_servo_angle(head_start_angle))
    time.sleep(0.3) # Laisse le temps au servo de se mettre en position
    
    # 2. Braquer les roues et lancer les moteurs
    apply_steering(steering_angle)
    drive(20, 1)
    
    # 3. Boucle de virage : on ramene la tete au centre pendant le temps imparti
    for i in range(steps):
        # Calcul du pourcentage d'avancement du virage (de 0.0 a 1.0)
        progress = (i + 1) / steps
        # Interpolation lineaire pour ramener doucement l'angle vers CENTER_ANGLE
        current_head_angle = head_start_angle + (head_start_angle - CENTER_ANGLE) * progress
        
        set_angle(head_pan_channel, to_servo_angle(current_head_angle))
        time.sleep(step_time)
    
    # 4. Fin du virage
    stop()
    apply_steering(CENTER_ANGLE)
    set_angle(head_pan_channel, to_servo_angle(CENTER_ANGLE)) # Securite
    print("-> Fin de la manoeuvre")

if __name__ == '__main__':
    setup()
    switchSetup()
    
    # Incliner la camera vers le bas pour le sol/mur (Canal 2 = Tilt)
    set_angle(2, to_servo_angle(0))
    # S'assurer que la tete est bien droite (Canal 1 = Pan)
    set_angle(head_pan_channel, to_servo_angle(CENTER_ANGLE))
    
    print("=== Navigation Labyrinthe Continue ===")
    
    try:
        if not vidcap.isOpened():
            print("Erreur Camera")
            sys.exit(1)

        while True:
            ret, frame = vidcap.read()
            if not ret:
                continue

            dist = checkdist()/10
            
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
                direction = detect_arrow_direction(frame)
                
                if direction is not None:
                    print(f"-> Fleche detectee : {direction}")
                    current_state = STATE_MANEUVER
                else:
                    print("-> Aucune fleche lisible. Ajustement...")
                    drive(35, -1)
                    time.sleep(0.3)
                    stop()
                    
            elif current_state == STATE_MANEUVER:
                execute_90_degree_turn(direction)
                current_state = STATE_DRIVE
                
            # --- AFFICHAGE DEBUG ---
            cv2.putText(frame, f"Etat: {current_state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f"Dist: {dist:.1f} cm", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow("Debug Labyrinthe", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nFin Ctrl-C")
    finally:
        stop()
        set_all_switch_off()
        # Remettre la tete a zero avant de quitter
        set_angle(head_pan_channel, to_servo_angle(CENTER_ANGLE))
        set_angle(2, to_servo_angle(CENTER_ANGLE))
        vidcap.release()
        cv2.destroyAllWindows()