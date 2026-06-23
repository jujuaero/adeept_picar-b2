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
    # 1. Convertir en niveaux de gris
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # 2. Binariser (Le noir devient blanc/actif)
    _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
    
    # 3. Trouver les contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        c = max(contours, key=cv2.contourArea)
        
        if cv2.contourArea(c) > 500:
            x, y, w, h = cv2.boundingRect(c)
            roi = thresh[y:y+h, x:x+w]
            
            left_half = roi[:, :w//2]
            right_half = roi[:, w//2:]
            
            left_pixels = cv2.countNonZero(left_half)
            right_pixels = cv2.countNonZero(right_half)
            
            if left_pixels > right_pixels:
                return "RIGHT"
            else:
                return "LEFT"
                
    return "UNKNOWN"

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