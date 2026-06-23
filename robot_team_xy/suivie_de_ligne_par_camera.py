import cv2
import numpy as np
import time, sys, select
from _04_motor import *
from _03_servo import *
from _09_ObstacleDetect import *
from _01_LedAvant import *
from threading import Event, Thread

channel = 0

# PD gains
Kp = 24
Kd = 5

# 1. VITESSES REDUITES POUR LE DEBUG
SPEED_STRAIGHT = 35   # % vitesse ligne droite (baisse)
SPEED_TURNING  = 25   # % vitesse en virage (baisse)
MAX_STEERING_DELTA = 36 

STOPPED = 0
RUNNING = 1
state = STOPPED

prev_error    = 0.0
last_steering = 0.0

obstacle_thread = None
obstacle_stop_event = None

vidcap = cv2.VideoCapture(0, cv2.CAP_V4L2)
vidcap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
vidcap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

def apply_steering(user_angle):
    clamped = max(
        CENTER_ANGLE - MAX_STEERING_DELTA,
        min(CENTER_ANGLE + MAX_STEERING_DELTA, user_angle),
    )
    set_angle(channel, to_servo_angle(clamped))

def start_move():
    global state, obstacle_thread, obstacle_stop_event
    global prev_error, last_steering

    obstacle_stop_event = Event()
    obstacle_thread = Thread(
        target=arretUrgence,
        args=(STOP_DIST, WARNING_DIST, obstacle_stop_event),
        daemon=True,
    )
    obstacle_thread.start()
    drive_ramp(SPEED_STRAIGHT, 1, RAMP_TIME)
    state = RUNNING
    prev_error = 0.0
    last_steering = 0.0
    print("-> Suivi ligne par camera demarre (Vitesse reduite)")

def stop_robot(reason="manuel"):
    global state, obstacle_thread, obstacle_stop_event

    if obstacle_stop_event is not None:
        obstacle_stop_event.set()
    if obstacle_thread and obstacle_thread.is_alive():
        obstacle_thread.join(timeout=0.2)
    obstacle_thread = None
    obstacle_stop_event = None
    stop()
    apply_steering(CENTER_ANGLE)
    state = STOPPED
    print("-> Arret (%s)" % reason)

def check_keyboard():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip().upper()
    return None

if __name__ == '__main__':
    setup()
    switchSetup()
    set_angle(2, to_servo_angle(-20))
    print("=== Debug Vision - Suivi de ligne ===")
    print("  M : demarrer")
    print("  A : arret")
    print("  Ctrl-C : quitter")

    try:
        if not vidcap.isOpened():
            print("Erreur: Impossible d'ouvrir la camera")
            sys.exit(1)

        while True:
            cmd = check_keyboard()
            if cmd == "M" and state == STOPPED:
                start_move()
            elif cmd == "A" and state != STOPPED:
                stop_robot(reason="manuel")

            ret, frame = vidcap.read()
            if not ret:
                continue

            height, width, _ = frame.shape
            
            # Definition de la zone de recherche (Region of Interest)
            roi_top = int(height * 2/3)
            roi = frame[roi_top:height, 0:width]
            
            # Dessiner un rectangle bleu pour visualiser la zone de recherche (ROI)
            cv2.rectangle(frame, (0, roi_top), (width, height), (255, 0, 0), 2)

            if state == RUNNING:
                if obstacle_thread and not obstacle_thread.is_alive():
                    stop_robot(reason="obstacle")
                    continue

                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                lower_red = np.array([0, 150, 150])
                upper_red = np.array([10, 255, 255])
                mask = cv2.inRange(hsv, lower_red, upper_red)

                M = cv2.moments(mask)
                
                if M['m00'] > 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    
                    center_img = width / 2
                    # 4. Calcul de l'erreur normalisee (INVERSEE pour corriger la direction)
                    error = (center_img - cx) / center_img
                    
                    d_error = error - prev_error
                    steering = Kp * error + Kd * d_error
                    
                    apply_steering(CENTER_ANGLE + steering)
                    last_steering = steering
                    prev_error = error

                    speed = SPEED_STRAIGHT if abs(steering) < 8 else SPEED_TURNING
                    drive(speed, 1)

                    # 2. AFFICHAGE DES DECISIONS SUR L'IMAGE
                    # Dessiner un cercle vert sur le centre detecte
                    cv2.circle(frame, (cx, roi_top + cy), 8, (0, 255, 0), -1)
                    # Afficher les valeurs mathematiques
                    cv2.putText(frame, f"Err: {error:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    cv2.putText(frame, f"Braq: {steering:.1f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                
                else:
                    apply_steering(CENTER_ANGLE + last_steering)
                    drive(SPEED_TURNING, 1)
                    # Afficher une alerte rouge si la ligne est perdue
                    cv2.putText(frame, "LIGNE PERDUE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # 3. AFFICHER LE RETOUR VIDEO EN DIRECT
            cv2.imshow("Debug Robot Vision", frame)
            
            # Necessaire pour que la fenetre cv2 s'actualise
            cv2.waitKey(1) 
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nFin de programme par Ctrl-C")
    finally:
        setup()
        stop()
        set_all_switch_off()
        vidcap.release()
        cv2.destroyAllWindows()
        print("Nettoyage final realise")