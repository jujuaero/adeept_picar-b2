import time, sys, select
from _04_motor import *
from _03_servo import *
from _09_ObstacleDetect import *
from _01_LedAvant import *
from threading import Thread
from gpiozero import InputDevice

line_pin_left   = 22
line_pin_middle = 27
line_pin_right  = 17

# Note: pin assignment swapped intentionally to match physical sensor layout
left   = InputDevice(pin=line_pin_right)
middle = InputDevice(pin=line_pin_middle)
right  = InputDevice(pin=line_pin_left)

channel = 0

# PD gains - si le robot oscille: baisser Kd; si trop lent a reagir: monter Kp
Kp = 30
Kd = 8

SPEED_STRAIGHT = 45   # % vitesse ligne droite
SPEED_TURNING  = 35   # % vitesse en virage
SPEED_RECOVERY = 30   # % vitesse quand la ligne est perdue
RECOVERY_ANGLE = 40   # degres de braquage lors de la recuperation

# Temps de manoeuvre (en secondes) pour la recuperation de ligne
RECOVERY_BACKWARD_TIME = 0.8   # Temps de recul initial
RECOVERY_FORWARD_TIME  = 0.5   # Temps pour la marche avant de recuperation
RECOVERY_EXTRA_TIME    = 0.0   # Temps supplementaire de recul une fois la ligne retrouvee

STOPPED = 0
RUNNING = 1
state = STOPPED

prev_error    = 0.0
last_turn_dir = 0   # +1 = dernier virage a droite, -1 = gauche

# Variables pour la recuperation de ligne
recovery_phase = None
recovery_timer = 0.0

obstacle_thread = None


def weighted_error(l, m, r):
    """Erreur de position normalisee : -1 (ligne a gauche) a +1 (ligne a droite).
    Retourne None si la ligne est completement perdue."""
    total = l + m + r
    if total == 0:
        return None
    return (-l + r) / total


def apply_steering(user_angle):
    clamped = max(-45, min(50, user_angle))
    set_angle(1, to_servo_angle(clamped))
    set_angle(channel, to_servo_angle(clamped))


def start_move():
    global state, obstacle_thread, recovery_phase
    obstacle_thread = Thread(target=arretUrgence, args=(STOP_DIST, WARNING_DIST), daemon=True)
    obstacle_thread.start()
    drive_ramp(SPEED_STRAIGHT, 1, RAMP_TIME)
    state = RUNNING
    recovery_phase = None
    print("-> Suivi ligne demarre")


def stop_robot(reason="manuel"):
    global state
    stop()
    apply_steering(CENTER_ANGLE)
    state = STOPPED
    print("-> Arret (%s)" % reason)


def check_keyboard():
    """Lecture clavier non-bloquante."""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip().upper()
    return None


if __name__ == '__main__':
    setup()
    switchSetup()
    print("=== Tache 11 - Suivi de ligne ===")
    print("  M : demarrer")
    print("  A : arret")
    print("  Ctrl-C : quitter")

    try:
        while True:
            cmd = check_keyboard()
            if cmd == "M" and state == STOPPED:
                start_move()
            elif cmd == "A" and state != STOPPED:
                stop_robot(reason="manuel")

            if state == RUNNING:
                if obstacle_thread and not obstacle_thread.is_alive():
                    stop_robot(reason="obstacle")
                    continue

                l, m, r = left.value, middle.value, right.value
                error = weighted_error(l, m, r)

                if error is None:
                    # Ligne perdue : alternance de recul et d'avance
                    if recovery_phase == 'BACKWARD':
                        # Direction -1 pour reculer
                        drive(SPEED_RECOVERY, -1) 
                        # Braque dans le sens INVERSE du dernier virage
                        recovery_angle = CENTER_ANGLE - RECOVERY_ANGLE * (last_turn_dir or 1)
                        apply_steering(recovery_angle)
                        
                        # Change de direction apres le temps defini
                        if time.time() - recovery_timer > RECOVERY_BACKWARD_TIME: 
                            recovery_phase = 'FORWARD'
                            recovery_timer = time.time()
                            
                    elif recovery_phase == 'FORWARD':
                        # Direction 1 pour avancer
                        drive(SPEED_RECOVERY, 1) 
                        # Braque dans le sens NORMAL pour retrouver la ligne
                        recovery_angle = CENTER_ANGLE + RECOVERY_ANGLE * (last_turn_dir or 1)
                        apply_steering(recovery_angle)
                        
                        # Change de direction apres le temps defini
                        if time.time() - recovery_timer > RECOVERY_FORWARD_TIME:
                            recovery_phase = 'BACKWARD'
                            recovery_timer = time.time()
                            
                    else:
                        # Initialisation de la phase de recuperation
                        # (Gere aussi le cas ou il reperd la ligne pendant l'extra)
                        recovery_phase = 'BACKWARD'
                        recovery_timer = time.time()
                else:
                    # Ligne detectee
                    if recovery_phase == 'BACKWARD':
                        # Ligne retrouvee pendant le recul : on amorce le temps supplementaire
                        recovery_phase = 'BACKWARD_EXTRA'
                        recovery_timer = time.time()
                        
                    if recovery_phase == 'BACKWARD_EXTRA':
                        # On continue le recul avec le meme braquage
                        drive(SPEED_RECOVERY, -1)
                        recovery_angle = CENTER_ANGLE - RECOVERY_ANGLE * (last_turn_dir or 1)
                        apply_steering(recovery_angle)
                        
                        # Fin du temps supplementaire
                        if time.time() - recovery_timer > RECOVERY_EXTRA_TIME:
                            recovery_phase = None
                            
                    else:
                        # Ligne retrouvee en marche avant ou suivi normal
                        recovery_phase = None 
                        
                        # Controleur PD : proportionnel + derive pour amortir les oscillations
                        d_error  = error - prev_error
                        steering = Kp * error + Kd * d_error
                        apply_steering(CENTER_ANGLE + steering)

                        if steering > 2:
                            last_turn_dir = 1
                        elif steering < -2:
                            last_turn_dir = -1

                        speed = SPEED_STRAIGHT if abs(steering) < 8 else SPEED_TURNING
                        drive(speed, 1)
                        prev_error = error

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nFin de programme par Ctrl-C")
    finally:
        setup()
        stop()
        set_all_switch_off()
        print("Nettoyage final realise")