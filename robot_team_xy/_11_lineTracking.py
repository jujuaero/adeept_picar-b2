import time, sys, select
from _04_motor import *
from _03_servo import *
from _09_ObstacleDetect import *
from _01_LedAvant import *
from threading import Event, Thread
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
Kp = 18
Kd = 4

SPEED_STRAIGHT = 40   # % vitesse ligne droite
SPEED_TURNING  = 25   # % vitesse en virage
SPEED_RECOVERY = 20   # % vitesse quand la ligne est perdue
RECOVERY_ANGLE = 28   # degres de braquage lors de la recuperation
MAX_STEERING_DELTA = 28  # limite de braquage autour du centre

# Temps de manoeuvre (en secondes) pour la recuperation de ligne
GRACE_PERIOD_TIME      = 0.12  # Temps pour franchir un petit trou sans panique
RECOVERY_BACKWARD_TIME = 0.5   # Temps de recul initial
RECOVERY_FORWARD_TIME  = 0.15  # Temps pour la marche avant de recuperation
RECOVERY_EXTRA_TIME    = 0.05  # Temps supplementaire de recul une fois la ligne retrouvee
LOST_LINE_CONFIRM_CYCLES = 3   # Nombre de lectures consecutives perdues avant recovery

STOPPED = 0
RUNNING = 1
state = STOPPED

prev_error    = 0.0
last_turn_dir = 0   # +1 = dernier virage a droite, -1 = gauche

# Variables pour la recuperation de ligne
recovery_phase = None
recovery_timer = 0.0
lost_line_count = 0

obstacle_thread = None
obstacle_stop_event = None


def weighted_error(l, m, r):
    """Erreur de position normalisee : -1 (ligne a gauche) a +1 (ligne a droite).
    Retourne None si la ligne est completement perdue."""
    total = l + m + r
    if total == 0:
        return None
    return (-l + r) / total


def apply_steering(user_angle):
    clamped = max(
        CENTER_ANGLE - MAX_STEERING_DELTA,
        min(CENTER_ANGLE + MAX_STEERING_DELTA, user_angle),
    )
    set_angle(channel, to_servo_angle(clamped))


def start_move():
    global state, obstacle_thread, obstacle_stop_event
    global prev_error, last_turn_dir, recovery_phase, recovery_timer, lost_line_count

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
    last_turn_dir = 0
    recovery_phase = None
    recovery_timer = 0.0
    lost_line_count = 0
    print("-> Suivi ligne demarre")


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
                    lost_line_count += 1

                    if lost_line_count < LOST_LINE_CONFIRM_CYCLES:
                        # Petit trou de ligne: on garde une marche avant douce
                        drive(SPEED_TURNING, 1)
                        continue

                    # Initialisation de la phase de grace a la perte de la ligne
                    if recovery_phase is None:
                        recovery_phase = 'GRACE'
                        recovery_timer = time.time()

                    # Phase 1: Tolerance pour les lignes discontinues et angles droits
                    if recovery_phase == 'GRACE':
                        drive(SPEED_RECOVERY, 1)
                        # On force le virage dans le dernier sens connu a l'aveugle
                        grace_angle = CENTER_ANGLE + RECOVERY_ANGLE * (last_turn_dir or 1)
                        apply_steering(grace_angle)

                        if time.time() - recovery_timer > GRACE_PERIOD_TIME:
                            # Echec de la recuperation rapide, on passe au recul
                            recovery_phase = 'BACKWARD'
                            recovery_timer = time.time()

                    # Phase 2: Recul
                    elif recovery_phase == 'BACKWARD':
                        drive(SPEED_RECOVERY, -1) 
                        recovery_angle = CENTER_ANGLE - RECOVERY_ANGLE * (last_turn_dir or 1)
                        apply_steering(recovery_angle)
                        
                        if time.time() - recovery_timer > RECOVERY_BACKWARD_TIME: 
                            recovery_phase = 'FORWARD'
                            recovery_timer = time.time()
                            
                    # Phase 3: Avance
                    elif recovery_phase == 'FORWARD':
                        drive(SPEED_RECOVERY, 1) 
                        recovery_angle = CENTER_ANGLE + RECOVERY_ANGLE * (last_turn_dir or 1)
                        apply_steering(recovery_angle)
                        
                        if time.time() - recovery_timer > RECOVERY_FORWARD_TIME:
                            # On recommence la boucle de recul si toujours perdu
                            recovery_phase = 'BACKWARD'
                            recovery_timer = time.time()
                            
                else:
                    lost_line_count = 0
                    # Ligne detectee
                    if recovery_phase == 'BACKWARD':
                        # Ligne retrouvee pendant le recul : on amorce le temps supplementaire
                        recovery_phase = 'BACKWARD_EXTRA'
                        recovery_timer = time.time()
                        
                    elif recovery_phase == 'BACKWARD_EXTRA':
                        # On continue le recul avec le meme braquage
                        drive(SPEED_RECOVERY, -1)
                        recovery_angle = CENTER_ANGLE - RECOVERY_ANGLE * (last_turn_dir or 1)
                        apply_steering(recovery_angle)
                        
                        # Fin du temps supplementaire
                        if time.time() - recovery_timer > RECOVERY_EXTRA_TIME:
                            recovery_phase = None
                            
                    else:
                        # Ligne retrouvee en marche avant (y compris pendant la phase de GRACE)
                        recovery_phase = None 
                        
                        # Controleur PD : proportionnel + derive pour amortir les oscillations
                        d_error  = error - prev_error
                        steering = Kp * error + Kd * d_error
                        apply_steering(CENTER_ANGLE + steering)

                        # On enregistre la direction pour anticiper les pertes de ligne
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
