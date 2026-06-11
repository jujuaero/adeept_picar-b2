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

# PD gains — si le robot oscille: baisser Kd; si trop lent a reagir: monter Kp
Kp = 30
Kd = 8

SPEED_STRAIGHT = 35   # % vitesse ligne droite
SPEED_TURNING  = 25   # % vitesse en virage
SPEED_RECOVERY = 18   # % vitesse quand la ligne est perdue
RECOVERY_ANGLE = 40   # degres de braquage lors de la recuperation

STOPPED = 0
RUNNING = 1
state = STOPPED

prev_error    = 0.0
last_turn_dir = 0   # +1 = dernier virage a droite, -1 = gauche

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
    set_angle(channel, to_servo_angle(clamped))


def start_move():
    global state, obstacle_thread
    obstacle_thread = Thread(target=arretUrgence, args=(STOP_DIST, WARNING_DIST), daemon=True)
    obstacle_thread.start()
    drive_ramp(SPEED_STRAIGHT, 1, RAMP_TIME)
    state = RUNNING
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
                    # Ligne perdue : on ralentit et on braque fort dans le dernier sens connu
                    drive(SPEED_RECOVERY, 1)
                    recovery_angle = CENTER_ANGLE + RECOVERY_ANGLE * (last_turn_dir or 1)
                    apply_steering(recovery_angle)
                else:
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