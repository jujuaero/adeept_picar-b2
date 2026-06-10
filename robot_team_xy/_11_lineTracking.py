import time
import argparse
from _04_motor import *
from _03_servo import *
from _09_ObstacleDetect import arretUrgence
from _01_LedAvant import *
import threading
from gpiozero import InputDevice

line_pin_left = 22
line_pin_middle = 27
line_pin_right = 17

left = InputDevice(pin=line_pin_right)
middle = InputDevice(pin=line_pin_middle)
right = InputDevice(pin=line_pin_left)
last_turn_angle = CENTER_ANGLE

def run():
    status_right = right.value
    status_middle = middle.value
    status_left = left.value
    return f"{status_left}{status_middle}{status_right}"

def angle(string):
    global last_turn_angle
    angle = {
        "000": CENTER_ANGLE,
        "001": -20,
        "010": CENTER_ANGLE,
        "011": -40,
        "100": 25,
        "101": CENTER_ANGLE,
        "110": 45,
        "111": CENTER_ANGLE
    }
    if angle[string] in ["001", "011", "100", "110"]:
        last_turn_angle = angle[string]
    return angle[string]

def execute_recovery():
    # On réduit la vitesse pour diminuer le rayon de braquage physique du robot
    drive(15, 1)

    # On braque au maximum selon le dernier sens enregistré
    if last_turn_angle > 0:
        return 45  # Braquage maximal vers un côté
    else:
        return -40 # Braquage maximal vers l'autre côté


if __name__ == '__main__':
    setup()
    switchSetup()
    print("=== Tache 9 - Marche avant et arret obstacle ===")
    print("  M : demarrer en marche avant")
    print("  A : arret immediat")
    print("  Ctrl-C : quitter")
    print()
    cmd = input("Commande : ").strip().upper()
    try:
      ultra=threading.Thread(target=arretUrgence, args=(400, 600), daemon=True)
      while True:
        set_angle(0, to_servo_angle(angle(run())))
        if cmd == "M":
            ultra.start()
            threading.Thread(target=police, daemon=True).start()
            cmd="waiting"
            drive_ramp(25, 1, 1)
        if not ultra.is_alive():
            cmd = "stoped"
            cmd = input("Commande : ").strip().upper()
    except KeyboardInterrupt:
        print("\nFin de programme par Ctrl-C")
    finally:
        stop()
        set_all_switch_off()
        print("Nettoyage final realise")
