#Prototype Main
from suivie_de_ligne_par_camera import *
from _11_lineTracking import *
from _11_lineTracking_predictive import *
from _12_MissionBObstacle import *
from _12_obstacleZone import *
from labyrinthe import *
from reconnaissance_panneaux import *

if __name__ == "__main__":
    print("Starting Robot")
    print("Choosing Starting Envirronement" \
    "1. Line Tracking" \
    "2. Obstacle Avoidance" \
    "3. Camera Line Tracking" \
    "4. Labyrinthe" \
    "Crtl-C to Quit")
    cmd = input("Envirronement: ")
    pannel = None
    try :
        while True:
            if cmd == "1" :
                print("Starting Line Tracking")
                cmd = "wait"
            elif cmd == "2" or pannel == "travaux":
                print("Starting Obstacle Avoidance")
                cmd = "wait"
            elif cmd == "3" or pannel == "???":
                print("Starting Camera Line Tracking")
                cmd = "wait"
            elif cmd == "4" or pannel == "tunnel":
                print("Starting Labyrinthe")
                cmd = "wait"
    except KeyboardInterrupt:
        print("Exiting Robot")