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
    try :
        while True:
            cmd = check_keyboard()
            if cmd == "1":
                print("Starting Line Tracking")
            elif cmd == "2":
                print("Starting Obstacle Avoidance")
            elif cmd == "3":
                print("Starting Camera Line Tracking")
            elif cmd == "4":
                print("Starting Labyrinthe")
    except KeyboardInterrupt:
        print("Exiting Robot")