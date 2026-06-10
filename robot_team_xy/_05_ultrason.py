from gpiozero import DistanceSensor
from time import sleep

Tr = 23
Ec = 24
sensor = DistanceSensor(echo=Ec, trigger=Tr,max_distance=2) 
distance = 0.0

# Get the distance of ultrasonic detection.
def checkdist():
    global distance 
    distance = (sensor.distance) *1000 # Unit: mm
    

if __name__ == "__main__":
    while True:
        checkdist() 
        print("%.2f mm" %distance)
        sleep(0.05)