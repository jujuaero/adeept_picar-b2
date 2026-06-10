import RPi.GPIO as GPIO
import time

def switchSetup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    # 3 LED BASIQUE
    GPIO.setup(9, GPIO.OUT)
    GPIO.setup(25, GPIO.OUT)
    GPIO.setup(11, GPIO.OUT)
    # 6 LED RGB
    # Gauche
    GPIO.setup(0, GPIO.OUT)
    GPIO.setup(19, GPIO.OUT)
    GPIO.setup(13, GPIO.OUT)
    # Droite
    GPIO.setup(1, GPIO.OUT)
    GPIO.setup(5, GPIO.OUT)
    GPIO.setup(6, GPIO.OUT)


LEDS = {
    1: (9, False),
    2: (25, False),
    3: (11, False),
    4: (13, True),
    5: (19, True),
    6: (0, True),
    7: (1, True),
    8: (5, True),
    9: (6, True)
    }


def light_on(LED):
    if LED[1]:
        GPIO.output(LED[0], GPIO.LOW)
    else:
        GPIO.output(LED[0], GPIO.HIGH)


def light_off(LED):
    if LED[1]:
        GPIO.output(LED[0], GPIO.HIGH)
    else:
        GPIO.output(LED[0], GPIO.LOW)


def set_all_switch_off():
    for i in LEDS:
        light_off(LEDS[i])

#preconstruct led patterns :

def warning():
    set_all_switch_off()
    for i in range(10):
        light_on(LEDS[4])
        light_on(LEDS[7])
        time.sleep(0.2)
        light_off(LEDS[4])
        light_off(LEDS[7])
        time.sleep(0.2)
    set_all_switch_off()

def police():
    set_all_switch_off()
    for i in range(10):
        light_on(LEDS[4])
        light_off(LEDS[9])
        time.sleep(0.5)
        light_off(LEDS[4])
        light_on(LEDS[9])
        time.sleep(0.5)
    set_all_switch_off()

def phare():
    set_all_switch_off()
    light_on(LEDS[4])
    light_on(LEDS[5])
    light_on(LEDS[8])
    light_on(LEDS[7])

if __name__ == "__main__":
    cmd = 0
    switchSetup()
    set_all_switch_off()
    while cmd >= 0:
        cmd = int(input("Choose between 11 and 29 or -1 to quit: "))
        if 11 <= cmd <= 19:
            light_on(LEDS[cmd - 10])
        if 21 <= cmd <= 29:
            light_off(LEDS[cmd - 20])
        if cmd == 20:
            set_all_switch_off()
        if cmd == 1:
            phare()
        if cmd == 2:
            police()
        if cmd == 3:
            warning()
        if cmd == -1:
            set_all_switch_off()
        
            

        
        
