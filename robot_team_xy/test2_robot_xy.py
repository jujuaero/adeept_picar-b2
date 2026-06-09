import RPi.GPIO as GPIO
import time

def switchSetup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    # LED standards
    GPIO.setup(9, GPIO.OUT)    # LED1
    GPIO.setup(25, GPIO.OUT)   # LED2
    GPIO.setup(11, GPIO.OUT)   # LED3

    # LED RGB avant
    GPIO.setup(0, GPIO.OUT)    # left_R
    GPIO.setup(19, GPIO.OUT)   # left_G
    GPIO.setup(13, GPIO.OUT)   # left_B
    GPIO.setup(1, GPIO.OUT)    # right_R
    GPIO.setup(5, GPIO.OUT)    # right_G
    GPIO.setup(6, GPIO.OUT)    # right_B

    set_all_switch_off()

def switch(port, status):
    # LED standards : 1 = ON, 0 = OFF
    if port == 1:
        GPIO.output(9, GPIO.HIGH if status == 1 else GPIO.LOW)
    elif port == 2:
        GPIO.output(25, GPIO.HIGH if status == 1 else GPIO.LOW)
    elif port == 3:
        GPIO.output(11, GPIO.HIGH if status == 1 else GPIO.LOW)

    # LED RGB : logique inverse, 0 = ON, 1 = OFF
    elif port == 4:
        GPIO.output(0, GPIO.LOW if status == 1 else GPIO.HIGH)
    elif port == 5:
        GPIO.output(19, GPIO.LOW if status == 1 else GPIO.HIGH)
    elif port == 6:
        GPIO.output(13, GPIO.LOW if status == 1 else GPIO.HIGH)
    elif port == 7:
        GPIO.output(1, GPIO.LOW if status == 1 else GPIO.HIGH)
    elif port == 8:
        GPIO.output(5, GPIO.LOW if status == 1 else GPIO.HIGH)
    elif port == 9:
        GPIO.output(6, GPIO.LOW if status == 1 else GPIO.HIGH)
    else:
        print("Wrong Command")

def set_all_switch_off():
    for port in range(1, 10):
        switch(port, 0)

if __name__ == "__main__":
    switchSetup()

    try:
        while True:
            cmd = input("Commande : ")

            if cmd == "q":
                break

            if not cmd.isdigit():
                print("Commande invalide")
                continue

            cmd = int(cmd)

            if 11 <= cmd <= 19:
                switch(cmd - 10, 1)
                print("LED ON")

            elif 21 <= cmd <= 29:
                switch(cmd - 20, 0)
                print("LED OFF")

            elif cmd == 0:
                set_all_switch_off()
                print("All OFF")

            else:
                print("Commande invalide")

    except KeyboardInterrupt:
        print("Stop")

    finally:
        set_all_switch_off()
        GPIO.cleanup()