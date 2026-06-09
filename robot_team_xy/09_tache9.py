# Tache 9 : Marche avant et arret si obstacle

import time
import sys
import select
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor
from gpiozero import DistanceSensor, LED
from spi_ws2812 import Adeept_SPI_LedPixel

# --- Moteurs ---
M1_IN1 = 15
M1_IN2 = 14
M2_IN1 = 12
M2_IN2 = 13

pwm    = None
motor1 = None
motor2 = None


def setup():
    global pwm, motor1, motor2
    i2c = busio.I2C(SCL, SDA)
    pwm = PCA9685(i2c, address=0x5f)
    pwm.frequency = 50
    motor1 = motor.DCMotor(pwm.channels[M1_IN1], pwm.channels[M1_IN2])
    motor1.decay_mode = motor.SLOW_DECAY
    motor2 = motor.DCMotor(pwm.channels[M2_IN1], pwm.channels[M2_IN2])
    motor2.decay_mode = motor.SLOW_DECAY


def stop():
    motor1.throttle = 0
    motor2.throttle = 0


def drive(speed_pct, direction):
    if direction == 0:
        stop()
        return
    throttle = (max(0, min(100, speed_pct)) / 100.0) * direction
    motor1.throttle =  throttle
    motor2.throttle = -throttle


def drive_ramp(speed_pct, direction, ramp_time=1.0):
    steps = 50
    delay = ramp_time / steps
    for i in range(1, steps + 1):
        drive(speed_pct * i / steps, direction)
        time.sleep(delay)


def destroy_motor():
    stop()
    pwm.deinit()


# --- Capteur ultrason ---
sensor = DistanceSensor(echo=24, trigger=23, max_distance=2)

# --- LEDs WS2812 ---
leds = Adeept_SPI_LedPixel(8, 60)

# --- Phares ---
phare_g = LED(9)
phare_d = LED(25)

# --- Parametres ---
CENTER_ANGLE  = 97.5
SPEED         = 40   # % vitesse (reduite pour les tests)
OBSTACLE_DIST = 20   # cm
RAMP_TIME     = 0.5  # secondes

# --- Etats ---
STOPPED  = 0
RUNNING  = 1
OBSTACLE = 2

state       = STOPPED
_blink      = False
_last_blink = 0.0


def checkdist():
    return sensor.distance * 100


def _set_hazard(on):
    if on:
        leds.set_all_led_color(255, 80, 0)
        phare_g.on()
        phare_d.on()
    else:
        leds.set_all_led_color(0, 0, 0)
        phare_g.off()
        phare_d.off()


def update_blink():
    global _blink, _last_blink
    now = time.time()
    if now - _last_blink >= 0.4:
        _blink = not _blink
        _last_blink = now
        _set_hazard(_blink)


def start_move():
    global state
    _set_hazard(False)
    drive_ramp(SPEED, 1, ramp_time=RAMP_TIME)
    state = RUNNING
    print("-> Marche avant %d%%" % SPEED)


def stop_robot(reason="manuel"):
    global state
    stop()
    state = STOPPED
    print("-> Arret (%s)" % reason)


def obstacle_stop():
    global state, _last_blink
    stop()
    state = OBSTACLE
    _last_blink = 0.0
    print("-> OBSTACLE detecte ! Feux de detresse actives")


def read_cmd():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip()
    return None


def destroy():
    destroy_motor()
    _set_hazard(False)
    sensor.close()


if __name__ == "__main__":
    setup()
    leds.daemon = True
    leds.start()

    print("=== Tache 9 - Marche avant et arret obstacle ===")
    print("  M : demarrer en marche avant")
    print("  A : arret immediat")
    print("  Ctrl-C : quitter")
    print()

    try:
        while True:
            cmd = read_cmd()

            if cmd in ("M", "m"):
                if state in (STOPPED, OBSTACLE):
                    start_move()

            elif cmd in ("A", "a"):
                if state != STOPPED:
                    stop_robot(reason="manuel")

            if state == RUNNING:
                dist = checkdist()
                if dist < OBSTACLE_DIST:
                    obstacle_stop()

            elif state == OBSTACLE:
                update_blink()

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nFin de programme par Ctrl-C")
    finally:
        destroy()
        print("Nettoyage final realise")
