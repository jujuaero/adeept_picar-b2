# Tache 9 : Marche avant et arret si obstacle

import time
import sys
import select
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor, servo
from gpiozero import DistanceSensor, LED
from spi_ws2812 import Adeept_SPI_LedPixel

# --- Moteurs ---
M1_IN1 = 15
M1_IN2 = 14
M2_IN1 = 12
M2_IN2 = 13

pwm       = None
motor1    = None
motor2    = None
servo_dir = None


def setup():
    global pwm, motor1, motor2, servo_dir
    i2c = busio.I2C(SCL, SDA)
    pwm = PCA9685(i2c, address=0x5f)
    pwm.frequency = 50
    motor1 = motor.DCMotor(pwm.channels[M1_IN1], pwm.channels[M1_IN2])
    motor1.decay_mode = motor.SLOW_DECAY
    motor2 = motor.DCMotor(pwm.channels[M2_IN1], pwm.channels[M2_IN2])
    motor2.decay_mode = motor.SLOW_DECAY
    servo_dir = servo.Servo(pwm.channels[0], min_pulse=500, max_pulse=2400, actuation_range=180)
    servo_dir.angle = CENTER_ANGLE


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
        if checkdist() < OBSTACLE_DIST:
            stop()
            return False
        drive(speed_pct * i / steps, direction)
        time.sleep(delay)
    return True


def destroy_motor():
    stop()
    pwm.deinit()


# --- Capteur ultrason ---
sensor = DistanceSensor(echo=24, trigger=23, max_distance=2, queue_len=1)

# --- LEDs WS2812 ---
leds = Adeept_SPI_LedPixel(12, 60)

# --- Phares ---
phare_g = LED(9)
phare_d = LED(25)

# --- Parametres ---
CENTER_ANGLE  = 97.5
SPEED         = 40   # % vitesse (reduite pour les tests)
OBSTACLE_DIST = 20   # cm
WARNING_DIST  = 40   # cm - seuil d'alerte avant arret
RAMP_TIME     = 0.5  # secondes

# --- Etats ---
STOPPED  = 0
RUNNING  = 1
OBSTACLE = 2

state         = STOPPED
_blink        = False
_last_blink   = 0.0
_warned       = False


def checkdist():
    d = sensor.distance
    if d is None:
        return 999
    return d * 100


def _set_leds(r, g, b, phares_on=False):
    leds.set_all_led_color(r, g, b)
    if phares_on:
        phare_g.on()
        phare_d.on()
    else:
        phare_g.off()
        phare_d.off()


def update_blink():
    global _blink, _last_blink
    now = time.time()
    if now - _last_blink >= 0.4:
        _blink = not _blink
        _last_blink = now
        if _blink:
            _set_leds(255, 0, 0, True)   # rouge + phares
        else:
            _set_leds(0, 0, 0, False)


def start_move():
    global state, _warned
    _warned = False
    _set_leds(0, 255, 0, False)   # vert au depart
    state = RUNNING
    if not drive_ramp(SPEED, 1, ramp_time=RAMP_TIME):
        obstacle_stop()
    else:
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
    time.sleep(0.15)
    stable = 0
    prev = checkdist()
    while stable < 3:
        time.sleep(0.05)
        curr = checkdist()
        if abs(curr - prev) < 0.08:
            stable += 1
        else:
            stable = 0
        prev = curr
    print("-> ARRET  : distance finale %.1f cm" % curr)
    print("-> OBSTACLE detecte ! Feux de detresse actives")


def read_cmd():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip()
    return None


def destroy():
    destroy_motor()
    _set_leds(0, 0, 0, False)
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
                    print("-> STOP  : obstacle a %.1f cm" % dist)
                    obstacle_stop()
                elif dist < WARNING_DIST and not _warned:
                    _warned = True
                    _set_leds(255, 80, 0, True)    # orange + phares
                    print("-> ALERTE: obstacle a %.1f cm" % dist)
                elif dist >= WARNING_DIST and _warned:
                    _warned = False
                    _set_leds(0, 255, 0, False)    # retour vert

            elif state == OBSTACLE:
                update_blink()

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nFin de programme par Ctrl-C")
    finally:
        destroy()
        print("Nettoyage final realise")
