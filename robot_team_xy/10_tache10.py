    # Tache 10 : Suivi de source lumineuse et obstacle

    import time
    import sys
    import select
    import smbus
    from board import SCL, SDA
    import busio
    from adafruit_pca9685 import PCA9685
    from adafruit_motor import motor, servo
    from gpiozero import DistanceSensor, LED, TonalBuzzer
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


    def steer(angle):
        servo_dir.angle = max(0, min(180, angle))


    def destroy_motor():
        stop()
        pwm.deinit()


    # --- Capteur ultrason ---
    sensor = DistanceSensor(echo=24, trigger=23, max_distance=2, queue_len=1)


    # --- Capteur de lumiere (ADS7830) ---
    class ADS7830:
        def __init__(self):
            self.cmd = 0x84
            self.bus = smbus.SMBus(1)
            self.address = 0x48
        def read(self, chn):
            return self.bus.read_byte_data(
                self.address, self.cmd | (((chn << 2 | chn >> 1) & 0x07) << 4)
            )

    adc = ADS7830()

    # --- Buzzer ---
    buzzer = TonalBuzzer(18)

    # --- LEDs WS2812 ---
    leds = Adeept_SPI_LedPixel(12, 60)

    # --- Phares ---
    phare_g = LED(9)
    phare_d = LED(25)

    # --- Parametres ---
    CENTER_ANGLE  = 97.5
    STEER_ANGLE   = 30    # amplitude de virage en degres
    SPEED         = 40    # % vitesse avant
    REVERSE_SPEED = 25    # % vitesse recul
    REVERSE_TIME  = 2.0   # duree recul en secondes (~30 cm)
    OBSTACLE_DIST = 20    # cm
    WARNING_DIST  = 40    # cm
    LIGHT_BASE    = 127   # valeur ADC neutre (milieu de plage 0-255)
    LIGHT_THRESH  = 15    # zone morte autour de la baseline

    # --- Etats ---
    STOPPED = 0
    RUNNING = 1

    state   = STOPPED
    _blink  = False
    _last_blink = 0.0
    _warned = False


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


    def _blink_red(duration):
        t_end = time.time() + duration
        blink = False
        t_last = 0.0
        while time.time() < t_end:
            now = time.time()
            if now - t_last >= 0.4:
                blink = not blink
                t_last = now
                if blink:
                    _set_leds(255, 0, 0, True)
                else:
                    _set_leds(0, 0, 0, False)
            time.sleep(0.02)


    def track_light():
        val = adc.read(1)
        if val < LIGHT_BASE - LIGHT_THRESH:
            steer(CENTER_ANGLE + STEER_ANGLE)
        elif val > LIGHT_BASE + LIGHT_THRESH:
            steer(CENTER_ANGLE - STEER_ANGLE)
        else:
            steer(CENTER_ANGLE)
        drive(SPEED, 1)


    def obstacle_recovery():
        global state, _warned

        print("-> Feux de detresse (1s)")
        _blink_red(1.0)

        # Recul avec bip bip
        print("-> Recul ~30 cm avec bip bip")
        steer(CENTER_ANGLE)
        drive(REVERSE_SPEED, -1)
        t_end = time.time() + REVERSE_TIME
        bip = False
        t_bip = time.time()
        while time.time() < t_end:
            _set_leds(255, 0, 0, True)
            if time.time() - t_bip >= 0.5:
                bip = not bip
                t_bip = time.time()
                buzzer.play("A4") if bip else buzzer.stop()
            time.sleep(0.05)

        stop()
        buzzer.stop()
        _set_leds(0, 0, 0, False)

        # Pause 2 secondes
        print("-> Pause 2s")
        time.sleep(2.0)

        # Reprise
        _warned = False
        _set_leds(0, 255, 0, False)
        state = RUNNING
        print("-> Reprise suivi lumiere")


    def stop_robot(reason="manuel"):
        global state
        stop()
        steer(CENTER_ANGLE)
        state = STOPPED
        _set_leds(0, 0, 0, False)
        print("-> Arret (%s)" % reason)


    def start_move():
        global state, _warned
        _warned = False
        _set_leds(0, 255, 0, False)
        state = RUNNING
        print("-> Suivi lumiere demarre")


    def read_cmd():
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.readline().strip()
        return None


    def destroy():
        destroy_motor()
        buzzer.stop()
        _set_leds(0, 0, 0, False)
        sensor.close()


    if __name__ == "__main__":
        setup()
        leds.daemon = True
        leds.start()

        print("=== Tache 10 - Suivi lumiere et obstacle ===")
        print("  M : demarrer")
        print("  A : arret immediat")
        print("  Ctrl-C : quitter")
        print()

        try:
            while True:
                cmd = read_cmd()

                if cmd in ("M", "m"):
                    if state == STOPPED:
                        start_move()

                elif cmd in ("A", "a"):
                    if state != STOPPED:
                        stop_robot(reason="manuel")

                if state == RUNNING:
                    dist = checkdist()

                    if dist < OBSTACLE_DIST:
                        stop()
                        print("-> STOP  : obstacle a %.1f cm" % dist)
                        obstacle_recovery()

                    elif dist < WARNING_DIST and not _warned:
                        _warned = True
                        _set_leds(255, 80, 0, True)
                        print("-> ALERTE: obstacle a %.1f cm" % dist)

                    elif dist >= WARNING_DIST and _warned:
                        _warned = False
                        _set_leds(0, 255, 0, False)

                    if state == RUNNING:
                        track_light()

                time.sleep(0.02)

        except KeyboardInterrupt:
            print("\nFin de programme par Ctrl-C")
        finally:
            destroy()
            print("Nettoyage final realise")
