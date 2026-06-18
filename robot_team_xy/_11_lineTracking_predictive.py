import sys
import time
import select
from collections import deque
from threading import Event, Thread

from gpiozero import InputDevice

from _01_LedAvant import *
from _03_servo import *
from _04_motor import *
from _09_ObstacleDetect import *

line_pin_left = 22
line_pin_middle = 27
line_pin_right = 17

# Note: pin assignment swapped intentionally to match physical sensor layout
left = InputDevice(pin=line_pin_right)
middle = InputDevice(pin=line_pin_middle)
right = InputDevice(pin=line_pin_left)

channel = 0

# PD gains: enough authority to take turns, but less twitchy than the original.
Kp = 24
Kd = 5

SPEED_STRAIGHT = 95
SPEED_MIN_PREDICTIVE = 5
SPEED_GRACE = 42
SPEED_RECOVERY = 18

MAX_STEERING_DELTA = 38
GRACE_MAX_STEERING = 22
RECOVERY_ANGLE = 38

GRACE_PERIOD_TIME = 0.65
RECOVERY_BACKWARD_TIME = 0.55
RECOVERY_FORWARD_TIME = 0.30
RECOVERY_SETTLE_TIME = 0.05
LOST_LINE_CONFIRM_CYCLES = 5

SPEED_STEP_UP = 8
SPEED_STEP_DOWN = 14
HISTORY_LEN = 20
TURN_SLOWDOWN_START_PERCENT = 18
TURN_SLOWDOWN_EXPONENT = 1.65
RECOVERY_FORWARD_ANGLE_SCALE = 0.82
RECOVERY_SWITCH_SIDE_AFTER = 4

STOPPED = 0
RUNNING = 1
state = STOPPED

prev_error = 0.0
last_steering = 0.0
last_turn_dir = 0
lost_line_count = 0
recovery_phase = None
recovery_timer = 0.0
current_speed = 0
obstacle_thread = None
obstacle_stop_event = None
recovery_direction = 1
recovery_attempt = 0

error_history = deque(maxlen=HISTORY_LEN)
pattern_history = deque(maxlen=HISTORY_LEN)


def weighted_error(l, m, r):
    total = l + m + r
    if total == 0:
        return None
    return (-l + r) / total


def sensor_pattern(l, m, r):
    return (l << 2) | (m << 1) | r


def apply_steering(user_angle):
    clamped = max(
        CENTER_ANGLE - MAX_STEERING_DELTA,
        min(CENTER_ANGLE + MAX_STEERING_DELTA, user_angle),
    )
    set_angle(channel, to_servo_angle(clamped))


def set_drive_speed(target_speed, direction):
    global current_speed

    target_speed = max(0, min(100, int(target_speed)))
    step = SPEED_STEP_UP if target_speed >= current_speed else SPEED_STEP_DOWN

    if target_speed > current_speed:
        current_speed = min(target_speed, current_speed + step)
    else:
        current_speed = max(target_speed, current_speed - step)

    drive(current_speed, direction)


def clamp01(value):
    return max(0.0, min(1.0, value))


def predicted_turn_percent(current_error):
    if not error_history:
        return 0.0

    abs_now = abs(current_error)
    abs_mean = sum(abs(v) for v in error_history) / len(error_history)
    abs_peak = max(abs(v) for v in error_history)
    trend = abs(error_history[-1] - error_history[0]) if len(error_history) > 1 else 0.0

    left_hits = sum(1 for p in pattern_history if p in (0b100, 0b110))
    right_hits = sum(1 for p in pattern_history if p in (0b001, 0b011))
    side_bias_ratio = max(left_hits, right_hits) / max(1, len(pattern_history))

    # Score continu 0..1: plus il monte, plus le virage predit est exigeant.
    score = (
        0.40 * clamp01(abs_now)
        + 0.25 * clamp01(abs_mean)
        + 0.20 * clamp01(abs_peak)
        + 0.10 * clamp01(trend / 1.5)
        + 0.05 * clamp01(side_bias_ratio)
    )
    return 100.0 * clamp01(score)


def speed_for_turn_percent(turn_percent):
    if turn_percent <= TURN_SLOWDOWN_START_PERCENT:
        return SPEED_STRAIGHT

    turn_ratio = clamp01(
        (turn_percent - TURN_SLOWDOWN_START_PERCENT)
        / (100.0 - TURN_SLOWDOWN_START_PERCENT)
    )
    speed_span = SPEED_STRAIGHT - SPEED_MIN_PREDICTIVE

    # Zone franche en ligne droite, puis ralentissement progressif seulement
    # quand la prediction de virage devient significative.
    target_speed = SPEED_STRAIGHT - speed_span * (turn_ratio ** TURN_SLOWDOWN_EXPONENT)
    return int(round(max(SPEED_MIN_PREDICTIVE, min(SPEED_STRAIGHT, target_speed))))


def start_move():
    global state, obstacle_thread, obstacle_stop_event
    global prev_error, last_steering, last_turn_dir, lost_line_count
    global recovery_phase, recovery_timer, current_speed
    global recovery_direction, recovery_attempt

    obstacle_stop_event = Event()
    obstacle_thread = Thread(
        target=arretUrgence,
        args=(STOP_DIST, WARNING_DIST, obstacle_stop_event),
        daemon=True,
    )
    obstacle_thread.start()

    prev_error = 0.0
    last_steering = 0.0
    last_turn_dir = 0
    lost_line_count = 0
    recovery_phase = None
    recovery_timer = 0.0
    current_speed = 0
    recovery_direction = 1
    recovery_attempt = 0
    error_history.clear()
    pattern_history.clear()

    apply_steering(CENTER_ANGLE)
    state = RUNNING
    print("-> Suivi ligne predictif demarre")


def stop_robot(reason="manuel"):
    global state, obstacle_thread, obstacle_stop_event, current_speed
    global recovery_phase, recovery_attempt

    if obstacle_stop_event is not None:
        obstacle_stop_event.set()
    if obstacle_thread and obstacle_thread.is_alive():
        obstacle_thread.join(timeout=0.2)

    obstacle_thread = None
    obstacle_stop_event = None
    current_speed = 0
    recovery_phase = None
    recovery_attempt = 0
    stop()
    apply_steering(CENTER_ANGLE)
    state = STOPPED
    print("-> Arret (%s)" % reason)


def check_keyboard():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip().upper()
    return None


if __name__ == "__main__":
    setup()
    switchSetup()
    print("=== Tache 11 - Suivi de ligne predictif ===")
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
                pattern = sensor_pattern(l, m, r)
                error = weighted_error(l, m, r)

                if error is None:
                    lost_line_count += 1

                    if lost_line_count < LOST_LINE_CONFIRM_CYCLES:
                        held_steering = max(
                            -GRACE_MAX_STEERING,
                            min(GRACE_MAX_STEERING, last_steering),
                        )
                        apply_steering(CENTER_ANGLE + held_steering)
                        set_drive_speed(SPEED_GRACE, 1)
                        time.sleep(0.02)
                        continue

                    if recovery_phase is None:
                        recovery_phase = "GRACE"
                        recovery_timer = time.time()
                        recovery_direction = last_turn_dir or (1 if last_steering >= 0 else -1)
                        recovery_attempt = 0

                    if recovery_phase == "GRACE":
                        held_steering = max(
                            -GRACE_MAX_STEERING,
                            min(GRACE_MAX_STEERING, last_steering),
                        )
                        apply_steering(CENTER_ANGLE + held_steering)
                        set_drive_speed(SPEED_GRACE, 1)

                        if time.time() - recovery_timer > GRACE_PERIOD_TIME:
                            recovery_phase = "SETTLE_BACKWARD"
                            recovery_timer = time.time()

                    elif recovery_phase == "SETTLE_BACKWARD":
                        stop()
                        apply_steering(CENTER_ANGLE)

                        if time.time() - recovery_timer > RECOVERY_SETTLE_TIME:
                            recovery_phase = "BACKWARD"
                            recovery_timer = time.time()

                    elif recovery_phase == "BACKWARD":
                        recovery_angle = CENTER_ANGLE - RECOVERY_ANGLE * recovery_direction
                        apply_steering(recovery_angle)
                        set_drive_speed(SPEED_RECOVERY, -1)

                        if time.time() - recovery_timer > RECOVERY_BACKWARD_TIME:
                            recovery_phase = "SETTLE_FORWARD"
                            recovery_timer = time.time()

                    elif recovery_phase == "SETTLE_FORWARD":
                        stop()
                        apply_steering(CENTER_ANGLE)

                        if time.time() - recovery_timer > RECOVERY_SETTLE_TIME:
                            recovery_phase = "FORWARD"
                            recovery_timer = time.time()

                    elif recovery_phase == "FORWARD":
                        recovery_angle = CENTER_ANGLE + (
                            RECOVERY_ANGLE * RECOVERY_FORWARD_ANGLE_SCALE * recovery_direction
                        )
                        apply_steering(recovery_angle)
                        set_drive_speed(SPEED_GRACE, 1)

                        if time.time() - recovery_timer > RECOVERY_FORWARD_TIME:
                            recovery_attempt += 1
                            if recovery_attempt >= RECOVERY_SWITCH_SIDE_AFTER:
                                recovery_direction *= -1
                                recovery_attempt = 0
                            recovery_phase = "SETTLE_BACKWARD"
                            recovery_timer = time.time()

                else:
                    lost_line_count = 0
                    recovery_phase = None
                    recovery_attempt = 0

                    error_history.append(error)
                    pattern_history.append(pattern)

                    d_error = error - prev_error
                    steering = Kp * error + Kd * d_error
                    apply_steering(CENTER_ANGLE + steering)
                    last_steering = steering

                    if steering > 2:
                        last_turn_dir = 1
                    elif steering < -2:
                        last_turn_dir = -1

                    turn_percent = predicted_turn_percent(error)
                    target_speed = speed_for_turn_percent(turn_percent)
                    set_drive_speed(target_speed, 1)
                    prev_error = error

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nFin de programme par Ctrl-C")
    finally:
        stop()
        set_all_switch_off()
        print("Nettoyage final realise")
