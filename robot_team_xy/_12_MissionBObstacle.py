#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Zone obstacle : navigation libre + panel de vision 2D (pygame)

import sys
import select
import time
import math
from threading import Thread, Lock

try:
    import pygame
    import pygame.gfxdraw
    PYGAME_OK = True
except ImportError:
    PYGAME_OK = False

from gpiozero import InputDevice
from _03_servo import set_angle, to_servo_angle
from _04_motor import setup, stop, drive, CENTER_ANGLE
from _05_ultrason import checkdist

# ================================================================ capteurs ligne
line_pin_left   = 22
line_pin_middle = 27
line_pin_right  = 17

left_s   = InputDevice(pin=line_pin_right)
middle_s = InputDevice(pin=line_pin_middle)
right_s  = InputDevice(pin=line_pin_left)

# ================================================================ constantes
STEER_CH = 0
US_CH    = 1

US_FORWARD = 100
US_RIGHT   = 58
US_LEFT    = 142
US_SETTLE  = 0.12

HEAD_AMPLITUDE = 18
HEAD_PERIOD    = 3.0

STEER_CENTER = CENTER_ANGLE
STEER_AMOUNT = 38
STEER_LEFT   = STEER_CENTER - STEER_AMOUNT
STEER_RIGHT  = STEER_CENTER + STEER_AMOUNT

SPEED_CRUISE  = 35
SPEED_REVERSE = 38
SPEED_TURN    = 30

OBSTACLE_DIST = 420
OBSTACLE_WARN_DIST = 900
VIEW_MAX_MM = 2000
VIEW_MIN_ZOOM = 0.50
VIEW_MAX_ZOOM = 4.00

ROBOT_WIDTH_MM  = 100.0
ROBOT_LENGTH_MM = 250.0
ROBOT_HALF_WIDTH_MM = ROBOT_WIDTH_MM / 2.0
IR_SENSOR_FWD_MM = 90.0
IR_SENSOR_X_MM   = (-45.0, 0.0, 45.0)
IR_LINE_TIMEOUT  = 45.0
IR_LINE_MAX      = 700

OBSTACLE_RADIUS_MM     = 60.0
AVOID_CLEARANCE_MM     = 80.0
AVOID_PATH_HALF_MM     = ROBOT_HALF_WIDTH_MM + OBSTACLE_RADIUS_MM + AVOID_CLEARANCE_MM
AVOID_LOOKAHEAD_MM     = 900.0
AVOID_SIDE_TARGET_MM   = AVOID_PATH_HALF_MM + 130.0
AVOID_CRITICAL_Y_MM    = 260.0
AVOID_MIN_CONF         = 0.30
SPEED_AVOID_MIN        = 22
SPEED_PROFILE_MM_S     = (
    (30.0, 210.0),                  # mesures 1s avec accel/freinage inclus
    (40.0, 290.0),
    (50.0, 340.0),
)
SPEED_MM_PER_SEC_AT_35 = 250.0      # interpolation de la table ci-dessus
STEER_MAX_WHEEL_DEG    = 26.0
STEER_SLEW_STEP        = 6.0

T_REVERSE = 0.55
T_TURN    = 0.50

STOPPED = 0
RUNNING = 1
_exit_flag = False

# ================================================================ etat partage
_lock = Lock()
_state = {
    'mode':     'ARRET',
    'l': 0, 'm': 0, 'r': 0,
    'us_angle': US_FORWARD,
    'us_dist':  2000,
    'speed':    0,
    'steer':    STEER_CENTER,
    'target_steer': STEER_CENTER,
    'avoid_dir': 0,
    'sim_scroll': 0.0,
    'sim_lateral': 0.0,
}

robot_state = STOPPED
head_t0     = 0.0


def _upd(key, val):
    with _lock:
        _state[key] = val

def _snap():
    with _lock:
        return dict(_state)

def _raw_to_rad(raw):
    deg_from_fwd = (raw - US_FORWARD) * (45.0 / 42.0)
    return math.radians(-90 - deg_from_fwd)

def _clamp(v, lo, hi):
    return max(lo, min(hi, v))

class WorldModel:
    """Carte d'objets persistante, centree sur le robot.

    - un ping qui touche cree ou renforce un objet (information positive)
    - un ping qui traverse une zone supposee occupee affaiblit l'objet :
      il a disparu (information negative / espace libre)
    - les objets non revus depuis longtemps s'effacent

    Repere robot : x = lateral (+ = droite), y = avant (+ = devant), en mm.
    Un objet peu confiant est "potentiel", tres confiant il est "confirme".
    """
    GATE_MM     = 170.0              # rayon d'association ping <-> objet
    BEAM_HALF   = math.radians(12)   # demi-ouverture du faisceau US
    MAP_MAX     = 2000.0             # mm : portee max capteur / carte
    FREE_MARGIN = 90.0               # mm de marge avant de liberer
    CONF_HIT    = 0.35
    CONF_MISS   = 0.30
    CONF_MAX    = 1.5
    CONF_OK     = 0.80               # seuil potentiel -> confirme
    CONF_MIN    = 0.12               # en dessous : oubli
    TIMEOUT     = 14.0               # s sans aucune mise a jour devant
    TRAIL_TIMEOUT = 60.0             # s : objets gardes derriere pour la vue
    HIT_TIMEOUT = 30.0               # s : pings bruts gardes pour la vue
    HIT_MAX     = 600
    ALPHA       = 0.28               # lissage de position

    def __init__(self):
        self._lock = Lock()
        self._objs = []              # liste de dict(x, y, conf, last)
        self._hits = []              # pings bruts, non fusionnes
        self._line = []              # points de ligne noire vus par les IR

    def clear(self):
        with self._lock:
            self._objs = []
            self._hits = []
            self._line = []

    @staticmethod
    def _angdiff(a, b):
        return (a - b + math.pi) % (2 * math.pi) - math.pi

    def update(self, raw, dist):
        rad = _raw_to_rad(raw)
        fwd = -math.sin(rad) * dist
        lat =  math.cos(rad) * dist
        bearing = math.atan2(lat, fwd)        # 0 = devant, + = droite
        hit = dist < self.MAP_MAX
        now = time.time()
        with self._lock:
            # 1) information negative : on libere la ligne de visee
            for o in self._objs:
                if o['y'] <= 0:
                    continue
                ob = math.atan2(o['x'], o['y'])
                if abs(self._angdiff(ob, bearing)) <= self.BEAM_HALF:
                    od = math.hypot(o['x'], o['y'])
                    if (not hit) or od < dist - self.FREE_MARGIN:
                        o['conf'] -= self.CONF_MISS
            # 2) information positive : objet detecte -> associe ou cree
            if hit:
                self._hits.append({'x': lat, 'y': fwd, 'last': now})
                self._hits = self._hits[-self.HIT_MAX:]
                best, bd = None, self.GATE_MM
                for o in self._objs:
                    if o['y'] <= -80.0:
                        continue
                    d2 = math.hypot(o['x'] - lat, o['y'] - fwd)
                    if d2 < bd:
                        bd, best = d2, o
                if best is not None:
                    best['x'] = (1 - self.ALPHA) * best['x'] + self.ALPHA * lat
                    best['y'] = (1 - self.ALPHA) * best['y'] + self.ALPHA * fwd
                    best['conf'] = min(self.CONF_MAX, best['conf'] + self.CONF_HIT)
                    best['last'] = now
                else:
                    self._objs.append({'x': lat, 'y': fwd,
                                       'conf': self.CONF_HIT, 'last': now})
            # 3) oubli des objets disparus ou perimes
            self._objs = [o for o in self._objs
                          if o['conf'] > self.CONF_MIN
                          and (
                              (o['y'] >= 0 and now - o['last'] < self.TIMEOUT)
                              or (o['y'] < 0 and now - o['last'] < self.TRAIL_TIMEOUT)
                          )]
            self._hits = [h for h in self._hits
                          if h['y'] > -VIEW_MAX_MM and now - h['last'] < self.HIT_TIMEOUT]

    def update_line(self, l, m, r):
        now = time.time()
        flags = (l, m, r)
        with self._lock:
            for flag, x in zip(flags, IR_SENSOR_X_MM):
                if flag:
                    self._line.append({'x': x, 'y': IR_SENSOR_FWD_MM, 'last': now})
            self._line = self._line[-IR_LINE_MAX:]
            self._line = [p for p in self._line
                          if p['y'] > -VIEW_MAX_MM and now - p['last'] < IR_LINE_TIMEOUT]

    def advance(self, ds_mm, dtheta=0.0):
        """Ego-motion : le robot avance de ds_mm (et tourne de dtheta rad).
        On deplace les objets suivis dans le sens inverse pour rester dans
        le repere du robot. Les objets passes derriere sont oublies."""
        with self._lock:
            for o in self._objs:
                y = o['y'] - ds_mm
                x = o['x']
                if dtheta:
                    c, s = math.cos(-dtheta), math.sin(-dtheta)
                    o['x'] = x * c - y * s
                    o['y'] = x * s + y * c
                else:
                    o['x'], o['y'] = x, y
            for h in self._hits:
                y = h['y'] - ds_mm
                x = h['x']
                if dtheta:
                    c, s = math.cos(-dtheta), math.sin(-dtheta)
                    h['x'] = x * c - y * s
                    h['y'] = x * s + y * c
                else:
                    h['x'], h['y'] = x, y
            for p in self._line:
                y = p['y'] - ds_mm
                x = p['x']
                if dtheta:
                    c, s = math.cos(-dtheta), math.sin(-dtheta)
                    p['x'] = x * c - y * s
                    p['y'] = x * s + y * c
                else:
                    p['x'], p['y'] = x, y
            self._objs = [o for o in self._objs if o['y'] > -VIEW_MAX_MM]
            self._hits = [h for h in self._hits if h['y'] > -VIEW_MAX_MM]
            self._line = [p for p in self._line if p['y'] > -VIEW_MAX_MM]

    def snapshot(self):
        with self._lock:
            return [(o['x'], o['y'], o['conf'], o['conf'] >= self.CONF_OK)
                    for o in self._objs]

    def detections_snapshot(self):
        now = time.time()
        with self._lock:
            return [(h['x'], h['y'], now - h['last']) for h in self._hits]

    def line_snapshot(self):
        now = time.time()
        with self._lock:
            return [(p['x'], p['y'], now - p['last']) for p in self._line]


world = WorldModel()

def _record_obstacle(dist):
    with _lock:
        raw = _state['us_angle']
    world.update(raw, dist)

def _clear_obstacles():
    world.clear()


# ================================================================ planification obstacle

def _speed_to_mm_s(speed_percent):
    sign = -1.0 if speed_percent < 0 else 1.0
    pct = abs(float(speed_percent))
    profile = SPEED_PROFILE_MM_S

    if pct <= profile[0][0]:
        p0, v0 = profile[0]
        p1, v1 = profile[1]
    elif pct >= profile[-1][0]:
        p0, v0 = profile[-2]
        p1, v1 = profile[-1]
    else:
        for i in range(len(profile) - 1):
            p0, v0 = profile[i]
            p1, v1 = profile[i + 1]
            if p0 <= pct <= p1:
                break

    ratio = (pct - p0) / max(1e-6, p1 - p0)
    return sign * (v0 + ratio * (v1 - v0))

def _steer_to_yaw(ds_mm, steer_angle):
    delta = _clamp(
        (steer_angle - STEER_CENTER) / float(STEER_AMOUNT),
        -1.0, 1.0
    )
    if abs(delta) < 0.05 or abs(ds_mm) < 0.01:
        return 0.0
    wheel = math.radians(delta * STEER_MAX_WHEEL_DEG)
    # Repere world.advance(): dtheta positif = rotation robot vers la gauche.
    return -(ds_mm / ROBOT_LENGTH_MM) * math.tan(wheel)

def advance_world_from_motion(dt):
    s = _snap()
    speed = s.get('speed', 0)
    if speed == 0 or dt <= 0:
        return
    ds = _speed_to_mm_s(speed) * dt
    world.advance(ds, _steer_to_yaw(ds, s.get('steer', STEER_CENTER)))
    _upd('sim_scroll', s.get('sim_scroll', 0.0) + abs(ds))
    _advance_visual_lateral(ds, s.get('steer', STEER_CENTER), s)

def _advance_visual_lateral(ds_mm, steer_angle, s=None):
    if s is None:
        s = _snap()
    steer_ratio = _clamp(
        (steer_angle - STEER_CENTER) / float(STEER_AMOUNT),
        -1.0, 1.0
    )
    lateral = s.get('sim_lateral', 0.0)
    lateral += steer_ratio * abs(ds_mm) * 0.55
    if abs(steer_ratio) < 0.08:
        lateral *= 0.985
    lateral = _clamp(lateral, -360.0, 360.0)
    _upd('sim_lateral', lateral)

def _avoid_candidates():
    return [
        (x, y, conf, ok)
        for x, y, conf, ok in world.snapshot()
        if -ROBOT_LENGTH_MM * 0.45 < y < AVOID_LOOKAHEAD_MM
        and (ok or conf >= AVOID_MIN_CONF)
    ]

def _side_risk(objs, side_dir):
    target_x = side_dir * AVOID_SIDE_TARGET_MM
    risk = 0.0
    for x, y, conf, ok in objs:
        if y <= 0:
            continue
        lateral_clearance = abs(x - target_x)
        overlap = max(0.0, AVOID_PATH_HALF_MM - lateral_clearance)
        forward_weight = 1.0 + (AVOID_LOOKAHEAD_MM - y) / AVOID_LOOKAHEAD_MM
        confidence_weight = 1.0 if ok else 0.65
        risk += overlap * forward_weight * confidence_weight
    return risk

def plan_avoidance():
    """Retourne une consigne d'evitement, ou None si le couloir est libre.

    Le couloir reserve tient compte du robot (100 mm de large) plus une marge.
    x positif = droite, donc dir positif = eviter par la droite.
    """
    objs = _avoid_candidates()
    blockers = [
        (x, y, conf, ok)
        for x, y, conf, ok in objs
        if 0.0 < y < AVOID_LOOKAHEAD_MM and abs(x) < AVOID_PATH_HALF_MM
    ]
    if not blockers:
        _upd('avoid_dir', 0)
        return None

    closest = min(blockers, key=lambda o: o[1])
    left_risk = _side_risk(objs, -1)
    right_risk = _side_risk(objs, 1)
    if abs(left_risk - right_risk) < 1.0:
        side_dir = -1 if closest[0] >= 0 else 1
    else:
        side_dir = -1 if left_risk < right_risk else 1

    urgency = _clamp(
        (AVOID_LOOKAHEAD_MM - closest[1]) /
        max(1.0, AVOID_LOOKAHEAD_MM - AVOID_CRITICAL_Y_MM),
        0.0, 1.0
    )
    steer_mag = 12.0 + urgency * (STEER_AMOUNT - 12.0)
    speed = int(round(SPEED_CRUISE - urgency * (SPEED_CRUISE - SPEED_AVOID_MIN)))
    command = {
        'dir': side_dir,
        'steer': STEER_CENTER + side_dir * steer_mag,
        'speed': max(SPEED_AVOID_MIN, speed),
        'closest_y': closest[1],
        'closest_x': closest[0],
    }
    _upd('avoid_dir', side_dir)
    return command


# ================================================================ helpers servo/moteur

def set_us(raw):
    set_angle(US_CH, raw)
    _upd('us_angle', raw)

def measure():
    d = checkdist()
    _upd('us_dist', d)
    _record_obstacle(d)
    return d

def steer(angle):
    angle = _clamp(angle, STEER_LEFT, STEER_RIGHT)
    set_angle(STEER_CH, to_servo_angle(angle))
    _upd('steer', angle)

def steer_limited(target, step=STEER_SLEW_STEP):
    target = _clamp(target, STEER_LEFT, STEER_RIGHT)
    current = _snap().get('steer', STEER_CENTER)
    delta = _clamp(target - current, -step, step)
    steer(current + delta)
    _upd('target_steer', target)

def drive_avoidance(avoid):
    steer_limited(avoid['steer'])
    set_us(US_FORWARD - 16 * avoid['dir'])
    drive(avoid['speed'], 1)
    _upd('speed', avoid['speed'])

def direct_ping_avoidance(dist):
    if dist >= OBSTACLE_WARN_DIST:
        return None
    s = _snap()
    raw = s.get('us_angle', US_FORWARD)
    rad = _raw_to_rad(raw)
    fwd = -math.sin(rad) * dist
    lat = math.cos(rad) * dist
    if fwd <= 0 or abs(lat) > AVOID_PATH_HALF_MM:
        return None
    side_dir = -1 if lat >= 0 else 1
    urgency = _clamp(
        (OBSTACLE_WARN_DIST - dist) /
        max(1.0, OBSTACLE_WARN_DIST - AVOID_CRITICAL_Y_MM),
        0.0, 1.0
    )
    steer_mag = 16.0 + urgency * (STEER_AMOUNT - 16.0)
    speed = int(round(SPEED_CRUISE - urgency * (SPEED_CRUISE - SPEED_AVOID_MIN)))
    _upd('avoid_dir', side_dir)
    return {
        'dir': side_dir,
        'steer': STEER_CENTER + side_dir * steer_mag,
        'speed': max(SPEED_AVOID_MIN, speed),
        'closest_y': fwd,
        'closest_x': lat,
    }

def read_boundary():
    l, m, r = left_s.value, middle_s.value, right_s.value
    with _lock:
        _state['l'], _state['m'], _state['r'] = l, m, r
    world.update_line(l, m, r)
    return l, m, r

def sleep_with_motion(duration, step=0.03):
    end = time.time() + duration
    last = time.time()
    while not _exit_flag:
        remaining = end - time.time()
        if remaining <= 0:
            break
        time.sleep(min(step, remaining))
        now = time.time()
        advance_world_from_motion(now - last)
        last = now


# ================================================================ mouvements de tete

def initial_scan():
    step, delay = 4, 0.035
    for a in range(US_FORWARD, US_RIGHT - 1, -step):
        set_us(a); time.sleep(delay)
    time.sleep(0.20)
    for a in range(US_RIGHT, US_LEFT + 1, step):
        set_us(a); time.sleep(delay)
    time.sleep(0.20)
    for a in range(US_LEFT, US_FORWARD - 1, -step):
        set_us(a); time.sleep(delay)
    set_us(US_FORWARD)

def head_idle():
    elapsed = time.time() - head_t0
    pos = US_FORWARD + int(HEAD_AMPLITUDE * math.sin(2 * math.pi * elapsed / HEAD_PERIOD))
    set_us(pos)

def us_scan():
    set_us(US_RIGHT);  time.sleep(US_SETTLE); d_right = measure()
    set_us(US_LEFT);   time.sleep(US_SETTLE); d_left  = measure()
    set_us(US_FORWARD)
    return 1 if d_right >= d_left else -1


# ================================================================ manoeuvres

def handle_boundary(l, _m, r):
    stop(); _upd('speed', 0)
    time.sleep(0.05)

    if l and not r:
        turn_dir = 1
    elif r and not l:
        turn_dir = -1
    else:
        turn_dir = us_scan()

    set_us(US_FORWARD - 20 * turn_dir)
    steer(STEER_LEFT if turn_dir > 0 else STEER_RIGHT)
    drive(SPEED_REVERSE, -1); _upd('speed', -SPEED_REVERSE)

    cleared = False
    clear_t = 0.0
    t0 = time.time()
    motion_t = t0
    while time.time() - t0 < T_REVERSE:
        now = time.time()
        advance_world_from_motion(now - motion_t)
        motion_t = now
        l2, m2, r2 = read_boundary()
        if not l2 and not m2 and not r2:
            if not cleared:
                cleared = True
                clear_t = time.time()
            elif time.time() - clear_t > 0.15:
                break
        else:
            cleared = False
        time.sleep(0.02)

    stop(); _upd('speed', 0)
    time.sleep(0.06)

    steer(STEER_RIGHT if turn_dir > 0 else STEER_LEFT)
    drive(SPEED_TURN, 1); _upd('speed', SPEED_TURN)
    sleep_with_motion(T_TURN * 1.3)
    stop(); _upd('speed', 0)
    steer(STEER_CENTER)
    set_us(US_FORWARD)

def handle_obstacle():
    stop(); _upd('speed', 0)
    steer(STEER_CENTER)
    drive(SPEED_REVERSE, -1); _upd('speed', -SPEED_REVERSE)
    sleep_with_motion(0.30)
    stop(); _upd('speed', 0)
    turn_dir = us_scan()
    set_us(US_FORWARD - 20 * turn_dir)
    steer(STEER_RIGHT if turn_dir > 0 else STEER_LEFT)
    drive(SPEED_TURN, 1); _upd('speed', SPEED_TURN)
    sleep_with_motion(T_TURN * 1.3)
    stop(); _upd('speed', 0)
    steer(STEER_CENTER)
    set_us(US_FORWARD)


# ================================================================ controle robot

def start_zone():
    global robot_state, head_t0
    setup()
    steer(STEER_CENTER)
    _upd('target_steer', STEER_CENTER)
    _clear_obstacles()
    _upd('mode', 'SCAN')
    initial_scan()
    head_t0 = time.time()
    robot_state = RUNNING
    _upd('mode', 'ACTIF')

def stop_zone():
    global robot_state
    stop()
    steer(STEER_CENTER)
    set_us(US_FORWARD)
    robot_state = STOPPED
    _upd('mode', 'ARRET')
    _upd('speed', 0)
    _upd('target_steer', STEER_CENTER)
    _upd('avoid_dir', 0)

def robot_loop():
    boundary_hits = 0
    motion_t = time.time()
    while not _exit_flag:
        now = time.time()
        advance_world_from_motion(now - motion_t)
        motion_t = now

        if robot_state == RUNNING:
            l, m, r = read_boundary()
            if l or m or r:
                boundary_hits += 1
                if boundary_hits >= 2:
                    boundary_hits = 0
                    _upd('mode', 'FRONTIERE')
                    handle_boundary(l, m, r)
                    _upd('mode', 'ACTIF')
                continue
            else:
                boundary_hits = 0
            dist = measure()

            avoid = direct_ping_avoidance(dist) or plan_avoidance()
            if dist < AVOID_CRITICAL_Y_MM or (
                avoid is not None and
                avoid['closest_y'] < AVOID_CRITICAL_Y_MM and
                abs(avoid['closest_x']) < ROBOT_HALF_WIDTH_MM + OBSTACLE_RADIUS_MM
            ):
                _upd('mode', 'OBSTACLE')
                handle_obstacle()
                _upd('mode', 'ACTIF')
                continue
            if avoid is not None or dist < OBSTACLE_DIST:
                if avoid is None:
                    seen_angle = _snap().get('us_angle', US_FORWARD)
                    turn_dir = 1 if seen_angle > US_FORWARD else -1
                    avoid = {
                        'dir': turn_dir,
                        'steer': STEER_CENTER + turn_dir * (STEER_AMOUNT * 0.75),
                        'speed': SPEED_AVOID_MIN,
                    }
                    _upd('avoid_dir', turn_dir)
                _upd('mode', 'EVITEMENT')
                drive_avoidance(avoid)
                continue
            _upd('avoid_dir', 0)
            steer_limited(STEER_CENTER, step=STEER_SLEW_STEP * 0.7)
            drive(SPEED_CRUISE, 1)
            _upd('speed', SPEED_CRUISE)
            head_idle()
        time.sleep(0.03)


# ================================================================ visualisation pygame

FPS = 30

# palette claire facon Google Maps
C_BG    = (244, 242, 235)   # blanc casse
C_GRID  = (231, 228, 219)
C_INK   = (55,  60,  68)
C_DIM   = (150, 150, 158)
GOOG    = (66,  133, 244)   # bleu navigation
GOOG_DK = (40,  95,  200)

MODE_COL = {
    'ARRET':     (120, 124, 134),
    'SCAN':      (200, 150,   0),
    'ACTIF':     (16,  160,  72),
    'EVITEMENT': (66,  133, 244),
    'FRONTIERE': (235, 140,   0),
    'OBSTACLE':  (220,  50,  47),
}


class VizPygame:
    def __init__(self):
        pygame.init()
        pygame.display.set_caption('Zone Obstacle  -  Vision 2D')

        info = pygame.display.Info()
        self.W = max(520, info.current_w // 2)
        self.H = max(520, info.current_h - 80)
        self.screen = pygame.display.set_mode((self.W, self.H))

        self.HUD_H = 64
        self.zoom = 1.0

        FF = 'dejavusans,arial,freesans,liberationsans'
        self.f_huge  = pygame.font.SysFont(FF, 30, bold=True)
        self.f_large = pygame.font.SysFont(FF, 20, bold=True)
        self.f_sm    = pygame.font.SysFont(FF, 11)

        self._layout_map()
        self.bg    = self._build_bg()
        self.clock = pygame.time.Clock()

    def _layout_map(self):
        side = min(self.W - 32, self.H - self.HUD_H - 44)
        side = max(360, side)
        self.map_left = (self.W - side) // 2
        self.map_right = self.map_left + side
        self.map_top = self.HUD_H + 16
        self.map_bottom = self.map_top + side
        self.CX = self.map_left + side // 2
        self.car_y = self.map_top + side // 2
        self.view_mm = VIEW_MAX_MM / self.zoom
        self.DS = (side / 2.0) / self.view_mm
        self.MAX_B = int(self.view_mm * self.DS)

    def set_zoom(self, zoom):
        zoom = _clamp(zoom, VIEW_MIN_ZOOM, VIEW_MAX_ZOOM)
        if abs(zoom - self.zoom) < 0.001:
            return
        self.zoom = zoom
        self._layout_map()
        self.bg = self._build_bg()

    def zoom_by(self, factor):
        self.set_zoom(self.zoom * factor)

    # ---- fond statique : carte metrique -------------------------------
    def _build_bg(self):
        W, H = self.W, self.H
        s = pygame.Surface((W, H))
        s.fill(C_BG)

        pygame.draw.rect(
            s, (236, 235, 228),
            (self.map_left, self.map_top, self.map_right - self.map_left, self.map_bottom - self.map_top)
        )
        pygame.draw.rect(
            s, (204, 202, 194),
            (self.map_left, self.map_top, self.map_right - self.map_left, self.map_bottom - self.map_top),
            1
        )

        # Grille tous les 250mm, repere robot-centre. A zoom 1, les bords
        # sont a 2m devant/derriere/gauche/droite.
        grid_step = 250
        max_mm = int(self.view_mm)
        for x_mm in range(-max_mm, max_mm + 1, grid_step):
            x = self.CX + int(x_mm * self.DS)
            col = (211, 209, 201) if x_mm else (170, 172, 174)
            pygame.draw.line(s, col, (x, self.map_top), (x, self.map_bottom), 1)
        for y_mm in range(-max_mm, max_mm + 1, grid_step):
            y = self.car_y - int(y_mm * self.DS)
            col = (211, 209, 201) if y_mm else (170, 172, 174)
            pygame.draw.line(s, col, (self.map_left, y), (self.map_right, y), 1)

        for d_mm in (500, 1000, 1500, 2000):
            if d_mm > self.view_mm:
                continue
            r = int(d_mm * self.DS)
            pygame.gfxdraw.aacircle(s, self.CX, self.car_y, r, (150, 154, 160, 85))
            if d_mm >= 1000:
                txt = ('%.1f' % (d_mm / 1000.0)).rstrip('0').rstrip('.') + 'm'
            else:
                txt = str(d_mm) + 'mm'
            label = self.f_sm.render(txt, True, C_DIM)
            s.blit(label, (self.CX + 6, self.car_y - r - 13))

        view_m = ('%.1f' % (self.view_mm / 1000.0)).rstrip('0').rstrip('.')
        back_label = self.f_sm.render('-' + view_m + 'm arriere', True, C_DIM)
        s.blit(back_label, (self.CX + 6, self.map_bottom - 18))

        return s

    def _car_origin(self, s):
        return self.CX, self.car_y

    def _world_to_px(self, lat, fwd, s):
        cx, cy = self._car_origin(s)
        px = int(cx + lat * self.DS)
        py = int(cy - fwd * self.DS)
        return px, py

    def _draw_detection_range(self, surf, s):
        cx, cy = self._car_origin(s)
        for d_mm, alpha, col in (
            (OBSTACLE_DIST, 80, (220, 50, 47)),
            (OBSTACLE_WARN_DIST, 55, GOOG),
            (VIEW_MAX_MM, 38, C_DIM),
        ):
            r_px = int(d_mm * self.DS)
            pygame.gfxdraw.aacircle(surf, cx, cy, r_px, (*col, alpha))

    def _draw_current_sonar(self, surf, s):
        cx, cy = self._car_origin(s)
        raw = s.get('us_angle', US_FORWARD)
        dist = float(s.get('us_dist', WorldModel.MAP_MAX))
        ray_dist = _clamp(dist, 0.0, min(self.view_mm, WorldModel.MAP_MAX))
        rad = _raw_to_rad(raw)
        fwd = -math.sin(rad) * ray_dist
        lat = math.cos(rad) * ray_dist
        bearing = math.atan2(lat, fwd)

        left = self._world_to_px(
            math.sin(bearing - WorldModel.BEAM_HALF) * ray_dist,
            math.cos(bearing - WorldModel.BEAM_HALF) * ray_dist,
            s
        )
        right = self._world_to_px(
            math.sin(bearing + WorldModel.BEAM_HALF) * ray_dist,
            math.cos(bearing + WorldModel.BEAM_HALF) * ray_dist,
            s
        )
        end = self._world_to_px(lat, fwd, s)

        hit = dist < WorldModel.MAP_MAX
        col = (220, 50, 47) if dist < OBSTACLE_WARN_DIST else GOOG
        beam = pygame.Surface((self.W, self.H), pygame.SRCALPHA)
        pygame.gfxdraw.filled_polygon(beam, [(cx, cy), left, right], (*col, 20 if hit else 12))
        pygame.gfxdraw.aapolygon(beam, [(cx, cy), left, right], (*col, 42 if hit else 28))
        pygame.draw.line(beam, (*col, 115 if hit else 75), (cx, cy), end, 2)
        if hit:
            pygame.gfxdraw.filled_circle(beam, end[0], end[1], 5, (*col, 150))
            pygame.gfxdraw.aacircle(beam, end[0], end[1], 7, (*col, 120))
        surf.blit(beam, (0, 0))

    # ---- objets du monde virtuel --------------------------------------
    def _draw_obstacles(self, surf, s=None):
        if s is None:
            s = _snap()

        for lat, fwd, age in world.detections_snapshot():
            if fwd < -self.view_mm or fwd > self.view_mm:
                continue
            px, py = self._world_to_px(lat, fwd, s)
            if px < self.map_left or px > self.map_right or py < self.map_top or py > self.map_bottom:
                continue
            a = int(_clamp(150 * (1.0 - age / max(0.01, world.HIT_TIMEOUT)), 22, 150))
            dot = pygame.Surface((14, 14), pygame.SRCALPHA)
            pygame.gfxdraw.filled_circle(dot, 7, 7, 3, (255, 95, 35, a))
            surf.blit(dot, (px - 7, py - 7))

        for lat, fwd, conf, confirmed in world.snapshot():
            if fwd < -self.view_mm or fwd > self.view_mm:
                continue
            px, py = self._world_to_px(lat, fwd, s)
            if px < self.map_left or px > self.map_right or py < self.map_top or py > self.map_bottom:
                continue

            behind = fwd < 0

            if confirmed:
                # objet confirme : disque rouge plein, grise quand il est derriere.
                sh = pygame.Surface((30, 30), pygame.SRCALPHA)
                pygame.gfxdraw.filled_circle(sh, 15, 15, 12, (60, 50, 45, 36 if behind else 50))
                surf.blit(sh, (px - 15, py - 13))
                fill = (135, 95, 88) if behind else (217, 60, 45)
                edge = (95, 70, 65) if behind else (150, 30, 22)
                pygame.gfxdraw.filled_circle(surf, px, py, 9, fill)
                pygame.gfxdraw.aacircle(surf, px, py, 9, edge)
                pygame.gfxdraw.aacircle(surf, px, py, 10, edge)
            else:
                # objet potentiel : cercle orange translucide, intensite ~ conf
                a  = int((55 if behind else 80) + 95 * min(conf, 1.0))
                ds = pygame.Surface((30, 30), pygame.SRCALPHA)
                pygame.gfxdraw.filled_circle(ds, 15, 15, 8, (235, 150, 60, a // 2))
                pygame.gfxdraw.aacircle(ds, 15, 15, 8, (220, 130, 40, a))
                surf.blit(ds, (px - 15, py - 15))

    # ---- trace de ligne IR memorisee ---------------------------------
    def _draw_ir_history(self, surf, s=None):
        if s is None:
            s = _snap()
        pts = []
        for lat, fwd, age in world.line_snapshot():
            if fwd < -self.view_mm or fwd > self.view_mm:
                continue
            px, py = self._world_to_px(lat, fwd, s)
            if px < self.map_left or px > self.map_right or py < self.map_top or py > self.map_bottom:
                continue
            alpha = int(_clamp(210 * (1.0 - age / max(0.01, IR_LINE_TIMEOUT)), 45, 210))
            pts.append((px, py, alpha, fwd))

        for px, py, alpha, _fwd in pts:
            dot = pygame.Surface((12, 12), pygame.SRCALPHA)
            pygame.gfxdraw.filled_circle(dot, 6, 6, 4, (10, 10, 10, alpha))
            surf.blit(dot, (px - 6, py - 6))

    # ---- ligne IR -----------------------------------------------------
    def _draw_ir_line(self, surf, l, m, r, s=None):
        if s is None:
            s = _snap()
        cx, cy = self._car_origin(s)
        def pt(dx, dy):
            return int(cx + dx), int(cy + dy)

        hits = [(flag, dx) for flag, dx in ((l, -20), (m, 0), (r, 20)) if flag]
        if hits:
            min_dx = min(dx for _flag, dx in hits) - 18
            max_dx = max(dx for _flag, dx in hits) + 18
            pygame.draw.line(surf, (12, 12, 12), pt(min_dx, -34), pt(max_dx, -34), 10)

        for flag, dx in ((l, -20), (m, 0), (r, 20)):
            px, py = pt(dx, -33)
            col = (8, 8, 8) if flag else (54, 176, 92)
            pygame.gfxdraw.filled_circle(surf, px, py, 4, col)
            pygame.gfxdraw.aacircle(surf, px, py, 4, (255, 255, 255))

    # ---- robot : fleche style Google Maps -----------------------------
    def _draw_car(self, surf, l, m, r, s=None):
        if s is None:
            s = _snap()
        cx, cy = self._car_origin(s)

        def rot(px, py):
            return int(px), int(py)

        # fleche/chevron pointant vers l'avant (haut)
        pts = [
            (cx,        cy - 24),   # pointe
            (cx + 17,   cy + 16),   # aile droite
            (cx,        cy + 6),    # encoche arriere
            (cx - 17,   cy + 16),   # aile gauche
        ]
        pts = [rot(x, y) for x, y in pts]

        # ombre portee
        sh = pygame.Surface((90, 90), pygame.SRCALPHA)
        shp = [(x - cx + 45, y - cy + 45 + 3) for x, y in pts]
        pygame.gfxdraw.filled_polygon(sh, shp, (40, 50, 70, 55))
        surf.blit(sh, (cx - 45, cy - 45))

        # contour blanc (legerement agrandi)
        cxp = sum(p[0] for p in pts) / 4.0
        cyp = sum(p[1] for p in pts) / 4.0
        white = [(int(cxp + (x - cxp) * 1.22), int(cyp + (y - cyp) * 1.22)) for x, y in pts]
        pygame.gfxdraw.filled_polygon(surf, white, (255, 255, 255, 255))
        pygame.gfxdraw.aapolygon(surf, white, (255, 255, 255, 255))

        # corps bleu
        ipts = [(int(x), int(y)) for x, y in pts]
        pygame.gfxdraw.filled_polygon(surf, ipts, GOOG)
        pygame.gfxdraw.aapolygon(surf, ipts, GOOG_DK)

    # ---- HUD haut -----------------------------------------------------
    def _draw_hud(self, surf, s):
        W = self.W
        mode = s['mode']
        dist = s['us_dist']
        spd  = s['speed']
        mc   = MODE_COL.get(mode, C_INK)

        pygame.draw.rect(surf, (255, 255, 255), (0, 0, W, self.HUD_H))
        pygame.draw.line(surf, (224, 221, 212), (0, self.HUD_H), (W, self.HUD_H))
        pygame.draw.rect(surf, mc, (0, 0, 6, self.HUD_H))

        # mode (centre)
        t = self.f_huge.render(mode, True, mc)
        surf.blit(t, (W // 2 - t.get_width() // 2, self.HUD_H // 2 - t.get_height() // 2))

        # distance (gauche)
        ds = str(int(dist)) + ' mm' if dist < 1990 else '---'
        surf.blit(self.f_sm.render('DISTANCE', True, C_DIM), (24, 12))
        surf.blit(self.f_large.render(ds, True, C_INK), (24, 30))

        # vitesse (droite)
        sym = '^' if spd > 0 else ('v' if spd < 0 else '|')
        ss  = sym + ' ' + str(abs(spd)) + ' %'
        sv  = self.f_large.render(ss, True, C_INK)
        sl  = self.f_sm.render('VITESSE', True, C_DIM)
        surf.blit(sl, (W - 24 - sl.get_width(), 12))
        surf.blit(sv, (W - 24 - sv.get_width(), 30))

        # compteur d'objets du monde virtuel (coin bas-gauche de la carte)
        n = len(world.snapshot())
        raw = len(world.detections_snapshot())
        lines = len(world.line_snapshot())
        cnt = self.f_sm.render(str(n) + ' objet' + ('s' if n != 1 else '') +
                               ' | ' + str(raw) + ' pings bruts' +
                               ' | ' + str(lines) + ' pts ligne',
                               True, C_DIM)
        surf.blit(cnt, (16, self.H - 22))

        edge_m = ('%.1f' % (self.view_mm / 1000.0)).rstrip('0').rstrip('.')
        ztxt = 'zoom x%.1f | bord %sm' % (self.zoom, edge_m)
        z = self.f_sm.render(ztxt, True, C_DIM)
        surf.blit(z, (self.W - 16 - z.get_width(), self.H - 22))

    # ---- boucle principale --------------------------------------------
    def run(self):
        while not _exit_flag:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    _shutdown()
                    return
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_m and robot_state == STOPPED:
                        Thread(target=start_zone, daemon=True).start()
                    elif event.key == pygame.K_a and robot_state == RUNNING:
                        stop_zone()
                    elif event.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                        self.zoom_by(1.25)
                    elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                        self.zoom_by(1.0 / 1.25)
                    elif event.key in (pygame.K_0, pygame.K_KP0):
                        self.set_zoom(1.0)
                    elif event.key in (pygame.K_ESCAPE, pygame.K_q):
                        _shutdown()
                        return
                elif event.type == pygame.MOUSEWHEEL:
                    self.zoom_by(1.15 if event.y > 0 else 1.0 / 1.15)
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 4:
                        self.zoom_by(1.15)
                    elif event.button == 5:
                        self.zoom_by(1.0 / 1.15)

            s = _snap()
            l, m, r = s['l'], s['m'], s['r']

            self.screen.blit(self.bg, (0, 0))
            self._draw_detection_range(self.screen, s)
            self._draw_ir_history(self.screen, s)
            self._draw_obstacles(self.screen, s)
            self._draw_current_sonar(self.screen, s)
            self._draw_car(self.screen, l, m, r, s)
            self._draw_ir_line(self.screen, l, m, r, s)
            self._draw_hud(self.screen, s)

            pygame.display.flip()
            self.clock.tick(FPS)

        pygame.quit()


# ================================================================ main

def _run_terminal():
    print("=== Zone Obstacle (mode terminal) ===")
    print("  M : demarrer    A : arreter    Ctrl-C : quitter")
    try:
        while True:
            if select.select([sys.stdin], [], [], 0)[0]:
                cmd = sys.stdin.readline().strip().upper()
                if cmd == 'M' and robot_state == STOPPED:
                    Thread(target=start_zone, daemon=True).start()
                elif cmd == 'A' and robot_state == RUNNING:
                    stop_zone()
            s = _snap()
            print("\r  %-10s  dist=%4dmm  obj=%d  dir=%2d  L=%d M=%d R=%d  v=%3d%%" % (
                s['mode'], int(s['us_dist']), len(world.snapshot()), s.get('avoid_dir', 0),
                s['l'], s['m'], s['r'], s['speed']),
                end='', flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()


_shutdown_done = False

def _shutdown():
    global _exit_flag, _shutdown_done
    if _shutdown_done:
        return
    _shutdown_done = True
    _exit_flag = True
    if robot_state == RUNNING:
        stop_zone()
    else:
        stop()
        steer(STEER_CENTER)
        set_us(US_FORWARD)
    print('Nettoyage final')


if __name__ == '__main__':
    Thread(target=robot_loop, daemon=True).start()

    use_gui = '--no-gui' not in sys.argv and PYGAME_OK
    if use_gui:
        try:
            viz = VizPygame()
            viz.run()
        except Exception as e:
            print('GUI error:', e)
            use_gui = False

    if not use_gui:
        if not PYGAME_OK:
            print("pygame non disponible - mode terminal (pip install pygame)")
        try:
            _run_terminal()
        except Exception:
            pass

    _shutdown()
