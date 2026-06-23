#!/usr/bin/env python3
# Zone obstacle : navigation libre + panel de vision 2D en temps reel

import time
import math
import tkinter as tk
from threading import Thread, Lock

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

T_REVERSE = 0.55
T_TURN    = 0.50

STOPPED = 0
RUNNING = 1

# ================================================================ etat partage
_lock = Lock()
_state = {
    'mode':     'ARRET',
    'l': 0, 'm': 0, 'r': 0,
    'us_angle': US_FORWARD,
    'us_dist':  2000,
    'speed':    0,
    'steer':    STEER_CENTER,
}

robot_state = STOPPED
head_t0     = 0.0


def _upd(key, val):
    with _lock:
        _state[key] = val

def _snap():
    with _lock:
        return dict(_state)


# ================================================================ helpers servo/moteur

def set_us(raw):
    set_angle(US_CH, raw)
    _upd('us_angle', raw)

def measure():
    d = checkdist()
    _upd('us_dist', d)
    return d

def steer(angle):
    set_angle(STEER_CH, to_servo_angle(angle))
    _upd('steer', angle)

def read_boundary():
    l, m, r = left_s.value, middle_s.value, right_s.value
    with _lock:
        _state['l'], _state['m'], _state['r'] = l, m, r
    return l, m, r


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

def maneuver(rev_t, turn_dir, turn_t):
    steer(STEER_CENTER)
    drive(SPEED_REVERSE, -1); _upd('speed', -SPEED_REVERSE)
    time.sleep(rev_t)
    stop(); _upd('speed', 0)
    set_us(US_FORWARD - 20 * turn_dir)
    steer(STEER_RIGHT if turn_dir > 0 else STEER_LEFT)
    drive(SPEED_TURN, 1); _upd('speed', SPEED_TURN)
    time.sleep(turn_t)
    stop(); _upd('speed', 0)
    steer(STEER_CENTER)
    set_us(US_FORWARD)

def handle_boundary(l, m, r):
    if l and r:
        maneuver(T_REVERSE, us_scan(), T_TURN)
    elif m and not l and not r:
        maneuver(T_REVERSE, us_scan(), T_TURN)
    elif l:
        maneuver(T_REVERSE * 0.8,  1, T_TURN)
    elif r:
        maneuver(T_REVERSE * 0.8, -1, T_TURN)

def handle_obstacle():
    stop(); _upd('speed', 0)
    steer(STEER_CENTER)
    drive(SPEED_REVERSE, -1); _upd('speed', -SPEED_REVERSE)
    time.sleep(0.30)
    stop(); _upd('speed', 0)
    turn_dir = us_scan()
    set_us(US_FORWARD - 20 * turn_dir)
    steer(STEER_RIGHT if turn_dir > 0 else STEER_LEFT)
    drive(SPEED_TURN, 1); _upd('speed', SPEED_TURN)
    time.sleep(T_TURN * 1.3)
    stop(); _upd('speed', 0)
    steer(STEER_CENTER)
    set_us(US_FORWARD)


# ================================================================ controle robot

def start_zone():
    global robot_state, head_t0
    setup()
    steer(STEER_CENTER)
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

def robot_loop():
    while True:
        if robot_state == RUNNING:
            l, m, r = read_boundary()
            if l or m or r:
                _upd('mode', 'FRONTIERE')
                handle_boundary(l, m, r)
                _upd('mode', 'ACTIF')
                continue
            dist = measure()
            if dist < OBSTACLE_DIST:
                _upd('mode', 'OBSTACLE')
                handle_obstacle()
                _upd('mode', 'ACTIF')
                continue
            steer(STEER_CENTER)
            drive(SPEED_CRUISE, 1)
            _upd('speed', SPEED_CRUISE)
            head_idle()
        time.sleep(0.03)


# ================================================================ visualisation 2D

W, H  = 440, 540
CX    = W // 2
CY    = 310          # centre robot dans le canvas
ROB_W, ROB_H = 30, 48

# origine du faisceau (avant du robot)
BX, BY = CX, CY - ROB_H // 2

# positions capteurs de ligne (relatif a CX, CY)
SENSOR_POS = [(-13, -ROB_H // 2 - 5),
              (  0, -ROB_H // 2 - 5),
              ( 13, -ROB_H // 2 - 5)]

DIST_SCALE  = 0.11   # mm → pixels  (420mm ≈ 46px)
MAX_BEAM_PX = 170

# palette Tesla-like
BG_COL     = '#0b0b1a'
GRID_COL   = '#161630'
ROBOT_COL  = '#ccccdd'
ARROW_COL  = '#ffffff'
BEAM_CLEAR = '#1a7fff'
BEAM_WARN  = '#ff4400'
OBS_COL    = '#ff4422'
S_ON       = '#ff3333'
S_OFF      = '#22cc55'
TXT        = '#dde0ff'
TXT_DIM    = '#44446a'

MODE_COLOR = {
    'ARRET':     '#666688',
    'SCAN':      '#ffcc00',
    'ACTIF':     '#22dd66',
    'FRONTIERE': '#ff8800',
    'OBSTACLE':  '#ff3333',
}


def _raw_to_rad(raw):
    """Convertit un angle servo brut en radians canvas (0 = droite, -pi/2 = haut)."""
    deg_from_fwd = (raw - US_FORWARD) * (45.0 / 42.0)
    return math.radians(-90 - deg_from_fwd)


class RobotViz:
    def __init__(self, root):
        self.root = root
        root.title('Zone Obstacle — Vision 2D')
        root.configure(bg=BG_COL)
        root.resizable(False, False)

        self.cv = tk.Canvas(root, width=W, height=H, bg=BG_COL, highlightthickness=0)
        self.cv.pack()

        self._draw_static()
        self._tick()

        root.bind('<m>', lambda _: self._start())
        root.bind('<M>', lambda _: self._start())
        root.bind('<a>', lambda _: self._stop())
        root.bind('<A>', lambda _: self._stop())

    # ---- commandes clavier
    def _start(self):
        if robot_state == STOPPED:
            Thread(target=start_zone, daemon=True).start()

    def _stop(self):
        if robot_state == RUNNING:
            stop_zone()

    # ---- elements statiques (dessinés une seule fois)
    def _draw_static(self):
        # grille de fond
        for x in range(0, W + 1, 44):
            self.cv.create_line(x, 56, x, H - 70, fill=GRID_COL)
        for y in range(56, H - 70, 44):
            self.cv.create_line(0, y, W, y, fill=GRID_COL)
        # cercles de distance de reference
        for r_mm in [200, 400, 800, 1500]:
            r_px = int(r_mm * DIST_SCALE)
            self.cv.create_oval(BX - r_px, BY - r_px, BX + r_px, BY + r_px,
                                outline=GRID_COL, dash=(3, 6))
            self.cv.create_text(BX + r_px + 3, BY, text=f'{r_mm}mm',
                                fill=TXT_DIM, font=('Helvetica', 7), anchor='w')
        # legende bas
        self.cv.create_text(W // 2, H - 12,
                            text='M  démarrer      A  arrêter',
                            fill=TXT_DIM, font=('Helvetica', 9))

    # ---- boucle de rendu (50 ms)
    def _tick(self):
        s = _snap()
        self.cv.delete('dyn')
        self._draw_frame(s)
        self.root.after(50, self._tick)

    def _draw_frame(self, s):
        mode  = s['mode']
        dist  = s['us_dist']
        raw_a = s['us_angle']
        l, m, r = s['l'], s['m'], s['r']
        spd   = s['speed']

        # ---- barre de statut haut
        self.cv.create_rectangle(0, 0, W, 54, fill='#10101f', outline='', tags='dyn')
        mode_c = MODE_COLOR.get(mode, TXT)
        self.cv.create_text(W // 2, 17, text=mode,
                            fill=mode_c, font=('Helvetica', 16, 'bold'), tags='dyn')

        dist_txt = f'{int(dist):>4} mm' if dist < 1990 else ' ---  mm'
        dir_sym  = '▲' if spd > 0 else ('▼' if spd < 0 else '■')
        self.cv.create_text(20, 40, text=f'DIST  {dist_txt}',
                            fill=TXT, font=('Helvetica', 9), anchor='w', tags='dyn')
        self.cv.create_text(W - 20, 40, text=f'{dir_sym} {abs(spd):>2}%',
                            fill=TXT, font=('Helvetica', 9), anchor='e', tags='dyn')

        # ---- faisceau ultrason
        angle = _raw_to_rad(raw_a)
        beam_px = min(dist * DIST_SCALE, MAX_BEAM_PX)
        ex = BX + math.cos(angle) * beam_px
        ey = BY + math.sin(angle) * beam_px

        spread = math.radians(9)
        lx = BX + math.cos(angle - spread) * beam_px
        ly = BY + math.sin(angle - spread) * beam_px
        rx = BX + math.cos(angle + spread) * beam_px
        ry = BY + math.sin(angle + spread) * beam_px

        b_col = BEAM_WARN if dist < OBSTACLE_DIST else BEAM_CLEAR
        self.cv.create_polygon(BX, BY, lx, ly, ex, ey, rx, ry,
                               fill=b_col, stipple='gray25', outline='', tags='dyn')
        self.cv.create_line(BX, BY, ex, ey, fill=b_col, width=2, tags='dyn')

        # point obstacle
        if dist < OBSTACLE_DIST:
            self.cv.create_oval(ex - 9, ey - 9, ex + 9, ey + 9,
                               fill=OBS_COL, outline='#ff9966', width=2, tags='dyn')

        # ---- robot (rectangle + fleche)
        x1, y1 = CX - ROB_W // 2, CY - ROB_H // 2
        x2, y2 = CX + ROB_W // 2, CY + ROB_H // 2
        self.cv.create_rectangle(x1, y1, x2, y2,
                                 fill=ROBOT_COL, outline='#ffffff', width=2, tags='dyn')
        self.cv.create_polygon(CX,        CY - ROB_H // 2 - 10,
                               CX - 7,    CY - ROB_H // 2 + 3,
                               CX + 7,    CY - ROB_H // 2 + 3,
                               fill=ARROW_COL, tags='dyn')

        # ---- capteurs de ligne (dots avant du robot)
        for (ox, oy), val in zip(SENSOR_POS, [l, m, r]):
            sx, sy = CX + ox, CY + oy
            col = S_ON if val else S_OFF
            self.cv.create_oval(sx - 5, sy - 5, sx + 5, sy + 5,
                               fill=col, outline='', tags='dyn')

        # ---- legende capteurs (barre bas)
        self.cv.create_rectangle(0, H - 68, W, H - 30,
                                 fill='#10101f', outline='', tags='dyn')
        for i, (label, val) in enumerate(zip(['G', 'M', 'D'], [l, m, r])):
            cx_s = W // 2 + (i - 1) * 55
            col = S_ON if val else S_OFF
            self.cv.create_oval(cx_s - 11, H - 63, cx_s + 11, H - 41,
                               fill=col, outline='', tags='dyn')
            self.cv.create_text(cx_s, H - 34, text=label,
                               fill=TXT_DIM, font=('Helvetica', 8), tags='dyn')


# ================================================================ main

if __name__ == '__main__':
    root = tk.Tk()
    RobotViz(root)
    Thread(target=robot_loop, daemon=True).start()
    root.mainloop()
    stop()
    set_us(US_FORWARD)
    print('Nettoyage final')
