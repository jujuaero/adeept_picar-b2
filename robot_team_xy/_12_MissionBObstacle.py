#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Zone obstacle : navigation libre + panel de vision 2D (pygame)

import sys
import select
import time
import math
import os
import json
import base64
import urllib.request
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

STEER_CENTER = CENTER_ANGLE
STEER_AMOUNT = 38
STEER_LEFT   = STEER_CENTER - STEER_AMOUNT
STEER_RIGHT  = STEER_CENTER + STEER_AMOUNT

VIEW_MAX_MM = 2000
VIEW_MIN_ZOOM = 0.50
VIEW_MAX_ZOOM = 4.00

ROBOT_LENGTH_MM = 120.0         # empattement mesure (axe AV <-> axe AR)
IR_SENSOR_FWD_MM = 90.0
IR_SENSOR_X_MM   = (-45.0, 0.0, 45.0)
IR_LINE_TIMEOUT  = 45.0
IR_LINE_MAX      = 700

SPEED_PROFILE_MM_S     = (
    (30.0, 250.0),                  # v stabilisee (methode 2 durees) 2026-06-29
    (40.0, 315.0),                  # v stabilisee (l'ancien 370 surestimait)
    (50.0, 380.0),                  # extrapole (non mesure, jamais atteint en mission)
)
SPEED_MM_PER_SEC_AT_35 = 282.0      # interpolation de la table ci-dessus
STEER_MAX_WHEEL_DEG    = 20.0       # rayon de virage MESURE (cercle chrono a 30%) :
                                    # diametre 670mm -> R=335mm -> atan(120/335)=20deg
                                    # (a 22% le robot tourne plus serre, R~283mm -> 23deg)
TURN_SCRUB_FULL        = 0.82       # patinage : vitesse d'arc / vitesse droite a plein
                                    # braquage (cercle 30% = 204 mm/s vs 250 en ligne droite)

STOPPED = 0
RUNNING = 1
STARTING = 2
_exit_flag = False
_hardware_ready = False

# ================================================================ etat partage
_lock = Lock()
_drive_lock = Lock()
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
    'ai_ok': 0,
    'ai_gate': None,
    'ai_width': None,
    'ai_age': 999.0,
    'ai_dets': 0,
    'ai_ms': 0,
}

robot_state = STOPPED

# anti-collage : cote du dernier contact de bord et nombre de contacts consecutifs
_boundary_last_dir = 0
_boundary_streak = 0
# cap estime (rad) integre depuis la motricite ; sert d'anti demi-tour. + = gauche.
_heading = 0.0
# biais interieur residuel garde en croisiere apres un contact de bord :
# le robot continue a se tirer vers le centre au lieu de re-longer le bord.
_inward_bias_dir = 0
_inward_bias_until = 0.0
# alternance evitement : dernier cote d'evitement et son horodatage (anti-derive :
# on n'evite jamais 2x de suite du meme cote dans une fenetre de AVOID_ALTERNATE_T).
_last_avoid_dir = 0
_last_avoid_time = 0.0
# derniere direction de trajet (camera) : on choisit ensuite la porte a l'extremite
# INVERSE (alternance). +1 = droite, -1 = gauche. 0 = pas encore de reference ->
# le tout premier scan choisit son cote via sonar (_initial_search_side), PAS
# code en dur, sinon la mission commence toujours par regarder a droite.
_last_go_dir = 0


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

def _running():
    """Vrai tant que la mission tourne. Les manoeuvres bloquantes doivent le
    surveiller pour pouvoir s'interrompre net quand on appuie sur 'A'."""
    return robot_state == RUNNING and not _exit_flag

class WorldModel:
    """Carte d'objets persistante, centree sur le robot.

    - un ping qui touche cree ou renforce un objet (information positive)
    - un ping qui traverse une zone supposee occupee affaiblit l'objet :
      il a disparu (information negative / espace libre)
    - les objets non revus depuis longtemps s'effacent

    Repere robot : x = lateral (+ = droite), y = avant (+ = devant), en mm.
    Un objet peu confiant est "potentiel", tres confiant il est "confirme".
    """
    ASSOC_MM    = 170.0              # rayon d'association ping <-> objet
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
                best, bd = None, self.ASSOC_MM
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

def _turn_scrub(steer_angle):
    """Le robot patine en virage : il avance moins le long de l'arc qu'en ligne
    droite. Renvoie 1.0 tout droit -> TURN_SCRUB_FULL a plein braquage."""
    delta = _clamp((steer_angle - STEER_CENTER) / float(STEER_AMOUNT), -1.0, 1.0)
    return 1.0 - (1.0 - TURN_SCRUB_FULL) * abs(delta)

def advance_world_from_motion(dt):
    global _heading
    s = _snap()
    speed = s.get('speed', 0)
    if speed == 0 or dt <= 0:
        return
    steer = s.get('steer', STEER_CENTER)
    ds = _speed_to_mm_s(speed) * dt * _turn_scrub(steer)
    dtheta = _steer_to_yaw(ds, steer)
    _heading += dtheta
    world.advance(ds, dtheta)
    _upd('sim_scroll', s.get('sim_scroll', 0.0) + abs(ds))
    _advance_visual_lateral(ds, steer, s)

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

# ================================================================ controle reactif

# Parametres reglables de la mission. Valeurs volontairement prudentes :
# la ligne noire est prioritaire, donc la croisiere reste lente.
CONTROL_DT = 0.018              # s : periode max entre deux lectures ligne en marche avant
SONAR_MIN_MM = 45               # mm : en dessous, mesure HC-SR04 aberrante
SONAR_NO_ECHO_MM = 1950         # mm : 2000 ~= pas d'echo, traite a part
SONAR_STICKY_T = 0.45           # s : garde un obstacle recent malgre un no-echo isole
SONAR_SAMPLES_FRONT = 3         # mediane de N mesures en face
SONAR_SAMPLES_SIDE = 2          # mediane courte pendant les scans gauche/droite

SPEED_CRUISE = 25              # % : baisser si la ligne est encore franchie
SPEED_AVOID = 22               # % : vitesse des arcs d'evitement en marche avant
SPEED_REVERSE = 34             # % : recul franc pour sortir de la frontiere

OBSTACLE_TRIGGER_MM = 560      # mm : declenche l'evitement
OBSTACLE_CRITICAL_MM = 300     # mm : trop pres -> recul avant contournement
OBSTACLE_CLEAR_MM = 760        # mm : distance frontale jugee degagee apres braquage

# Memoire : on ne fonce pas dans un obstacle vu il y a peu mais sorti du faisceau.
MAP_CORRIDOR_HALF_MM = 110     # mm : demi-largeur du corridor devant (robot + marge)
MAP_LOOKAHEAD_MM = 0           # mm : 0 = memoire desactivee (ego-motion trop imprecis) ; sonar-live seul
MAP_MIN_CONF = 0.50            # confiance mini pour agir sur un objet memorise (evite les pings isoles)

# Perception avant par balayage de la tete (modele-independant : on re-mesure).
# Remplace la lecture unique droit-devant, qui rate les obstacles legerement decales.
SWEEP_RAWS   = (US_FORWARD + 34, US_FORWARD + 17, US_FORWARD,
                US_FORWARD - 17, US_FORWARD - 34)   # ~ +/-36 deg autour de l'avant
SWEEP_SETTLE = 0.06            # s : pose de la tete a chaque angle (monter si mesures bruitees)

BOUNDARY_REVERSE_T = 0.95      # s : recul en virage quand la ligne est vue (arrivee de face)
BOUNDARY_NUDGE_T = 0.35        # s : arc AVANT doux pour se decoller d'un effleurement
BOUNDARY_NUDGE_SPEED = 20      # % : vitesse du decollement (on garde la progression)
BOUNDARY_INWARD_T = 0.55       # s : arc de reprise courbe vers l'interieur (au lieu de tout droit)
BOUNDARY_INWARD_SPEED = 20     # % : vitesse de la reprise vers le centre
BOUNDARY_INWARD_SCALE = 0.60   # 0..1 : braquage de base de la reprise vers l'interieur
BOUNDARY_STREAK_MAX = 3        # contacts consecutifs au-dela desquels le biais interieur sature
BOUNDARY_BIAS_T = 1.6          # s : duree du braquage interieur residuel garde en croisiere
BOUNDARY_BIAS_SCALE = 0.32     # 0..1 : intensite du braquage residuel (0 = ancien comportement)
OBSTACLE_BACKUP_T = 0.32       # s : petit recul si obstacle trop proche
AVOID_STEP_T = 0.22            # s : pas d'arc court entre deux relectures sonar (evitement reactif)
AVOID_PASS_T = 0.60            # s : avance ~droit pour passer l'obstacle une fois le front degage
AVOID_MAX_STEPS = 20           # borne dure du contournement (anti-boucle infinie)
AVOID_EMERG_MM = 160           # mm : quasi-collision -> petit recul (sinon on tourne EN avancant)
AVOID_REACT_MM = 380           # mm : nouvel obstacle pendant passage/realignement -> on relache
HEADING_MAX_DEG = 95.0         # anti demi-tour : rotation max autorisee pendant UNE manoeuvre
HEADING_ALIGN_DEG = 12.0       # deg : tolerance pour se juger realigne sur le cap d'avant-obstacle
AVOID_DEFAULT_DIR = -1         # cote choisi si gauche/droite sont vraiment identiques
AVOID_SIDE_MARGIN_MM = 140     # diff sonar mini pour croire qu'un cote est vraiment meilleur
AVOID_STUCK_FLIPS = 2          # apres N quasi-collisions, on tente l'autre cote
AVOID_ALTERNATE_T = 6.0        # s : deux evitements rapproches ne se font JAMAIS du meme
                               # cote -> on force l'alternance (droite-gauche-droite...).
AVOID_ALTERNATE_MARGIN_MM = 450  # mm : sauf si un cote est BEAUCOUP plus degage que l'autre
                                 # (mur d'un cote) : la securite prime alors sur l'alternance.
GATE_SEARCH_MAX_STEPS = 6      # arcs de recherche de porte du cote OPPOSE au dernier evitement
GATE_SEARCH_STEP_T = 0.25      # s : duree d'un arc de recherche (la camera doit avoir le temps de voir)
GATE_SEARCH_SPEED = 18         # % : vitesse lente de recherche (sinon la camera ne suit pas)

RECENTER_SPEED = 20            # % : recentrage volontairement plus calme
RECENTER_T = 1.05              # s : fallback si l'estimation laterale est indisponible
RECENTER_STEER_SCALE = 0.70    # 0..1 : braquage pour revenir vers le centre
RECENTER_ABORT_MM = 700        # mm : abandon du recentrage si obstacle devant
RECENTER_DEADBAND_MM = 45.0    # ignore les petites derives estimees (modele open-loop)
RECENTER_MAX_T = 1.40          # borne dure : ne pas traverser la zone pour "corriger"
RECENTER_STEP_T = 0.18         # correction par petits arcs interruptibles

# Guidage camera deporte sur laptop GPU. Desactive par defaut :
#   PICAR_REMOTE_AI_URL=http://IP_DU_LAPTOP:8765/detect python3 _12_MissionBObstacle.py
REMOTE_AI_URL = os.environ.get('PICAR_REMOTE_AI_URL', '').strip()
REMOTE_AI_PERIOD = 0.25        # s : frequence d'envoi camera vers le laptop
REMOTE_AI_TIMEOUT = 1.5        # s : marge large. Le client IA tourne dans SON thread (ne
                               # bloque jamais robot_loop) ; un timeout trop court jetait des
                               # reponses 200 valides et forcait une reouverture camera de 2s.
REMOTE_AI_MAX_AGE = 1.2        # s : au-dela, on ignore le dernier conseil IA
REMOTE_AI_W = 640
REMOTE_AI_H = 480
REMOTE_AI_JPEG_QUALITY = 65
REMOTE_AI_HFOV_DEG = 54.0
REMOTE_AI_MIN_WIDTH_DEG = 8.0  # passage trop etroit/peu fiable -> ignore
REMOTE_AI_STEER_MAX_DEG = 24.0
REMOTE_AI_STEER_SCALE = 0.62   # 0..1 : intensite max du braquage IA en croisiere
# Sens du braquage IA. La camera renvoie +deg = passage a DROITE ; le materiel a un
# offset direction INVERSE (STEER_CENTER + offset positif = roues a GAUCHE, cf.
# _04_motor / parcours.py). Ce chemin IA n'a PAS la double inversion du sonar : on
# inverse donc ici (-1) pour aller VERS la porte. Si le robot fuit la porte, mettre +1.
REMOTE_AI_STEER_SIGN = -1.0
# Camera prioritaire : quand un passage fiable est vu, on le SUIT sans bouger la tete.
# Le balayage sonar lateral (qui fait "fouetter" la tete) n'est declenche que si la
# camera ne voit pas de passage, ou si un obstacle tres proche est nettement HORS de
# l'axe de la porte (vrai obstacle, pas un montant sur le cote du passage).
REMOTE_AI_COMMIT_DEG = 10.0   # deg : en dessous, la porte est jugee "dans l'axe"
REMOTE_AI_CREEP_SPEED = 18    # % : vitesse d'approche rapprochee pour franchir proprement

# --- Parcours porte par porte (camera fixe + tete sonar fixe vers l'avant) ---
GO_T = 4.0             # s : duree de progression vers une porte choisie
TURN_BODY_T = 0.5      # s : rotation du corps vers l'AUTRE cote apres une porte
DOOR_STEP_T = 0.15     # s : pas de reajustement du cap pendant la progression
SEARCH_STEP_T = 0.4    # s : pas de rotation quand aucune porte n'est vue
SEARCH_CAM_DIR = 1     # sens de balayage quand on cherche (+1 = droite camera)

# --- Option 3 : fusion camera(angle) + sonar(distance) ---
US_CAM_SIGN = 1.0      # +1 : angle camera + (droite) -> tete raw vers US_LEFT (droite phys.)
                       # si le sonar pointe du mauvais cote, mettre -1
GATE_PASS_EXTRA_MM = 150  # mm : distance a parcourir AU-DELA de la ligne des bouteilles
                          # pour franchir la porte (borne par GO_T de toute facon)

MOVE_DONE = 'done'
MOVE_STOPPED = 'stopped'
MOVE_LINE = 'line'
MOVE_OBSTACLE = 'obstacle'


class SonarFilter:
    """Filtre minimal pour un HC-SR04 bruyant.

    Les valeurs proches de 2000 mm ne sont pas prises comme preuve immediate
    que la voie est libre. En face, on garde pendant SONAR_STICKY_T la derniere
    vraie mesure valide afin qu'un no-echo isole ne supprime pas un obstacle.
    """
    def __init__(self):
        self.last_valid = VIEW_MAX_MM
        self.last_valid_t = 0.0

    @staticmethod
    def valid(d):
        return SONAR_MIN_MM <= d < SONAR_NO_ECHO_MM

    def read(self, samples=SONAR_SAMPLES_FRONT, sticky=True, delay=0.012, watch_line=False):
        vals = []
        for _ in range(samples):
            if not _running():
                return VIEW_MAX_MM
            if watch_line and line_seen():
                safe_stop_outputs(center=False)
                return None

            d = checkdist()
            if watch_line and line_seen():
                safe_stop_outputs(center=False)
                return None

            d = VIEW_MAX_MM if d is None else float(d)
            d = _clamp(d, 0.0, VIEW_MAX_MM)
            _upd('us_dist', d)
            _record_obstacle(d)
            if self.valid(d):
                vals.append(d)
            if delay > 0:
                if not interruptible_sleep(delay, step=0.004, watch_line=watch_line):
                    return None if watch_line else VIEW_MAX_MM

        now = time.time()
        if vals:
            vals.sort()
            filtered = vals[len(vals) // 2]
            self.last_valid = filtered
            self.last_valid_t = now
        elif sticky and now - self.last_valid_t <= SONAR_STICKY_T:
            filtered = self.last_valid
        else:
            filtered = VIEW_MAX_MM

        _upd('us_dist', filtered)
        return filtered

    def reset(self):
        self.last_valid = VIEW_MAX_MM
        self.last_valid_t = 0.0


sonar_filter = SonarFilter()


# ================================================================ helpers materiel

def set_us(raw):
    set_angle(US_CH, raw)
    _upd('us_angle', raw)

def steer(angle):
    angle = _clamp(angle, STEER_LEFT, STEER_RIGHT)
    set_angle(STEER_CH, to_servo_angle(angle))
    _upd('steer', angle)

def _motor_stop_unlocked():
    if _hardware_ready:
        stop()
    _upd('speed', 0)

def safe_stop_outputs(center=True):
    with _drive_lock:
        _motor_stop_unlocked()
        if center:
            steer(STEER_CENTER)
            _upd('target_steer', STEER_CENTER)

def safe_drive(speed_percent, direction, steer_angle):
    """Commande moteur atomique vis-a-vis de stop_zone().

    Si A vient de passer le robot en STOPPED, cette fonction refuse de relancer
    les moteurs. stop_zone() utilise le meme verrou et gagne donc proprement.
    """
    with _drive_lock:
        if not _running():
            _motor_stop_unlocked()
            return False
        steer(steer_angle)
        if direction == 0 or speed_percent <= 0:
            _motor_stop_unlocked()
        else:
            drive(speed_percent, direction)
            _upd('speed', speed_percent if direction > 0 else -speed_percent)
    return True

def read_boundary():
    l, m, r = left_s.value, middle_s.value, right_s.value
    with _lock:
        _state['l'], _state['m'], _state['r'] = l, m, r
    world.update_line(l, m, r)
    return l, m, r

def line_seen(flags=None):
    if flags is None:
        flags = read_boundary()
    return bool(flags[0] or flags[1] or flags[2])

def interruptible_sleep(duration, step=CONTROL_DT, watch_line=False):
    end = time.time() + duration
    last = time.time()
    while _running():
        remaining = end - time.time()
        if remaining <= 0:
            return True
        if watch_line and line_seen():
            safe_stop_outputs(center=False)
            return False
        time.sleep(min(step, remaining))
        now = time.time()
        advance_world_from_motion(now - last)
        last = now
        if watch_line and line_seen():
            safe_stop_outputs(center=False)
            return False
    safe_stop_outputs(center=False)
    return False


# ================================================================ mouvements de tete

def initial_scan():
    """Un SEUL balayage lent (centre->gauche->droite->centre) pour peupler la carte.
    Avant : 3 passages rapides (step 8 / delay 0.025 ~ 320 u/s) = tete qui fouette.
    Ici step 6 / delay 0.05 (~120 u/s) = calme."""
    step, delay = 6, 0.05
    passes = (
        range(US_FORWARD, US_LEFT + 1, step),    # centre -> gauche
        range(US_LEFT, US_RIGHT - 1, -step),     # gauche -> droite
        range(US_RIGHT, US_FORWARD + 1, step),   # droite -> centre
    )
    for arc in passes:
        for a in arc:
            if _exit_flag:
                set_us(US_FORWARD)
                return
            set_us(a)
            time.sleep(delay)
    set_us(US_FORWARD)

def scan_side(raw_angle):
    set_us(raw_angle)
    if not interruptible_sleep(US_SETTLE, step=CONTROL_DT):
        return VIEW_MAX_MM
    return sonar_filter.read(samples=SONAR_SAMPLES_SIDE, sticky=False)

def _cam_deg_to_us_raw(cam_deg):
    """Convertit un angle CAMERA (deg, + = droite) en position servo sonar (raw).
    Materiel : US_LEFT=142 pointe physiquement a DROITE (cf. [[steering-sign-inverted]]),
    donc un angle camera positif -> raw > US_FORWARD (regler US_CAM_SIGN si inverse)."""
    raw = US_FORWARD + US_CAM_SIGN * cam_deg * (42.0 / 45.0)
    return int(_clamp(raw, US_RIGHT, US_LEFT))

def measure_at_angle(cam_deg):
    """Pointe la tete sonar vers l'angle CAMERA donne, mesure une fois, puis
    RECENTRE la tete devant. Renvoie la distance (mm), VIEW_MAX_MM si rien."""
    d = scan_side(_cam_deg_to_us_raw(cam_deg))
    set_us(US_FORWARD)
    return d

def choose_clear_side():
    """Retourne +1 pour droite, -1 pour gauche.

    Le sonar a un cone large : une petite difference gauche/droite n'est pas
    fiable. Dans ce cas on choisit le cote qui rembourse le decalage lateral
    estime, pour eviter d'empiler tous les contournements du meme cote.
    """
    safe_stop_outputs(center=True)
    d_right = scan_side(US_RIGHT)
    d_left = scan_side(US_LEFT)
    set_us(US_FORWARD)
    if not _running():
        return AVOID_DEFAULT_DIR

    lateral = float(_snap().get('sim_lateral', 0.0))
    if d_right > d_left + AVOID_SIDE_MARGIN_MM:
        turn_dir = 1
    elif d_left > d_right + AVOID_SIDE_MARGIN_MM:
        turn_dir = -1
    elif abs(lateral) > RECENTER_DEADBAND_MM:
        turn_dir = -1 if lateral > 0 else 1
    else:
        turn_dir = AVOID_DEFAULT_DIR

    # Alternance : deux evitements rapproches (< AVOID_ALTERNATE_T) ne se font jamais
    # du meme cote -> droite/gauche/droite... Sauf si un cote est BEAUCOUP plus degage
    # (mur de l'autre cote) : la securite prime alors sur l'alternance.
    global _last_avoid_dir, _last_avoid_time
    now = time.time()
    strong = abs(d_right - d_left) >= AVOID_ALTERNATE_MARGIN_MM
    if (not strong and _last_avoid_dir != 0
            and now - _last_avoid_time < AVOID_ALTERNATE_T
            and turn_dir == _last_avoid_dir):
        turn_dir = -_last_avoid_dir
    _last_avoid_dir = turn_dir
    _last_avoid_time = now

    _upd('avoid_dir', turn_dir)
    return turn_dir

def _initial_search_side():
    """Cote de depart pour la toute PREMIERE recherche de porte d'une mission
    (avant qu'un trajet ou un echec de scan ne donne un cote de reference).
    Base sur une vraie mesure sonar gauche/droite au lieu d'etre code en dur
    a droite (sinon chaque mission commencait systematiquement en regardant
    a droite, peu importe l'environnement)."""
    d_right = scan_side(US_RIGHT)
    d_left = scan_side(US_LEFT)
    set_us(US_FORWARD)
    return 1 if d_right >= d_left else -1

def _map_corridor_dist():
    """Distance frontale du plus proche obstacle MEMORISE dans le corridor devant
    le robot, meme s'il est sorti du faisceau sonar. VIEW_MAX_MM si rien.

    Repere carte : x = lateral (+ = droite), y = avant. Sert a completer le sonar
    live pour ne pas foncer dans un objet vu il y a peu mais hors champ maintenant.
    """
    best = float(VIEW_MAX_MM)
    for x, y, conf, confirmed in world.snapshot():
        if y <= 0 or y > MAP_LOOKAHEAD_MM:
            continue
        if abs(x) > MAP_CORRIDOR_HALF_MM:
            continue
        if not confirmed and conf < MAP_MIN_CONF:
            continue
        if y < best:
            best = y
    return best

def _sweep_front_sector():
    """Balaye la tete sur le secteur avant (SWEEP_RAWS) PENDANT que le robot roule
    (coast) et renvoie (dmin, best_dir). Modele-independant : on re-mesure la
    realite au lieu de faire confiance a la carte.

    dmin      : distance mini valide du secteur (VIEW_MAX_MM si rien).
    best_dir  : +1 si le plus degage est a droite, -1 a gauche, 0 pile devant.
    Retourne None si la ligne est vue (l'appelant traite la frontiere) ou a l'arret.
    """
    dmin = float(VIEW_MAX_MM)
    best_d, best_raw = -1.0, US_FORWARD
    for raw in SWEEP_RAWS:
        if not _running():
            return None
        set_us(raw)
        if not interruptible_sleep(SWEEP_SETTLE, step=0.01, watch_line=True):
            return None                      # ligne vue (ou arret)
        d = checkdist()
        d = VIEW_MAX_MM if d is None else _clamp(float(d), 0.0, VIEW_MAX_MM)
        _upd('us_dist', d)
        _record_obstacle(d)                  # alimente la carte (pour la vue)
        if SonarFilter.valid(d) and d < dmin:
            dmin = d
        if d > best_d:
            best_d, best_raw = d, raw
    set_us(US_FORWARD)
    best_dir = 1 if best_raw < US_FORWARD else (-1 if best_raw > US_FORWARD else 0)
    return dmin, best_dir


# ================================================================ primitives de mouvement

def guarded_motion(duration, speed, direction, steer_angle,
                   stop_at_end=True, stop_on_obstacle_mm=None):
    """Execute un mouvement court et interruptible.

    En marche avant, la ligne est lue avant de commander les moteurs puis a
    chaque CONTROL_DT. Au premier capteur noir : stop immediat et retour
    MOVE_LINE. En marche arriere on ne peut pas franchir la ligne frontale :
    le mouvement reste surtout surveille pour l'arret clavier.
    """
    if not _running():
        safe_stop_outputs(center=False)
        return MOVE_STOPPED

    if direction > 0 and line_seen():
        safe_stop_outputs(center=False)
        return MOVE_LINE

    if not safe_drive(speed, direction, steer_angle):
        return MOVE_STOPPED

    end = time.time() + duration
    last = time.time()
    next_sonar = last
    while _running():
        now = time.time()
        if now >= end:
            break

        if direction > 0 and line_seen():
            safe_stop_outputs(center=False)
            return MOVE_LINE

        if direction > 0 and stop_on_obstacle_mm is not None and now >= next_sonar:
            set_us(US_FORWARD)
            d_front = sonar_filter.read(
                samples=SONAR_SAMPLES_SIDE,
                sticky=False,
                watch_line=(direction > 0)
            )
            if d_front is None:
                return MOVE_LINE
            # sonar live complete par la memoire : un obstacle hors faisceau mais
            # encore dans le corridor doit aussi stopper le mouvement.
            d_eff = min(d_front, _map_corridor_dist())
            if stop_on_obstacle_mm is not None and d_eff < stop_on_obstacle_mm:
                safe_stop_outputs(center=False)
                return MOVE_OBSTACLE
            next_sonar = now + 0.09

        time.sleep(min(CONTROL_DT, end - now))
        now2 = time.time()
        advance_world_from_motion(now2 - last)
        last = now2

    if not _running():
        safe_stop_outputs(center=False)
        return MOVE_STOPPED

    if stop_at_end:
        safe_stop_outputs(center=False)
    return MOVE_DONE


# ================================================================ manoeuvres

def boundary_turn_dir(l, _m, r):
    if l and not r:
        return 1       # ligne a gauche -> nez vers la droite en reculant
    if r and not l:
        return -1      # ligne a droite -> nez vers la gauche en reculant
    avoid_dir = _snap().get('avoid_dir', 0)
    return avoid_dir if avoid_dir else AVOID_DEFAULT_DIR

def _boundary_kind(l, m, r):
    """Classe le contact de bord.

    - frontal : le capteur milieu touche, ou les deux exterieurs a la fois ->
      on arrive quasiment de face, impossible d'avancer, il faut reculer.
    - effleurement : un seul capteur exterieur touche -> le robot rase le bord,
      on peut le decoller en avancant en arc vers l'interieur.
    """
    frontal = bool(m) or (l and r)
    graze = (bool(l) or bool(r)) and not frontal
    return frontal, graze

def _boundary_reverse(turn_dir):
    """Recul en arc pour degager le nez du bord (arrivee de face ou echec du
    decollement). En marche arriere le braquage accumule une rotation Ackermann
    qui reoriente le robot vers turn_dir."""
    reverse_steer = STEER_LEFT if turn_dir > 0 else STEER_RIGHT
    return guarded_motion(BOUNDARY_REVERSE_T, SPEED_REVERSE, -1, reverse_steer)

def _boundary_nudge(turn_dir):
    """Effleurement : petit arc AVANT qui ecarte le nez du bord sans reculer.

    On surveille uniquement le capteur milieu : s'il touche, c'est qu'on fonce
    vraiment dans le bord -> retour MOVE_LINE pour que l'appelant bascule sur le
    recul. Des que les exterieurs se liberent, on s'est decolle : retour DONE.
    """
    if not _running():
        safe_stop_outputs(center=False)
        return MOVE_STOPPED
    nudge_steer = STEER_RIGHT if turn_dir > 0 else STEER_LEFT
    if not safe_drive(BOUNDARY_NUDGE_SPEED, 1, nudge_steer):
        return MOVE_STOPPED

    end = time.time() + BOUNDARY_NUDGE_T
    last = time.time()
    while _running():
        now = time.time()
        if now >= end:
            break
        l, m, r = read_boundary()
        if m:
            safe_stop_outputs(center=False)
            return MOVE_LINE
        if not (l or r):
            break                       # decolle du bord
        time.sleep(min(CONTROL_DT, end - now))
        now2 = time.time()
        advance_world_from_motion(now2 - last)
        last = now2

    if not _running():
        safe_stop_outputs(center=False)
        return MOVE_STOPPED
    safe_stop_outputs(center=False)
    return MOVE_DONE

def _boundary_inward_arc(turn_dir):
    """Reprise apres correction : au lieu de repartir tout droit (qui finit
    toujours par redriver vers un bord dans une zone courbe), on roule un court
    arc courbe vers l'interieur. Le biais est renforce si on rase le meme bord
    plusieurs fois de suite (_boundary_streak)."""
    if not _running():
        return
    _upd('mode', 'RECENTRAGE')
    extra = (_boundary_streak - 1) / float(max(1, BOUNDARY_STREAK_MAX - 1))
    scale = _clamp(BOUNDARY_INWARD_SCALE + 0.30 * extra, 0.0, 1.0)
    dur = BOUNDARY_INWARD_T * (1.0 + 0.40 * extra)
    inward_steer = STEER_CENTER + turn_dir * STEER_AMOUNT * scale
    res = guarded_motion(
        dur, BOUNDARY_INWARD_SPEED, 1, inward_steer,
        stop_at_end=True,
        stop_on_obstacle_mm=RECENTER_ABORT_MM
    )
    if res == MOVE_LINE:
        # retouche pendant la reprise : recul franc, sans re-enchainer d'arc
        # (evite toute recursion en cas de robot reellement coince)
        _boundary_reverse(turn_dir)
        safe_stop_outputs(center=False)

def handle_boundary(l, m, r):
    """Bord de zone. La ligne noire n'est jamais franchie.

    - effleurement (un exterieur) -> correction douce en avant (_boundary_nudge)
    - arrivee de face (milieu, ou deux capteurs) -> recul en arc
    Dans les deux cas on repart en courbant vers l'interieur, avec un biais
    renforce si le meme bord est rase plusieurs fois de suite (anti-collage).
    """
    global _boundary_last_dir, _boundary_streak
    _upd('mode', 'FRONTIERE')
    turn_dir = boundary_turn_dir(l, m, r)
    _upd('avoid_dir', turn_dir)
    set_us(US_FORWARD)

    if turn_dir == _boundary_last_dir:
        _boundary_streak = min(BOUNDARY_STREAK_MAX, _boundary_streak + 1)
    else:
        _boundary_streak = 1
        _boundary_last_dir = turn_dir

    frontal, graze = _boundary_kind(l, m, r)
    if graze:
        res = _boundary_nudge(turn_dir)
        if res == MOVE_STOPPED:
            return
        if res == MOVE_LINE:            # l'effleurement a vire au contact franc
            safe_stop_outputs(center=False)
            if _boundary_reverse(turn_dir) == MOVE_STOPPED:
                return
    else:
        safe_stop_outputs(center=False)
        if _boundary_reverse(turn_dir) == MOVE_STOPPED:
            return

    if not _running():
        safe_stop_outputs(center=False)
        return

    _boundary_inward_arc(turn_dir)
    safe_stop_outputs(center=False)
    set_us(US_FORWARD)

    # Arme le biais interieur pour la croisiere qui suit : plus le bord est rase
    # de fois de suite, plus le maintien vers le centre dure longtemps.
    global _inward_bias_dir, _inward_bias_until
    _inward_bias_dir = turn_dir
    _inward_bias_until = time.time() + BOUNDARY_BIAS_T * (
        1.0 + 0.5 * (_boundary_streak - 1) / float(max(1, BOUNDARY_STREAK_MAX - 1))
    )

def handle_recenter(turn_dir):
    """Compense la derive laterale estimee pendant l'evitement.

    L'odometrie est open-loop, donc on ne pretend pas mesurer 10 cm exactement.
    Mais on garde une "dette" laterale : si les arcs precedents ont pousse le
    robot vers la droite, on rembourse par des petits arcs vers la gauche jusqu'a
    revenir pres de l'axe estime. turn_dir reste un fallback si l'estimation est
    trop faible ou indisponible.
    """
    _upd('mode', 'RECENTRAGE')
    lateral = float(_snap().get('sim_lateral', 0.0))
    if abs(lateral) < RECENTER_DEADBAND_MM:
        # Fallback ancien comportement : petit contre-arc apres un evitement
        # dont l'estimation laterale n'a pas assez bouge.
        if not turn_dir:
            safe_stop_outputs(center=True)
            set_us(US_FORWARD)
            return
        recenter_steer = STEER_CENTER - turn_dir * STEER_AMOUNT * RECENTER_STEER_SCALE
        res = guarded_motion(
            RECENTER_T, RECENTER_SPEED, 1, recenter_steer,
            stop_at_end=True,
            stop_on_obstacle_mm=RECENTER_ABORT_MM
        )
        if res == MOVE_LINE:
            l, m, r = read_boundary()
            handle_boundary(l, m, r)
        safe_stop_outputs(center=True)
        set_us(US_FORWARD)
        return

    start_t = time.time()
    while _running():
        lateral = float(_snap().get('sim_lateral', 0.0))
        if abs(lateral) <= RECENTER_DEADBAND_MM:
            break
        if time.time() - start_t >= RECENTER_MAX_T:
            break

        correction_dir = -1 if lateral > 0 else 1
        recenter_steer = STEER_CENTER + correction_dir * STEER_AMOUNT * RECENTER_STEER_SCALE
        res = guarded_motion(
            RECENTER_STEP_T, RECENTER_SPEED, 1, recenter_steer,
            stop_at_end=False,
            stop_on_obstacle_mm=RECENTER_ABORT_MM
        )
        if res == MOVE_LINE:
            l, m, r = read_boundary()
            handle_boundary(l, m, r)
            return
        if res in (MOVE_STOPPED, MOVE_OBSTACLE):
            break

    safe_stop_outputs(center=True)
    set_us(US_FORWARD)

def _turn_allowed(turn_dir, ref_heading):
    """Anti demi-tour : refuse de continuer a tourner du meme cote si le robot a
    deja pivote de plus de HEADING_MAX_DEG DEPUIS le debut de la manoeuvre (ref).
    Reference locale (et non l'axe de depart) pour rester valable en zone courbe.
    +heading = gauche ; turn_dir +1 = droite (fait diminuer le cap)."""
    dh = math.degrees(_heading - ref_heading)
    if turn_dir > 0 and dh <= -HEADING_MAX_DEG:
        return False
    if turn_dir < 0 and dh >= HEADING_MAX_DEG:
        return False
    return True

def _realign_to_axis(ref_heading):
    """Ramene le cap vers celui d'avant l'obstacle, sonar surveille (au lieu d'un
    contre-braquage chronometre a l'aveugle)."""
    _upd('mode', 'RECENTRAGE')
    steps = 0
    while _running() and steps < AVOID_MAX_STEPS:
        steps += 1
        dh = math.degrees(_heading - ref_heading)
        if abs(dh) <= HEADING_ALIGN_DEG:
            return MOVE_DONE
        align_steer = STEER_RIGHT if dh > 0 else STEER_LEFT   # dh>0 (trop a gauche) -> braquer droite
        res = guarded_motion(AVOID_STEP_T, SPEED_AVOID, 1, align_steer,
                             stop_at_end=False, stop_on_obstacle_mm=AVOID_REACT_MM)
        if res != MOVE_DONE:
            return res
    return MOVE_DONE

def handle_obstacle(initial_dist):
    """Evitement reactif, sonar lu en continu.

    Phase 1 : tourner vers le cote degage EN AVANCANT (l'Ackermann a besoin de
    rouler pour braquer) jusqu'a ce que le front soit libre. On ne s'arrete que
    sur quasi-collision (<AVOID_EMERG_MM) -> petit recul puis on repart.
    Phase 2 : avancer ~droit pour depasser l'obstacle.
    Phase 3 : se realigner sur le cap d'avant-obstacle.
    La rotation est bornee (HEADING_MAX_DEG) pour interdire tout demi-tour ; si le
    cap sature d'un cote, on bascule de l'autre.
    """
    _upd('mode', 'OBSTACLE' if initial_dist < OBSTACLE_CRITICAL_MM else 'EVITEMENT')
    safe_stop_outputs(center=True)
    ref_heading = _heading                       # cap de reference AVANT contournement
    turn_dir = choose_clear_side()
    if not _running():
        return
    if not _turn_allowed(turn_dir, ref_heading):
        turn_dir = -turn_dir

    if initial_dist < OBSTACLE_CRITICAL_MM:
        reverse_steer = STEER_LEFT if turn_dir > 0 else STEER_RIGHT
        if guarded_motion(OBSTACLE_BACKUP_T, SPEED_REVERSE, -1, reverse_steer) == MOVE_STOPPED:
            return

    # Phase 1 : tourner en avancant jusqu'a degager le front.
    steps = 0
    stuck_hits = 0
    while _running() and steps < AVOID_MAX_STEPS:
        steps += 1
        if not _turn_allowed(turn_dir, ref_heading):
            turn_dir = -turn_dir                 # anti demi-tour : bascule de cote
            stuck_hits = 0
        arc_steer = STEER_RIGHT if turn_dir > 0 else STEER_LEFT
        res = guarded_motion(AVOID_STEP_T, SPEED_AVOID, 1, arc_steer,
                             stop_at_end=False, stop_on_obstacle_mm=AVOID_EMERG_MM)
        if res == MOVE_LINE:
            l, m, r = read_boundary()
            handle_boundary(l, m, r)
            return
        if res == MOVE_STOPPED:
            return
        if res == MOVE_OBSTACLE:                 # quasi-collision : petit recul, puis on retourne
            reverse_steer = STEER_LEFT if turn_dir > 0 else STEER_RIGHT
            if guarded_motion(OBSTACLE_BACKUP_T, SPEED_REVERSE, -1, reverse_steer) == MOVE_STOPPED:
                return
            stuck_hits += 1
            if stuck_hits >= AVOID_STUCK_FLIPS:
                turn_dir = -turn_dir
                stuck_hits = 0
            continue
        stuck_hits = 0
        # MOVE_DONE : rien de proche dans l'axe courant -> le front est-il degage ?
        set_us(US_FORWARD)
        d_front = sonar_filter.read(samples=SONAR_SAMPLES_SIDE, sticky=False, watch_line=True)
        if d_front is None:
            l, m, r = read_boundary()
            handle_boundary(l, m, r)
            return
        if not _running():
            return
        if d_front > OBSTACLE_TRIGGER_MM:
            break                                # voie devant degagee

    # Phase 2 : avancer ~droit pour depasser l'obstacle qu'on vient de contourner.
    res = guarded_motion(AVOID_PASS_T, SPEED_AVOID, 1, STEER_CENTER,
                         stop_at_end=False, stop_on_obstacle_mm=AVOID_REACT_MM)
    if res == MOVE_LINE:
        l, m, r = read_boundary()
        handle_boundary(l, m, r)
        return
    if res == MOVE_STOPPED:
        return
    # MOVE_OBSTACLE : un nouvel obstacle -> on laisse robot_loop le redetecter.

    # Phase 3 : se remettre dans l'axe d'avant-obstacle.
    if _realign_to_axis(ref_heading) == MOVE_LINE:
        l, m, r = read_boundary()
        handle_boundary(l, m, r)
        return

    # Phase 4 : rembourser le decalage lateral estime. Pour les "portes", c'est
    # ce qui evite d'enchainer les contournements toujours du meme cote jusqu'au
    # bord de la zone.
    handle_recenter(turn_dir)

    safe_stop_outputs(center=True)
    set_us(US_FORWARD)
    _upd('avoid_dir', 0)


def search_for_gate(prefer_dir):
    """Recherche active d'une porte du cote 'prefer_dir' (typiquement l'OPPOSE du
    dernier evitement). On tourne la TETE et les ROUES de ce cote et on avance
    doucement : la camera etant fixe (solidaire du chassis), c'est en faisant
    pivoter le chassis qu'on la fait "regarder" ce cote et trouver un autre
    passage (ex : a evite a droite -> cherche a gauche). Retourne True des qu'une
    porte fiable apparait.

    Meme convention de signe que choose_clear_side / handle_obstacle (repere
    "code", double-inverse mais coherent avec le materiel)."""
    if _remote_ai is None or not _running():
        return False
    _upd('mode', 'RECHERCHE')
    head_raw = US_RIGHT if prefer_dir > 0 else US_LEFT
    search_steer = STEER_RIGHT if prefer_dir > 0 else STEER_LEFT
    ref_heading = _heading
    for _ in range(GATE_SEARCH_MAX_STEPS):
        if not _running():
            break
        if _remote_ai.fresh_gate() is not None:
            set_us(US_FORWARD)
            return True
        # anti demi-tour : on ne pivote pas indefiniment du meme cote
        if abs(math.degrees(_heading - ref_heading)) >= HEADING_MAX_DEG:
            break
        # tete du cote recherche : regarde ou l'on va ET donne le degagement
        if scan_side(head_raw) < OBSTACLE_CRITICAL_MM:
            break                       # ce cote est bouche de trop pres
        set_us(head_raw)
        # stop_on_obstacle_mm : recentre la tete pour verifier le front pendant
        # l'arc (avant : aucune protection -> le robot pouvait foncer dedans).
        res = guarded_motion(GATE_SEARCH_STEP_T, GATE_SEARCH_SPEED, 1, search_steer,
                             stop_at_end=False, stop_on_obstacle_mm=OBSTACLE_CRITICAL_MM)
        if res == MOVE_LINE:
            l, m, r = read_boundary()
            handle_boundary(l, m, r)
            set_us(US_FORWARD)
            return False
        if res == MOVE_STOPPED:
            break
        if res == MOVE_OBSTACLE:
            d_front = sonar_filter.read(samples=SONAR_SAMPLES_SIDE, sticky=False, watch_line=True)
            set_us(US_FORWARD)
            if d_front is not None and _running():
                handle_obstacle(d_front)
            return False
    found = _remote_ai.fresh_gate() is not None
    set_us(US_FORWARD)
    return found


# ================================================================ boucle robot

def _remote_ai_endpoint(url):
    url = (url or '').strip()
    if not url:
        return ''
    if url.endswith('/'):
        url = url[:-1]
    return url if url.endswith('/detect') else url + '/detect'


def _reset_ai_state():
    with _lock:
        _state['ai_ok'] = 0
        _state['ai_gate'] = None
        _state['ai_width'] = None
        _state['ai_age'] = 999.0
        _state['ai_dets'] = 0
        _state['ai_ms'] = 0


class RemoteAIGuide:
    """Client camera -> laptop GPU.

    La boucle tourne en arriere-plan DES LE LANCEMENT du script (pas besoin
    d'appuyer sur M) : ca permet de voir le retour camera / les detections
    tout de suite, meme mission a l'arret. Elle ne commande jamais le robot
    directement : elle publie seulement le centre du passage detecte, que
    robot_loop utilise comme braquage doux (et seulement quand RUNNING).
    """
    def __init__(self, url):
        self.url = _remote_ai_endpoint(url)
        self._lock = Lock()
        self._stop = False
        self._started = False
        self._latest = {
            'ok': False,
            'gate': None,
            'width': None,
            'last': 0.0,
            'dets': 0,
            'ms': 0,
            'det_ang': [],
        }

    def start(self):
        if self._started or not self.url:
            return
        self._started = True
        Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self._stop = True

    def snapshot(self):
        with self._lock:
            s = dict(self._latest)
        s['age'] = time.time() - s.get('last', 0.0)
        return s

    def fresh_gate(self):
        s = self.snapshot()
        gate = s.get('gate')
        width = s.get('width')
        if not s.get('ok') or gate is None or width is None:
            return None
        if s['age'] > REMOTE_AI_MAX_AGE or width < REMOTE_AI_MIN_WIDTH_DEG:
            return None
        return s

    def rightmost_door(self, min_width_deg):
        """Angle (deg, + = droite) du centre de la porte la plus a DROITE : le
        PREMIER intervalle assez large entre deux objets en balayant de droite a
        gauche. None si rien de fiable / trop vieux / moins de 2 objets."""
        s = self.snapshot()
        if s['age'] > REMOTE_AI_MAX_AGE:
            return None
        objs = sorted(s.get('det_ang') or [], key=lambda t: t[1])   # gauche -> droite
        # on part de la paire la plus a DROITE et on descend vers la gauche
        for i in range(len(objs) - 2, -1, -1):
            gap = objs[i + 1][0] - objs[i][2]      # bord gauche du droit - bord droit du gauche
            if gap >= min_width_deg:
                return 0.5 * (objs[i][2] + objs[i + 1][0])
        return None

    def rightmost_door_full(self, min_width_deg):
        """Comme rightmost_door, mais renvoie (centre_deg, bouteille_gauche_deg,
        bouteille_droite_deg) : les centres des 2 bouteilles qui bordent la porte,
        pour mesurer leur distance au sonar. None si rien de fiable."""
        s = self.snapshot()
        if s['age'] > REMOTE_AI_MAX_AGE:
            return None
        objs = sorted(s.get('det_ang') or [], key=lambda t: t[1])   # gauche -> droite
        for i in range(len(objs) - 2, -1, -1):
            gap = objs[i + 1][0] - objs[i][2]
            if gap >= min_width_deg:
                center = 0.5 * (objs[i][2] + objs[i + 1][0])
                return (center, objs[i][1], objs[i + 1][1])
        return None

    def extremity_door(self, min_width_deg, side):
        """Porte a l'EXTREMITE demandee : side +1 = la plus a DROITE, -1 = la plus a
        GAUCHE. Renvoie (centre_deg, bouteille_gauche_deg, bouteille_droite_deg) ou
        None. On calcule tous les intervalles assez larges entre bouteilles
        adjacentes, puis on garde celui dont le centre est le plus du cote 'side'."""
        s = self.snapshot()
        if s['age'] > REMOTE_AI_MAX_AGE:
            return None
        objs = sorted(s.get('det_ang') or [], key=lambda t: t[1])   # gauche -> droite
        best = None
        for i in range(len(objs) - 1):
            gap = objs[i + 1][0] - objs[i][2]
            if gap < min_width_deg:
                continue
            center = 0.5 * (objs[i][2] + objs[i + 1][0])
            if best is None or (center > best[0] if side >= 0 else center < best[0]):
                best = (center, objs[i][1], objs[i + 1][1])
        return best

    def _publish(self, ok, gate=None, width=None, dets=0, ms=0, det_ang=None):
        now = time.time()
        with self._lock:
            self._latest = {
                'ok': bool(ok),
                'gate': gate,
                'width': width,
                'last': now,
                'dets': int(dets),
                'ms': int(ms or 0),
                'det_ang': det_ang or [],
            }
        with _lock:
            _state['ai_ok'] = 1 if ok else 0
            _state['ai_gate'] = gate
            _state['ai_width'] = width
            _state['ai_age'] = 0.0
            _state['ai_dets'] = int(dets)
            _state['ai_ms'] = int(ms or 0)

    @staticmethod
    def _open_camera():
        import cv2
        from picamera2 import Picamera2

        cam = Picamera2()
        cfg = cam.preview_configuration
        cfg.size = (REMOTE_AI_W, REMOTE_AI_H)
        cfg.format = "RGB888"
        cam.configure("preview")
        cam.start()
        time.sleep(0.35)
        return cam, cv2

    @staticmethod
    def _jpeg_b64(cv2, bgr):
        ok, enc = cv2.imencode(
            ".jpg",
            bgr,
            [int(cv2.IMWRITE_JPEG_QUALITY), REMOTE_AI_JPEG_QUALITY]
        )
        if not ok:
            raise RuntimeError("encodage JPEG impossible")
        return base64.b64encode(enc.tobytes()).decode('ascii')

    def _call_server(self, image_b64):
        payload = {
            'image_b64': image_b64,
            'hfov_deg': REMOTE_AI_HFOV_DEG,
            'debug': False,
        }
        req = urllib.request.Request(
            self.url,
            data=json.dumps(payload).encode('utf-8'),
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        with urllib.request.urlopen(req, timeout=REMOTE_AI_TIMEOUT) as resp:
            return json.loads(resp.read().decode('utf-8'))

    def _loop(self):
        cam = None
        cv2 = None
        next_try = 0.0
        while not self._stop and not _exit_flag:
            try:
                if cam is None:
                    if time.time() < next_try:
                        time.sleep(0.15)
                        continue
                    cam, cv2 = self._open_camera()

                rgb = cam.capture_array()
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                ans = self._call_server(self._jpeg_b64(cv2, bgr))
                dets_list = ans.get('detections') or []
                det_ang = [
                    (d['angle_left_deg'], d['angle_center_deg'], d['angle_right_deg'])
                    for d in dets_list
                    if 'angle_left_deg' in d and 'angle_center_deg' in d
                    and 'angle_right_deg' in d
                ]
                gate = ans.get('gate') or {}
                ok = bool(ans.get('ok')) and bool(gate.get('ok'))
                self._publish(
                    ok,
                    gate.get('gate_center_deg') if ok else None,
                    gate.get('gate_width_deg') if ok else None,
                    len(dets_list),
                    ans.get('inference_ms', 0),
                    det_ang,
                )
                time.sleep(REMOTE_AI_PERIOD)
            except Exception as e:
                self._publish(False)
                print('Remote AI indisponible:', e)
                if cam is not None:
                    try:
                        cam.stop()
                        cam.close()
                    except Exception:
                        pass
                cam = None
                cv2 = None
                next_try = time.time() + 2.0

        if cam is not None:
            try:
                cam.stop()
                cam.close()
            except Exception:
                pass


_remote_ai = None


def _ensure_remote_ai():
    global _remote_ai
    if not REMOTE_AI_URL:
        return None
    if _remote_ai is None:
        _remote_ai = RemoteAIGuide(REMOTE_AI_URL)
        _remote_ai.start()
        print('Remote AI active:', _remote_ai.url)
    return _remote_ai

def start_zone():
    global robot_state, _hardware_ready, _boundary_last_dir, _boundary_streak
    global _inward_bias_dir, _inward_bias_until, _heading, _last_avoid_dir, _last_avoid_time, _last_go_dir
    with _drive_lock:
        if robot_state != STOPPED:
            return
        robot_state = STARTING

    setup()
    _hardware_ready = True
    safe_stop_outputs(center=True)
    set_us(US_FORWARD)
    _boundary_last_dir = 0
    _boundary_streak = 0
    _inward_bias_dir = 0
    _inward_bias_until = 0.0
    _last_avoid_dir = 0
    _last_avoid_time = 0.0
    _last_go_dir = 0
    _heading = 0.0
    _clear_obstacles()
    sonar_filter.reset()
    _reset_ai_state()
    _ensure_remote_ai()
    _upd('mode', 'SCAN')
    set_us(US_FORWARD)               # tete devant, pas de balayage : la camera percoit
    with _drive_lock:
        if robot_state != STARTING or _exit_flag:
            _motor_stop_unlocked()
            robot_state = STOPPED
            _upd('mode', 'ARRET')
            return
        robot_state = RUNNING
    _upd('mode', 'CROISIERE')

def stop_zone():
    global robot_state
    with _drive_lock:
        robot_state = STOPPED
        _motor_stop_unlocked()
        steer(STEER_CENTER)
        set_us(US_FORWARD)
    _upd('mode', 'ARRET')
    _upd('target_steer', STEER_CENTER)
    _upd('avoid_dir', 0)

def pick_rightmost_door():
    """Angle (deg, + = droite) de la porte la plus a DROITE vue par la camera,
    ou None. (Detail du choix : RemoteAIGuide.rightmost_door.)"""
    if _remote_ai is None:
        return None
    return _remote_ai.rightmost_door(REMOTE_AI_MIN_WIDTH_DEG)


def turn_body(cam_dir, dur):
    """Fait pivoter le CHASSIS vers cam_dir (+1 = droite camera, -1 = gauche) en
    roulant lentement (Ackermann : il faut avancer pour tourner), TETE FIXE devant.
    La ligne noire est prioritaire, et un obstacle qui apparait pendant le
    redressement declenche un evitement au lieu d'etre fonce dedans."""
    if cam_dir == 0 or not _running():
        return
    set_us(US_FORWARD)
    steer_angle = STEER_CENTER + REMOTE_AI_STEER_SIGN * cam_dir * STEER_AMOUNT
    res = guarded_motion(dur, GATE_SEARCH_SPEED, 1, steer_angle, stop_at_end=True,
                         stop_on_obstacle_mm=OBSTACLE_CRITICAL_MM)
    if res == MOVE_LINE:
        l, m, r = read_boundary()
        handle_boundary(l, m, r)
    elif res == MOVE_OBSTACLE:
        d_front = sonar_filter.read(samples=SONAR_SAMPLES_SIDE, sticky=False, watch_line=True)
        if d_front is not None and _running():
            handle_obstacle(d_front)


def drive_to_door(door_deg, go_t=GO_T):
    """ETAPE AVANCE : roule vers la porte pendant go_t s, cap FIXE sur l'angle
    trouve au scan (on ne rechoisit PAS de porte en route), TETE FIXE devant.
    S'arrete net sur ligne, et EVITE si un obstacle apparait en route (au lieu
    de juste stopper et laisser la boucle rescanner sur place). Renvoie le
    cote suivi (+1 droite, -1 gauche, 0 tout droit)."""
    _upd('mode', 'AVANCE')
    set_us(US_FORWARD)
    gate_deg = _clamp(door_deg, -REMOTE_AI_STEER_MAX_DEG, REMOTE_AI_STEER_MAX_DEG)
    followed = 1 if gate_deg > 0 else (-1 if gate_deg < 0 else 0)
    _upd('avoid_dir', followed)
    steer_angle = STEER_CENTER + REMOTE_AI_STEER_SIGN * (
        gate_deg / REMOTE_AI_STEER_MAX_DEG
    ) * STEER_AMOUNT * REMOTE_AI_STEER_SCALE
    res = guarded_motion(go_t, SPEED_CRUISE, 1, steer_angle,
                         stop_at_end=True, stop_on_obstacle_mm=OBSTACLE_CRITICAL_MM)
    if res == MOVE_LINE:
        l, m, r = read_boundary()
        handle_boundary(l, m, r)
    elif res == MOVE_OBSTACLE:
        set_us(US_FORWARD)
        d_front = sonar_filter.read(samples=SONAR_SAMPLES_SIDE, sticky=False, watch_line=True)
        if d_front is not None and _running():
            handle_obstacle(d_front)
    return followed


def _wait_fresh_door(side, timeout=1.5):
    """ETAPE SCAN (a l'arret) : attend jusqu'a timeout s une image camera FRAICHE,
    puis renvoie la porte a l'extremite 'side' (+1 = la plus a droite, -1 = la plus
    a gauche) : (centre, bouteille_gauche, bouteille_droite). None si rien vu / ligne.
    Le robot doit deja etre A L'ARRET quand on appelle (aucun mouvement ici)."""
    if _remote_ai is None:
        return None
    end = time.time() + timeout
    while _running() and time.time() < end:
        if line_seen():
            return None
        snap = _remote_ai.snapshot()
        _upd('ai_age', snap.get('age', 999.0))
        if snap.get('age', 999.0) <= REMOTE_AI_MAX_AGE:
            info = _remote_ai.extremity_door(REMOTE_AI_MIN_WIDTH_DEG, side)
            if info is not None:
                return info
        time.sleep(0.05)
    return None


def robot_loop():
    """Boucle decomposee, STOP entre chaque etape :
      1) STOP net (roues centrees, tete devant)
      2) securite sonar : un obstacle deja tres proche pile devant passe AVANT
         toute recherche de porte -> evitement immediat (handle_obstacle)
      3) SCAN camera (si dispo) a l'arret -> porte a l'extremite OPPOSEE au
         trajet precedent ; si rien dans le champ fixe, RECHERCHE ACTIVE
         (search_for_gate fait pivoter la TETE et le CHASSIS de ce cote pour
         regarder ailleurs) avant de retenter
      4) si une porte est trouvee : AVANCER vers elle (avec evitement si un
         obstacle apparait en route) puis TOURNER pour se remettre droit
      5) sinon (pas d'IA configuree, ou toujours rien apres la recherche) :
         croisiere reactive au sonar seul (avance tant que c'est degage,
         evite sinon) -> le robot ne reste jamais bloque a l'arret indefiniment.
    La ligne noire est toujours prioritaire."""
    global _last_go_dir
    motion_t = time.time()
    while not _exit_flag:
        now = time.time()
        advance_world_from_motion(now - motion_t)
        motion_t = now

        if robot_state != RUNNING:
            time.sleep(0.03)
            continue

        # --- ETAPE 1 : STOP net ---
        safe_stop_outputs(center=True)
        set_us(US_FORWARD)

        # securite : ligne noire prioritaire
        if line_seen():
            l, m, r = read_boundary()
            handle_boundary(l, m, r)
            continue

        # --- ETAPE 2 : securite sonar avant toute chose (ne jamais foncer dedans) ---
        d_front = sonar_filter.read(samples=SONAR_SAMPLES_SIDE, sticky=False, watch_line=True)
        if d_front is None:
            continue                        # ligne vue pendant la mesure
        if not _running():
            continue
        if d_front < OBSTACLE_CRITICAL_MM:
            handle_obstacle(d_front)
            continue

        # --- ETAPE 3 : SCAN camera (si dispo) -> porte a l'extremite INVERSE du dernier trajet ---
        info = None
        if _remote_ai is not None:
            _upd('mode', 'SCAN')
            side = -_last_go_dir if _last_go_dir != 0 else _initial_search_side()
            info = _wait_fresh_door(side)
            if info is None and search_for_gate(side):
                # trouvee pendant la recherche active (tete+chassis ont tourne)
                info = _remote_ai.extremity_door(REMOTE_AI_MIN_WIDTH_DEG, side)
            if info is None:
                # echec des deux cotes ce tour-ci : on alterne le cote scanne au
                # prochain essai (sinon _last_go_dir ne change qu'apres un passage
                # de porte REUSSI -> on restait bloque a rescanner le meme cote).
                _last_go_dir = side

        if info is not None:
            door, bottle_l, bottle_r = info

            # (option 3, a l'arret) distance sonar des 2 bouteilles -> duree d'avance
            dL = measure_at_angle(bottle_l)
            dR = measure_at_angle(bottle_r)
            cand = [d for d in (dL, dR) if SonarFilter.valid(d)]
            if cand:
                _upd('us_dist', min(cand))
                v = abs(_speed_to_mm_s(SPEED_CRUISE)) or 1.0
                go_t = _clamp((min(cand) + GATE_PASS_EXTRA_MM) / v, 0.8, GO_T)
            else:
                go_t = GO_T                 # rien de fiable au sonar -> avance pleine

            # --- ETAPE 4 : AVANCER vers la porte, puis se REDRESSER et RECENTRER ---
            ref_heading = _heading      # cap avant la manoeuvre : sert a corriger apres
            went = drive_to_door(door, go_t)
            if not _running():
                continue
            if went != 0:
                # redresse le CAP MESURE (pas un pulse aveugle a duree fixe comme
                # avant) : la courbe vers la porte devie le cap d'une quantite
                # variable selon go_t/vitesse/angle -> on corrige jusqu'a revenir
                # pres du cap d'avant-manoeuvre, pour que le prochain scan camera
                # regarde bien devant (et pas de travers).
                if _realign_to_axis(ref_heading) == MOVE_LINE:
                    l, m, r = read_boundary()
                    handle_boundary(l, m, r)
                    continue
            # recentrage lateral estime (sim_lateral) : ramene le robot vers le
            # milieu du corridor au lieu de deriver indefiniment d'un cote.
            handle_recenter(went)
            _last_go_dir = went             # -> prochaine porte a l'extremite inverse
            _upd('avoid_dir', 0)
            continue

        # --- ETAPE 5 : pas de porte fiable (pas d'IA, ou rien trouve) -> croisiere sonar ---
        _upd('mode', 'CROISIERE')
        if d_front > OBSTACLE_TRIGGER_MM:
            res = guarded_motion(AVOID_PASS_T, SPEED_CRUISE, 1, STEER_CENTER,
                                 stop_at_end=False, stop_on_obstacle_mm=OBSTACLE_TRIGGER_MM)
            if res == MOVE_LINE:
                l, m, r = read_boundary()
                handle_boundary(l, m, r)
            elif res == MOVE_OBSTACLE:
                d2 = sonar_filter.read(samples=SONAR_SAMPLES_SIDE, sticky=False, watch_line=True)
                if d2 is not None and _running():
                    handle_obstacle(d2)
        else:
            handle_obstacle(d_front)


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
    'CROISIERE': (16,  160,  72),
    'ACTIF':     (16,  160,  72),
    'IA_PASSAGE': (145, 80, 190),
    'RECHERCHE': (120, 90, 200),
    'RECENTRAGE': (40, 150, 170),
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
            (OBSTACLE_TRIGGER_MM, 80, (220, 50, 47)),
            (OBSTACLE_CLEAR_MM, 55, GOOG),
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
        col = (220, 50, 47) if dist < OBSTACLE_TRIGGER_MM else GOOG
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
                    elif event.key == pygame.K_a and robot_state != STOPPED:
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
                elif cmd == 'A' and robot_state != STOPPED:
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
    if _remote_ai is not None:
        _remote_ai.stop()
    stop_zone()
    print('Nettoyage final')


if __name__ == '__main__':
    _ensure_remote_ai()          # retour camera dispo tout de suite, pas besoin de M
    Thread(target=robot_loop, daemon=True).start()

    use_gui = '--gui' in sys.argv and PYGAME_OK   # Vision 2D OFF par defaut (--gui pour l'activer)
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
