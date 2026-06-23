#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# APERCU / SIMULATION Windows : le robot AVANCE sur la route, des piliers
# apparaissent au hasard devant et arrivent vers lui ; l'ultrason balaie,
# les detecte, et la carte (monde virtuel) les suit puis les oublie derriere.
# Lancer :  python preview_vision.py
import sys, os, types, math, time, random
from threading import Thread

def _mk(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m

class _Dev:
    def __init__(self, *a, **k): self.value = 0

_mk('gpiozero', InputDevice=_Dev)
_mk('_03_servo', set_angle=lambda *a, **k: None, to_servo_angle=lambda x: x + 90)
_mk('_04_motor', setup=lambda *a, **k: None, stop=lambda *a, **k: None,
    drive=lambda *a, **k: None, CENTER_ANGLE=6)
_mk('_05_ultrason', checkdist=lambda: 2000)

here = os.path.dirname(os.path.abspath(__file__))
import importlib.util
spec = importlib.util.spec_from_file_location('mod', os.path.join(here, '_12_MissionBObstacle.py'))
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def _angdiff(a, b):
    return (a - b + math.pi) % (2 * math.pi) - math.pi

def _head_bearing(raw):
    rad = mod._raw_to_rad(raw)
    return math.atan2(math.cos(rad), -math.sin(rad))   # 0 = devant, + = droite


# --- monde reel simule (verite terrain, inconnue du robot) -----------
SPAWN_EVERY  = (1.4, 2.8)  # intervalle aleatoire entre apparitions (s)
SPAWN_Y      = 1450.0    # distance d'apparition devant (mm)
ROAD_HALF    = 380.0     # demi-largeur ou peuvent apparaitre les piliers (mm)
BEAM_HALF    = math.radians(7)
LINE_HALF_W  = 24.0       # demi-largeur de la ligne noire simulee (mm)
LINE_SENSORS = [(-20.0, 'l'), (0.0, 'm'), (20.0, 'r')]

sim_obs = []             # piliers reels : dict(x, y) en mm, repere robot


def _spawn():
    sim_obs.append({'x': random.uniform(-ROAD_HALF, ROAD_HALF), 'y': SPAWN_Y})


def _simulate_line(now):
    # Ligne noire virtuelle sous les capteurs IR. Elle ondule un peu pour
    # montrer quel capteur la voit: gauche, milieu, droite.
    line_x = 38.0 * math.sin(now * 0.85)
    for sensor_x, key in LINE_SENSORS:
        mod._state[key] = 1 if abs(sensor_x - line_x) <= LINE_HALF_W else 0
    mod.world.update_line(mod._state['l'], mod._state['m'], mod._state['r'])


def _simulate():
    # quelques piliers au depart
    for _ in range(2):
        sim_obs.append({'x': random.uniform(-ROAD_HALF, ROAD_HALF),
                        'y': random.uniform(700, 1400)})
    mod._state['mode'] = 'ACTIF'
    mod._state['speed'] = mod.SPEED_CRUISE
    mod._state['steer'] = mod.STEER_CENTER
    mod._state['sim_scroll'] = 0.0
    mod._state['sim_lateral'] = 0.0
    raw, direction = mod.US_FORWARD, 1
    next_spawn = time.time() + random.uniform(*SPAWN_EVERY)
    last = time.time()

    while not mod._exit_flag:
        now = time.time()
        dt  = now - last
        last = now
        sim_speed = mod._state.get('speed', mod.SPEED_CRUISE)
        if sim_speed <= 0:
            sim_speed = mod.SPEED_CRUISE
        ds = mod._speed_to_mm_s(sim_speed) * dt
        dtheta = mod._steer_to_yaw(ds, mod._state.get('steer', mod.STEER_CENTER))
        mod._state['sim_scroll'] = mod._state.get('sim_scroll', 0.0) + abs(ds)
        mod._advance_visual_lateral(ds, mod._state.get('steer', mod.STEER_CENTER), mod._state)

        # 1) le robot avance/tourne : la verite terrain bouge dans son repere
        for o in sim_obs:
            y = o['y'] - ds
            x = o['x']
            if dtheta:
                c, s = math.cos(-dtheta), math.sin(-dtheta)
                o['x'] = x * c - y * s
                o['y'] = x * s + y * c
            else:
                o['x'], o['y'] = x, y
        sim_obs[:] = [o for o in sim_obs if o['y'] > -300.0]
        # ... et le monde virtuel suit (ego-motion)
        mod.world.advance(ds, dtheta)

        # 2) apparitions aleatoires devant
        if now >= next_spawn:
            _spawn()
            next_spawn = now + random.uniform(*SPAWN_EVERY)

        # 3) balayage ultrason + detection
        raw += direction * 4
        if raw >= mod.US_LEFT:
            raw, direction = mod.US_LEFT, -1
        elif raw <= mod.US_RIGHT:
            raw, direction = mod.US_RIGHT, 1
        hb = _head_bearing(raw)
        dist = 2000.0
        for o in sim_obs:
            if o['y'] <= 0:
                continue
            ob = math.atan2(o['x'], o['y'])
            if abs(_angdiff(ob, hb)) < BEAM_HALF:
                dist = min(dist, math.hypot(o['x'], o['y']) + random.uniform(-15, 15))
        mod._state['us_angle'] = raw
        mod._state['us_dist']  = dist
        mod.world.update(raw, dist)
        _simulate_line(now)

        # 4) decision d'evitement affichee et reinjectee dans la cinematique
        avoid = mod.plan_avoidance()
        if avoid and avoid['closest_y'] < mod.AVOID_CRITICAL_Y_MM:
            mod._state['mode'], mod._state['speed'] = 'OBSTACLE', 0
            mod._state['steer'] = mod.STEER_CENTER
        elif avoid:
            mod._state['mode'], mod._state['speed'] = 'EVITEMENT', avoid['speed']
            mod._state['steer'] = avoid['steer']
        else:
            mod._state['mode'], mod._state['speed'] = 'ACTIF', 35
            mod._state['steer'] = mod.STEER_CENTER

        time.sleep(0.03)


if __name__ == '__main__':
    if not mod.PYGAME_OK:
        print("pygame manquant : pip install pygame-ce")
        sys.exit(1)
    print("Simulation : le robot avance, des piliers arrivent au hasard.")
    print("Ferme la fenetre ou Echap pour quitter.")
    Thread(target=_simulate, daemon=True).start()
    viz = mod.VizPygame()
    viz.run()
