#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Lecture propre de la tension batterie via l'ADC ADS7830 (I2C 0x48, canal 0).
# Extrait minimal de web/Voltage.py, sans OLED ni buzzer.
#
#   tension = (adc/255) * Vref / DivisionRatio
#   pont diviseur R15=3000 / R17=1000 -> DivisionRatio = 0.25
#
# read_voltage() renvoie une mediane filtree (robuste au bruit), ou None si
# l'ADC n'est pas accessible (ex. execution hors robot).

import time
import statistics

try:
    import smbus
    _bus = smbus.SMBus(1)
except Exception:
    _bus = None

_ADDR    = 0x48
_CMD     = 0x84
ADCVref  = 4.93
R15, R17 = 3000.0, 1000.0
DIV      = R17 / (R15 + R17)     # 0.25
# Reperes (cf. README_batterie.md) :
FULL_V   = 8.4                   # pleine charge 2S Li-ion/LiPo
EMPTY_V  = 6.3                   # zone basse prudente -> 0 %
WARN_V   = 6.0                   # sous ce seuil : arreter les essais et recharger
MEAS_MIN_V = 5.65               # minimum observe pendant les tests
MEAS_MAX_V = 7.8                # maximum observe pendant les tests

def _read_raw(chn=0):
    return _bus.read_byte_data(_ADDR, _CMD | (((chn << 2 | chn >> 1) & 0x07) << 4))

def read_voltage(samples=12):
    """Tension batterie en V (mediane filtree). None si ADC indisponible."""
    if _bus is None:
        return None
    vals = []
    for _ in range(samples):
        try:
            raw = _read_raw(0)
        except Exception:
            return None
        vals.append(raw / 255.0 * ADCVref / DIV)
        time.sleep(0.02)
    med = statistics.median(vals)
    good = [v for v in vals if abs(v - med) < 1.0]   # rejette les aberrations
    return sum(good) / len(good) if good else med

def percentage(v=None):
    """% indicatif (lineaire tension, approximatif vu la courbe LiPo)."""
    if v is None:
        v = read_voltage()
    if v is None:
        return None
    return max(0.0, min(100.0, (v - EMPTY_V) / (FULL_V - EMPTY_V) * 100.0))

def is_low(v=None):
    """Vrai si la tension est sous le seuil de securite WARN_V (arret conseille)."""
    if v is None:
        v = read_voltage()
    return v is not None and v < WARN_V

if __name__ == "__main__":
    v = read_voltage()
    if v is None:
        print("ADC indisponible (hors robot ?)")
    else:
        msg = "Tension : %.2f V   (~%.0f %%)" % (v, percentage(v))
        if is_low(v):
            msg += "   /!\\ SOUS %.1f V : arreter et recharger" % WARN_V
        print(msg)
