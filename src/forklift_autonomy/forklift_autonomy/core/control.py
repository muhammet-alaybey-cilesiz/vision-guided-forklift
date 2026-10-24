"""
control.py — hareket ilkelleri (odometriyle) ve bölme ekseni servosu

Hepsi aynı arayüz: step(pose) → (v, w, done). pose: robotun odometri pozu (SE2).
Hızlar ve ivmeler autonomy.yaml'dan; DiffDrive kendi ivme sınırını ayrıca uygular.
"""

import math

import numpy as np

from forklift_autonomy.core.se2 import wrap


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class Turn:
    """Yerinde dönme: yaw hedefe ±tol yaklaşınca biter."""

    def __init__(self, target_yaw, cfg):
        self.target, self.c = target_yaw, cfg

    def step(self, pose):
        e = wrap(self.target - pose.yaw)
        if abs(e) < self.c['turn_tol']:
            return 0.0, 0.0, True
        w = clamp(self.c['k_turn'] * e, -self.c['w_max'], self.c['w_max'])
        if abs(w) < self.c['w_min']:
            w = math.copysign(self.c['w_min'], e)
        return 0.0, w, False


class Straight:
    """Başlangıç pozundan dist kadar düz (negatif = geri); başlangıç yönünü korur."""

    def __init__(self, start, dist, v, cfg):
        self.start, self.dist, self.v, self.c = start, dist, abs(v), cfg
        self.dir = np.array([math.cos(start.yaw), math.sin(start.yaw)])

    def step(self, pose):
        s = float(np.dot(pose.xy - self.start.xy, self.dir))
        rem = self.dist - s
        if abs(rem) < self.c['dist_tol'] or (self.dist > 0 and rem < 0) or (self.dist < 0 and rem > 0):
            return 0.0, 0.0, True
        v = math.copysign(clamp(self.c['k_dist'] * abs(rem), self.c['v_min'], self.v), rem)
        lat = float(np.cross(self.dir, pose.xy - self.start.xy))
        w = clamp(self.c['k_head'] * wrap(self.start.yaw - pose.yaw) - math.copysign(1, v) * self.c['k_lat'] * lat,
                  -self.c['w_max'], self.c['w_max'])
        return v, w, False


class GoTo:
    """Noktaya git: dön → düz git → hedef yöne dön (kısa yanal kaymalar için de)."""

    def __init__(self, target_xy, target_yaw, v, cfg):
        self.t, self.yaw, self.v, self.c = np.asarray(target_xy, float), target_yaw, v, cfg
        self.phase, self.prim = 0, None

    def step(self, pose):
        while True:
            if self.prim is None:
                d = self.t - pose.xy
                if self.phase == 0:
                    if np.linalg.norm(d) < self.c['goto_tol']:
                        self.phase = 2
                        continue
                    self.prim = Turn(math.atan2(d[1], d[0]), self.c)
                elif self.phase == 1:
                    self.prim = Straight(pose, float(np.linalg.norm(d)), self.v, self.c)
                elif self.phase == 2:
                    self.prim = Turn(self.yaw, self.c)
                else:
                    return 0.0, 0.0, True
            v, w, done = self.prim.step(pose)
            if not done:
                return v, w, False
            self.prim, self.phase = None, self.phase + 1


class BayServo:
    """Bölme ekseninde ilerleme (Stanley): yanal hata ve yön hatası sıfıra, durma mesafesine kadar.

    Eksen: giriş noktası E (ön çizgi) ve yön n (istasyonun içine), odometride. Mesafe: robot tabanından
    ön çizgiye eksen boyunca. stop_distance'a gelince biter."""

    def __init__(self, cfg, stop_distance, v_max):
        self.c, self.stop, self.v_max = cfg, stop_distance, v_max

    def errors(self, pose, entry, theta):
        n = np.array([math.cos(theta), math.sin(theta)])
        t = np.array([-n[1], n[0]])
        d = entry - pose.xy
        return float(np.dot(d, n)), float(-np.dot(d, t)), wrap(theta - pose.yaw)   # mesafe, yanal (+sol), yön

    def step(self, pose, entry, theta):
        dist, lat, eh = self.errors(pose, entry, theta)
        rem = dist - self.stop
        if rem <= 0:
            return 0.0, 0.0, True
        c = self.c
        v = clamp(c['k_dist'] * rem, c['v_final'], self.v_max)
        w = c['k_head'] * eh - math.atan2(c['k_stanley'] * lat, v + c['v_soft'])
        return v, clamp(w, -c['w_servo_max'], c['w_servo_max']), False
