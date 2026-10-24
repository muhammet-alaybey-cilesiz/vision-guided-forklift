"""se2.py — düzlemde poz (x, y, yaw): bileşke, ters, nokta dönüşümü."""

import math
from dataclasses import dataclass

import numpy as np


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass(frozen=True)
class SE2:
    x: float
    y: float
    yaw: float

    def __mul__(self, o):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return SE2(self.x + c * o.x - s * o.y, self.y + s * o.x + c * o.y, wrap(self.yaw + o.yaw))

    def inv(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return SE2(-c * self.x - s * self.y, s * self.x - c * self.y, -self.yaw)

    def apply(self, p):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.array([self.x + c * p[0] - s * p[1], self.y + s * p[0] + c * p[1]])

    @property
    def xy(self):
        return np.array([self.x, self.y])
