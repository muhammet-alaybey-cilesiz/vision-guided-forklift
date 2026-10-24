"""
attitude.py — IMU'dan gövde eğimi (pitch) ve yuvarlanması (roll): tamamlayıcı süzgeç (D-67)

Gerçek IMU'daki gibi yalnız ivmeölçer ve jiroskop kullanılır (simülatörün hatasız yönelim
alanı kullanılmaz). Ivmeölçer duruşta yerçekimini ölçer: yavaş ama sürüklenmez. Jiroskop
hızlıdır ama integrali sürüklenir. Süzgeç ikisini zaman sabiti tau ile birleştirir.
Ivmeölçer sapması (bias) açılışta düz zeminde ölçülür ve çıkarılır (seviye kalibrasyonu).
Hareket kapısı: ivmeölçer duruşta yerçekimini, hızlanırken hızlanmayı da ölçer (DiffDrive'ın 1 m/s²'si
~6° sahte eğim, yerinde dönüşün açısal ivmesi ~2° sahte yuvarlanma). İvme büyüklüğü yerçekiminden
acc_tol'dan fazla saparsa ya da dönüş hızı gyro_tol'u aşarsa düzeltme yapılmaz, yalnız jiroskop
integrali sürer (F3: kapısız süzgeç sürüşte ölçümleri 10 mm'ye kadar kaydırdı).
Seviye kalibrasyonu aykırı örneklere dayanıklıdır: g ortanca (median) ile bulunur, sapan örnekler
eğim ortalamasına girmez (F3: Gazebo IMU'su arada sıfır ivme yayınlıyor; ortalamayla g 9.67 çıktı,
kapı hiç açılmadı ve süzgeç jiroskop sapmasıyla dakikada ~2° kaydı). Gerçek IMU'da da kopukluk olur.

İşaretler (gövde çerçevesi, REP-103): pitch > 0 burun yukarı, roll > 0 sol taraf yukarı.
"""

import math

import numpy as np


class AttitudeEstimator:
    def __init__(self, tau, level_samples, acc_tol=None, gyro_tol=None):
        self.tau = tau
        self.level_samples = level_samples
        self.acc_tol, self.gyro_tol = acc_tol, gyro_tol
        self.g = None                         # seviye kalibrasyonunda ölçülen ivme büyüklüğü
        self._lvl = []
        self._norm = []
        self.offset = None                   # (pitch, roll) seviye kalibrasyonu
        self.pitch = self.roll = None
        self.t = None

    @staticmethod
    def from_accel(a):
        ax, ay, az = a
        return math.atan2(ax, math.hypot(ay, az)), math.atan2(ay, az)

    def update(self, t, accel, gyro):
        pa, ra = self.from_accel(accel)
        if self.offset is None:
            self._lvl.append((pa, ra))
            self._norm.append(float(np.linalg.norm(accel)))
            if len(self._lvl) >= self.level_samples:
                norm = np.array(self._norm)
                self.g = float(np.median(norm))
                tol = self.acc_tol if self.acc_tol is not None else 0.1 * self.g
                ok = np.abs(norm - self.g) <= tol
                self.offset = tuple(np.mean(np.array(self._lvl)[ok], axis=0))
                self.pitch, self.roll, self.t = 0.0, 0.0, t     # kalibrasyon: düz duruyor
            return
        pa, ra = pa - self.offset[0], ra - self.offset[1]
        if t <= self.t:
            return
        dt = t - self.t
        self.t = t
        k = dt / (self.tau + dt)
        if self.acc_tol is not None and (abs(float(np.linalg.norm(accel)) - self.g) > self.acc_tol
                                         or float(np.linalg.norm(gyro)) > self.gyro_tol):
            k = 0.0                           # hareket: yalnız jiroskop
        # jiroskop: +y etrafında pozitif dönüş burnu aşağı indirir (pitch'i azaltır); +x sol tarafı kaldırır
        self.pitch = (1 - k) * (self.pitch - gyro[1] * dt) + k * pa
        self.roll = (1 - k) * (self.roll + gyro[0] * dt) + k * ra

    @property
    def ready(self):
        return self.pitch is not None

    def value(self):
        return (self.pitch, self.roll) if self.ready else None
