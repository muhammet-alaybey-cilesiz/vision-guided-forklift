"""
camera_model.py — iğne deliği kamera modeli ve düzlem izdüşümü

ROS'a bağlı değil: içsel parametreler camera_info'dan (K, D), dışsal parametre
TF'ten (base_footprint ← optik çerçeve) gelir; gerçek kamerada aynı kod
kalibrasyon dosyasıyla çalışır (docs/PLAN_CV_AUTONOMY.md, R3).

Çerçeveler
  optik : x sağ, y aşağı, z ileri (REP-103 optical frame)
  base  : base_footprint; x ileri, y sol, z yukarı, zemin z = 0
Piksel kuralı OpenCV'ninki: (u, v) = (sütun, satır), (0, 0) sol üst pikselin merkezi.
"""

import cv2
import numpy as np


class CameraModel:
    """Bir kameranın içsel (K, D) ve dışsal (R, t: base ← optik) modeli."""

    def __init__(self, K, D, width, height, R_base_opt, t_base_opt):
        self.K = np.asarray(K, float).reshape(3, 3)
        self.D = np.zeros(0) if D is None else np.asarray(D, float).ravel()
        self.width, self.height = int(width), int(height)
        self.R = np.asarray(R_base_opt, float).reshape(3, 3)   # sütunlar: optik eksenlerin base'teki yönü
        self.t = np.asarray(t_base_opt, float).reshape(3)      # optik merkez, base'te
        self.fx, self.fy = self.K[0, 0], self.K[1, 1]
        self.cx, self.cy = self.K[0, 2], self.K[1, 2]

    @classmethod
    def from_camera_info(cls, info, R_base_opt, t_base_opt):
        """sensor_msgs/CameraInfo (veya aynı alanlara sahip nesne) + dışsal poz."""
        return cls(info.k, info.d, info.width, info.height, R_base_opt, t_base_opt)

    @property
    def distorted(self):
        return self.D.size > 0 and bool(np.any(self.D != 0.0))

    # ------------------------------------------------------- 3B → piksel
    def to_optical(self, pts_base):
        """base noktaları (N×3) → optik çerçeve (N×3): p_opt = Rᵀ (p_base − t)."""
        return (np.atleast_2d(np.asarray(pts_base, float)) - self.t) @ self.R

    def project(self, pts_base):
        """base noktaları → (uv N×2, önde mi N, görüntüde mi N).

        Kameranın arkasındaki noktaların uv'si NaN'dır.
        """
        pc = self.to_optical(pts_base)
        front = pc[:, 2] > 1e-6
        uv = np.full((len(pc), 2), np.nan)
        if np.any(front):
            if self.distorted:
                img, _ = cv2.projectPoints(pc[front].reshape(-1, 1, 3), np.zeros(3), np.zeros(3), self.K, self.D)
                uv[front] = img.reshape(-1, 2)
            else:
                n = pc[front, :2] / pc[front, 2:3]
                uv[front] = np.column_stack([self.fx * n[:, 0] + self.cx, self.fy * n[:, 1] + self.cy])
        inside = front & (uv[:, 0] >= -0.5) & (uv[:, 0] <= self.width - 0.5) \
            & (uv[:, 1] >= -0.5) & (uv[:, 1] <= self.height - 0.5)
        return uv, front, inside

    # ------------------------------------------------------- piksel → 3B
    def pixel_rays(self, uv):
        """Pikseller (N×2) → base çerçevesinde birim ışın yönleri (N×3); ışınların başlangıcı self.t."""
        uv = np.atleast_2d(np.asarray(uv, float))
        if self.distorted:
            n = cv2.undistortPoints(uv.reshape(-1, 1, 2), self.K, self.D).reshape(-1, 2)
        else:
            n = np.column_stack([(uv[:, 0] - self.cx) / self.fx, (uv[:, 1] - self.cy) / self.fy])
        d = np.column_stack([n, np.ones(len(n))])
        d /= np.linalg.norm(d, axis=1, keepdims=True)
        return d @ self.R.T

    def pixels_to_plane(self, uv, z=0.0):
        """Pikselleri yatay z düzlemiyle kesiştirir → (noktalar N×3 base'te, geçerli N).

        Geçersiz: ışın düzleme paralel ya da düzlem kameranın arkasında.
        """
        d = self.pixel_rays(uv)
        with np.errstate(divide='ignore', invalid='ignore'):
            s = (z - self.t[2]) / d[:, 2]
        valid = np.isfinite(s) & (s > 0) & (np.abs(d[:, 2]) > 1e-9)
        pts = self.t + np.where(valid[:, None], s[:, None], np.nan) * d
        return pts, valid

    def ground_range(self, z=0.0):
        """Görüntü orta sütununun z düzleminde gördüğü en yakın ve en uzak x [m] (görmüyorsa NaN)."""
        uv = np.array([[self.cx, self.height - 0.5], [self.cx, -0.5]])
        pts, valid = self.pixels_to_plane(uv, z)
        return tuple(float(p[0]) if ok else float('nan') for p, ok in zip(pts, valid))
