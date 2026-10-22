"""
dataset.py — kaydedilmiş değerlendirme kümesini okur (f1_probe record çıktısı)

Her kare: iki görüntü, kayıt anındaki GT (robot pozu, çatal, koliler) ve kamera
modelleri. Kalibrasyon dizini verilirse içsel parametreler oradan gelir (D-59);
dışsal parametreler kayıttaki TF'ten.
"""

import json
import os

import cv2
import numpy as np

from forklift_evaluation.world_gt import Pose
from forklift_perception.core import calibration as cal
from forklift_perception.core.camera_model import CameraModel

SHORT = {'front_camera': 'front', 'lane_camera': 'lane'}


class Frame:
    def __init__(self, ds, rec):
        self.ds, self.rec = ds, rec
        self.id = rec['id']
        self.bf = Pose(rec['robot_bf']['p'], rec['robot_bf']['R'])
        self.fork_q = rec.get('fork_q') or 0.0
        self.parcels = {n: Pose(v['p'], v['R']) for n, v in rec.get('parcels', {}).items()}

    def rgb(self, cam):
        """cam: 'front_camera' | 'lane_camera' → RGB görüntü."""
        bgr = cv2.imread(os.path.join(self.ds.root, f'{self.id}_{SHORT[cam]}.png'))
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    def to_base(self, pts_world):
        inv = self.bf.inv()
        return np.asarray(pts_world, float) @ inv.R.T + inv.p


class Dataset:
    def __init__(self, root, calib_dir=None):
        self.root = root
        with open(os.path.join(root, 'meta.json')) as f:
            self.meta = json.load(f)
        self.world = self.meta['world']
        self.cams = {}
        for c, m in self.meta['cameras'].items():
            cam = CameraModel(m['K'], m['D'], m['size'][0], m['size'][1], m['R_base_opt'], m['t_base_opt'])
            if calib_dir:
                K, D, _, _ = cal.load_yaml(os.path.join(calib_dir, f'sim_{c}.yaml'))
                cam = cam.with_intrinsics(K, D)
            self.cams[c] = cam
        with open(os.path.join(root, 'index.jsonl')) as f:
            self.records = [json.loads(line) for line in f if line.strip()]

    def frames(self, sets=None):
        for r in self.records:
            if sets is None or r['set'] in sets:
                yield Frame(self, r)

    def __len__(self):
        return len(self.records)
