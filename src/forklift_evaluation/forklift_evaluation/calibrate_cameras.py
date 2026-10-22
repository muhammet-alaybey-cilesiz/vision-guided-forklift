"""
calibrate_cameras.py — robot kameralarını dama tahtasıyla kalibre eder (DECISIONS D-59)

Gerçek kamera kalibrasyonunun simdeki karşılığı: dama tahtası modeli simülasyona
eklenir, kameranın önünde farklı konum ve eğimlere taşınır; her karede köşeler
bulunur ve OpenCV calibrateCamera K ve D'yi kestirir (forklift_perception.core.calibration).
Dama tahtasının pozu yalnız onu görüş alanına koymak için kullanılır; kestirimde
GT kullanılmaz (gerçek kalibrasyonla aynı bilgi).

  ros2 run forklift_evaluation calibrate_cameras [--out DIR] [--install-to forklift_perception/config/calibration]
Çıktı: <out>/sim_<kamera>.yaml (ROS kalibrasyon biçimi), <out>/calibration.json, örnek görüntüler.
"""

import argparse
import json
import math
import os
import time

import cv2
import numpy as np

from forklift_evaluation import overlay as ov
from forklift_evaluation.ros_cams import RosCams
from forklift_evaluation.world_gt import EvalSetup, Pose, simt
from forklift_perception.core import calibration as cal

CAMS = ('front_camera', 'lane_camera')
COLS, ROWS, SQUARE = 9, 6, 0.025          # iç köşeler ve kare kenarı
PX = 80                                   # dokuda kare başına piksel
BOARD = 'calib_board'


def board_png(path):
    """(COLS+1) × (ROWS+1) kare + bir kare beyaz kenar."""
    nx, ny = COLS + 1, ROWS + 1
    img = np.full(((ny + 2) * PX, (nx + 2) * PX), 255, np.uint8)
    for j in range(ny):
        for i in range(nx):
            if (i + j) % 2 == 0:
                img[(j + 1) * PX:(j + 2) * PX, (i + 1) * PX:(i + 2) * PX] = 0
    cv2.imwrite(path, img)
    return (nx + 2) * SQUARE, (ny + 2) * SQUARE


def board_sdf(png, w, h):
    # Dinamik, yerçekimsiz, collision'sız (D-49'daki test kamerası gibi: set_pose ile taşınır).
    # Dokulu yüz −x: kameraya bakar (D-47: −x yüzü aynalanmadan okunur).
    return f'''<?xml version="1.0"?>
<sdf version="1.11"><model name="{BOARD}"><link name="link">
  <gravity>false</gravity>
  <inertial><mass>0.01</mass><inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz></inertia></inertial>
  <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.002 {w} {h}</size></box></geometry>
    <material><diffuse>1 1 1 1</diffuse><ambient>1 1 1 1</ambient><specular>0 0 0 1</specular>
      <pbr><metal><albedo_map>{png}</albedo_map><roughness>1</roughness><metalness>0</metalness></metal></pbr>
    </material></visual>
</link></model></sdf>'''


def board_poses(cam, w, h, rng, n_target=40, tries=4000):
    """Görüntünün her yerini kaplayan, farklı uzaklık ve eğimlerde board pozları (base_footprint'te).
    Gerçek kalibrasyondaki kural: merkez noktası (cx, cy) ve bozulma ancak tahta görüntünün köşelerine
    ve kenarlarına, eğik açılarla da konursa iyi belirlenir; az görüşte kamera pozuyla karışır."""
    hfov = 2 * math.atan(cam.width / 2 / cam.fx)
    z0 = w / (0.45 * 2 * math.tan(hfov / 2))          # tahta görüntü genişliğinin ~%45'i
    poses = []
    for _ in range(tries):
        if len(poses) >= n_target:
            break
        fu, fv = rng.uniform(0.15, 0.85, 2)
        d = cam.pixel_rays([[fu * cam.width, fv * cam.height]])[0]
        c = cam.t + rng.uniform(0.75, 1.3) * z0 * d
        x = d
        y = np.cross(np.array([0.0, 0.0, 1.0]), x)
        y /= np.linalg.norm(y)
        z = np.cross(x, y)
        R = np.column_stack([x, y, z])
        a, b, r = np.radians(rng.uniform(-40, 40, 3))
        Ry = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])
        Rz = np.array([[math.cos(b), -math.sin(b), 0], [math.sin(b), math.cos(b), 0], [0, 0, 1]])
        Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
        R = R @ Ry @ Rz @ Rx
        corners = np.array([c + R @ [0, sy * w / 2, sz * h / 2] for sy in (-1, 1) for sz in (-1, 1)])
        uv, _, inside = cam.project(corners)
        if not inside.all() or corners[:, 2].min() < 0.02:
            continue
        m = 8                                                # köşeler kenardan en az 8 px içeride
        if (uv[:, 0] < m).any() or (uv[:, 0] > cam.width - m).any() or (uv[:, 1] < m).any() \
                or (uv[:, 1] > cam.height - m).any():
            continue
        poses.append(Pose(c, R))
    return poses


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default=os.path.expanduser('~/cv_forklift_ws/docs/logs/cv/f2/calibration'))
    ap.add_argument('--install-to', default='', help='YAML dosyalarının ayrıca kopyalanacağı dizin')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    es = EvalSetup('station_test', a.out)
    png = os.path.join(a.out, 'board.png')
    bw, bh = board_png(png)
    rng = np.random.default_rng(11)
    result = {'board': {'inner_corners': [COLS, ROWS], 'square_m': SQUARE, 'size_m': [bw, bh]}, 'cameras': {}}
    with simt.SimSession(es, 'calibration', control=False) as sess:
        gz = ros = None
        try:
            gz = simt.Gz(es)
            simt.wait_world_ready(es, gz, sess, [])
            gz.record(False)
            ros = RosCams({c: (es.robot['cameras'][c]['topic'], es.robot['cameras'][c]['camera_info_topic'])
                           for c in CAMS})
            simt.place_robot(gz, (0.0, 0.0), 0.0, settle=0.5)
            bf = simt.robot_bf(gz)
            spawned = False
            for c in CAMS:
                cam = ros.camera(c)
                poses = board_poses(cam, bw, bh, rng)
                pts, used = [], 0
                for k, pb in enumerate(poses):
                    pw = bf * pb
                    if not spawned:
                        gz.spawn(BOARD, board_sdf(png, bw, bh), pw.p, pw.R)
                        spawned = True
                    else:
                        gz.set_pose(BOARD, pw.p, pw.R)
                    gz.wait_sim(0.4)
                    t = gz.now()
                    r = ros.image(c, newer_than=t)
                    if r is None:
                        continue
                    gray = cv2.cvtColor(r[1], cv2.COLOR_RGB2GRAY)
                    corners = cal.find_corners(gray, COLS, ROWS)
                    if corners is None:
                        continue
                    pts.append(corners)
                    used += 1
                    if used <= 3:
                        img = r[1].copy()
                        cv2.drawChessboardCorners(img, (COLS, ROWS), corners.reshape(-1, 1, 2).astype(np.float32), True)
                        ov.save_rgb(img, os.path.join(a.out, f'{c}_view{used}.jpg'))
                obj = cal.board_object_points(COLS, ROWS, SQUARE)
                size = (cam.width, cam.height)
                rms, K, D, per_view = cal.calibrate(pts, obj, size)
                cr = {'views_tried': len(poses), 'views_used': used, 'rms_px': rms, 'per_view_max_px': max(per_view),
                      'K': K.tolist(), 'D': D.tolist(),
                      'camera_info_K': cam.K.tolist(),
                      'delta_cx_px': float(K[0, 2] - cam.cx), 'delta_cy_px': float(K[1, 2] - cam.cy),
                      'delta_fx_rel': float(K[0, 0] / cam.fx - 1), 'delta_fy_rel': float(K[1, 1] / cam.fy - 1)}
                result['cameras'][c] = cr
                comment = (f'Sim kamerası dama tahtasıyla kalibre edildi (DECISIONS D-59), '
                           f'{time.strftime("%Y-%m-%d %H:%M")}\n'
                           f'{used} görüş, {COLS}x{ROWS} iç köşe, kare {SQUARE} m; RMS {rms:.3f} px\n'
                           f'Üreten: ros2 run forklift_evaluation calibrate_cameras')
                for d in [a.out] + ([a.install_to] if a.install_to else []):
                    os.makedirs(d, exist_ok=True)
                    cal.save_yaml(os.path.join(d, f'sim_{c}.yaml'), c, size, K, D, comment)
                print(f'[kalibrasyon] {c}: {used}/{len(poses)} görüş, RMS {rms:.3f} px, '
                      f'cx {K[0, 2]:.2f} cy {K[1, 2]:.2f} fx {K[0, 0]:.2f} fy {K[1, 1]:.2f} D {np.round(D, 4)}',
                      flush=True)
        finally:
            if ros is not None:
                ros.close()
            if gz is not None:
                gz.close()
    with open(os.path.join(a.out, 'calibration.json'), 'w') as f:
        json.dump(result, f, indent=1)


if __name__ == '__main__':
    main()
