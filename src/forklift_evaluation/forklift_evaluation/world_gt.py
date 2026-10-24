"""
world_gt.py — değerlendirme ortamı ve dünyanın ground truth geometrisi

Kaynaklar (yalnız test; robot bunları okumaz, plan R2):
  stations.yaml, markers.yaml (build'de üretilir), factory.yaml + factory_lanes.yaml,
  station_params.yaml, forklift_params.yaml; anlık pozlar gz'den (sim_tools.Gz).
forklift_gazebo'nun ortak modülleri (station_layout, sim_tools) kurulu
lib/forklift_gazebo dizininden içe aktarılır.
"""

import math
import os
import sys

import numpy as np
from ament_index_python.packages import get_package_prefix, get_package_share_directory

sys.path.insert(0, os.path.join(get_package_prefix('forklift_gazebo'), 'lib', 'forklift_gazebo'))
import sim_tools as simt  # noqa: E402
import station_layout as sl  # noqa: E402

WORLDS = ('station_test', 'factory')


class EvalSetup:
    """sim_tools'un beklediği alanlar (world, gui, logs, robot, params) + GT dosyaları."""

    def __init__(self, world, out, gui=False, stations_file=None):
        if world not in WORLDS:
            raise ValueError(f'bilinmeyen dünya: {world}')
        share = get_package_share_directory('forklift_gazebo')
        cfg = os.path.join(share, 'config')
        self.world, self.gui = world, gui
        self.params = sl.load_yaml(os.path.join(cfg, 'station_params.yaml'))
        self.stations_file = stations_file or os.path.join(cfg, 'stations.yaml')    # A5: yerleri değişmiş
        self.layout = sl.load_yaml(self.stations_file)
        self.markers = sl.load_yaml(os.path.join(cfg, 'markers.yaml'))
        self.factory = sl.load_yaml(os.path.join(cfg, 'factory.yaml')) if world == 'factory' else None
        self.lanes = sl.load_yaml(os.path.join(cfg, 'factory_lanes.yaml'))['lanes'] if world == 'factory' else []
        self.robot = sl.load_yaml(os.path.join(get_package_share_directory('forklift_description'),
                                               'config', 'forklift_params.yaml'))
        self.out = out
        self.logs = os.path.join(out, 'logs')
        self.images = os.path.join(out, 'images')
        os.makedirs(self.logs, exist_ok=True)
        os.makedirs(self.images, exist_ok=True)

    def station(self, name):
        return next(s for s in self.layout['stations'] if s['name'] == name)


# =============================================================================
#  Yardımcılar
# =============================================================================
def world_pts(st_entry, local):
    """Station çerçevesindeki noktalar (N×3) → dünya (N×3)."""
    return np.array([sl.compose(st_entry['pose'], *p) for p in local], float)


def rect_on_ground(a, b, width, z):
    """a → b merkez çizgisi, genişlik ve yükseklikle zemindeki dikdörtgen (4 köşe, dünya)."""
    a, b = np.asarray(a[:2], float), np.asarray(b[:2], float)
    d = (b - a) / np.linalg.norm(b - a)
    n = np.array([-d[1], d[0]]) * width / 2.0
    return np.array([[*(a + n), z], [*(b + n), z], [*(b - n), z], [*(a - n), z]])


def pose_in_front(st_entry, d, lateral=0.0, yaw_off=0.0):
    """İstasyona bakan robot: base_footprint station çerçevesinde (−d, lateral); (dünya xy, yaw)."""
    x, y, _ = sl.compose(st_entry['pose'], -d, lateral)
    return (x, y), st_entry['pose'][2] + yaw_off


# =============================================================================
#  GT geometrisi (dünya çerçevesinde)
# =============================================================================
class WorldGT:
    def __init__(self, es):
        self.es = es
        self.P = es.params

    # ----------------------------------------------------- hizalama hedefleri
    def align_targets(self):
        """Her hedef: istasyon, bölme, taraf, merkez (dünya) ve plaka köşeleri (dünya, 4×3)."""
        a = self.P['markers']['align']['size'] / 2.0
        out = []
        for t in self.es.markers['align_targets']:
            stn = self.es.station(t['station'])
            x, y, z = t['point_station']
            plate = world_pts(stn, [(x, y + a, z + a), (x, y - a, z + a), (x, y - a, z - a), (x, y + a, z - a)])
            out.append({'station': t['station'], 'bay': t['bay'], 'side': t['side'],
                        'center': np.array(t['point_world'], float), 'plate': plate})
        return out

    # ----------------------------------------------------- şeritler
    def bay_lines(self):
        out = []
        for b in self.es.markers['bay_lines']:
            out.append({'station': b['station'], 'bay': b['bay'], 'color': b['color'],
                        'poly': rect_on_ground(b['start_world'], b['end_world'], b['width'], b['start_world'][2])})
        return out

    def floor_lanes(self):
        """factory: kılavuz şeritleri ve uçtaki beyaz durma çizgileri (dünya poligonları)."""
        if not self.es.lanes:
            return []
        M = self.es.factory['markings']
        h, sb = M['height'], M['lanes']['stop_bar']
        out = []
        for ln in self.es.lanes:
            yaw = ln['yaw']
            e = np.array(ln['end'], float)
            u = np.array([math.cos(yaw), math.sin(yaw)])
            out.append({'station': ln['station'], 'color': ln['color'],
                        'poly': rect_on_ground(ln['start'], ln['end'], ln['width'], h),
                        'stop_bar': rect_on_ground(e - u * sb['width'], e, sb['length'], h * 1.5)})
        return out

    # ----------------------------------------------------- tabelalar
    def signs(self):
        """Tabela dörtgeni ve renk bloğu (robota bakan yüz; dünya 4×3, sol üst'ten saat yönü)."""
        out = []
        for stn in self.es.layout['stations']:
            x, y0, y1, z0, z1 = sl.sign_rect(self.P, stn['bays'])
            h = z1 - z0
            quad = world_pts(stn, [(x, y1, z1), (x, y0, z1), (x, y0, z0), (x, y1, z0)])
            block = world_pts(stn, [(x, y1, z1), (x, y1 - h, z1), (x, y1 - h, z0), (x, y1, z0)])
            out.append({'station': stn['name'], 'color': stn['color'], 'quad': quad, 'block': block})
        return out

    # ----------------------------------------------------- bölme etiketleri (D-62)
    def bay_labels(self):
        """Arka plakadaki bölme etiketi (robota bakan yüz): istasyon, bölme, merkez ve dörtgen (dünya)."""
        v, s = self.P['appearance'], self.P['station']
        x = s['ledge_depth'] + v['back_plate_recess'] / 2.0 - v['label_thickness'] / 2.0
        a, zc = v['bay_label']['size'] / 2.0, v['bay_label']['center_z']
        out = []
        for stn in self.es.layout['stations']:
            for i in range(stn['bays']):
                y = sl.bay_y(self.P, stn['bays'], i)
                quad = world_pts(stn, [(x, y + a, zc + a), (x, y - a, zc + a), (x, y - a, zc - a), (x, y + a, zc - a)])
                out.append({'station': stn['name'], 'bay': i, 'center': world_pts(stn, [(x, y, zc)])[0], 'quad': quad})
        return out

    # ----------------------------------------------------- koliler (gz pozundan)
    def parcel_boxes(self, gz):
        """Her kolinin 8 köşesi ve −x yüzündeki mühür karesi (dünya)."""
        poses = {}
        for name, _, _ in sl.parcels(self.P, self.es.layout):
            pose = gz.model(name)
            if pose is not None:
                poses[name] = pose
        return self.parcel_boxes_from(poses)

    def parcel_boxes_from(self, poses):
        """poses: {koli adı: Pose} (kayıtlı kümeden ya da gz'den)."""
        S = self.P['parcel']['size']
        seal = self.P['parcel']['seal']
        hx, hy, hz = S['x'] / 2, S['y'] / 2, S['z'] / 2
        corners = np.array([[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        sy, sz = seal['size']['y'] / 2, seal['size']['z'] / 2
        sx = -hx - seal['thickness']
        seal_local = np.array([[sx, sy, sz], [sx, -sy, sz], [sx, -sy, -sz], [sx, sy, -sz]])
        face_local = np.array([[-hx, hy, hz], [-hx, -hy, hz], [-hx, -hy, -hz], [-hx, hy, -hz]])
        colors = {n: c for n, c, _ in sl.parcels(self.P, self.es.layout)}
        out = []
        for name, pose in poses.items():
            out.append({'name': name, 'color': colors.get(name),
                        'center': pose.p.copy(),
                        'corners': np.array([pose.apply(c) for c in corners]),
                        'face': np.array([pose.apply(c) for c in face_local]),
                        'seal': np.array([pose.apply(c) for c in seal_local])})
        return out

    # ----------------------------------------------------- robot geometrisi (base_footprint)
    def fork_boxes(self, q):
        """Çatallar ve taşıyıcı plakası: base_footprint'te kutu köşeleri (her biri 8×3)."""
        r = self.es.robot
        fk, fc = r['fork'], r['fork_carriage']
        boxes = []
        for s in (-1, 1):
            boxes.append(_box([fk['x_min'], fk['x_min'] + fk['size']['x']],
                              [s * fk['y'] - fk['size']['y'] / 2, s * fk['y'] + fk['size']['y'] / 2],
                              [fk['ground_z_bottom'] + q, fk['ground_z_bottom'] + fk['size']['z'] + q]))
        boxes.append(_box([fc['x'] - fc['size']['x'] / 2, fc['x'] + fc['size']['x'] / 2],
                          [-fc['size']['y'] / 2, fc['size']['y'] / 2],
                          [fc['ground_z_bottom'] + q, fc['ground_z_bottom'] + fc['size']['z'] + q]))
        return boxes

    def robot_footprint_ok(self, p_xy, yaw, margin=0.03):
        """Robot (şasi + çatal ucu) bu pozda hiçbir istasyonun gövdesine girmiyor mu (önlük hariç)."""
        r = self.es.robot
        ch = r['chassis']
        x0 = ch['center']['x'] - ch['size']['x'] / 2 - margin
        x1 = r['fork']['x_min'] + r['fork']['size']['x'] + margin
        hw = r['drive_wheel']['separation'] / 2 + r['drive_wheel']['width'] / 2 + margin
        robot = [(x0, -hw), (x1, -hw), (x1, hw), (x0, hw)]
        c, s = math.cos(yaw), math.sin(yaw)
        robot_w = np.array([[p_xy[0] + c * x - s * y, p_xy[1] + s * x + c * y] for x, y in robot])
        S, A = self.P['station'], self.P['appearance']
        depth = S['ledge_depth'] + S['back_plate_thickness'] + A['back_plate_recess'] + A['frame']['profile']
        for stn in self.es.layout['stations']:
            W = stn['bays'] * S['bay_pitch']
            body = world_pts(stn, [(0, -W / 2, 0), (depth, -W / 2, 0), (depth, W / 2, 0), (0, W / 2, 0)])[:, :2]
            if _convex_overlap(robot_w, body):
                return False
        return True


def _box(xr, yr, zr):
    return np.array([[x, y, z] for x in xr for y in yr for z in zr], float)


def _convex_overlap(a, b):
    """İki dışbükey çokgen kesişiyor mu (ayırıcı eksen teoremi)."""
    for poly in (a, b):
        for i in range(len(poly)):
            e = poly[(i + 1) % len(poly)] - poly[i]
            n = np.array([-e[1], e[0]])
            pa, pb = a @ n, b @ n
            if pa.max() < pb.min() or pb.max() < pa.min():
                return False
    return True


Pose = simt.Pose
