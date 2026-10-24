"""
mission_acceptance.py — otonomi kabul testleri: A1 (yanaşma + alma), A2 (yüklü yanaşma + bırakma),
A3 (transit), A4 (tam görev); A5 F5'te

Test operatör rolündedir: /e_stop'u 10 Hz "bırakılmış" yayınlar, /autonomy/set_mode ile Otonom
moda geçer, görev türünü mission_manager'ın 'task' / 'target' parametreleriyle seçer ve
/autonomy/start ile başlatır. Robot yalnız kendi algısı ve odometrisiyle sürer; GT (gz pozları,
stations.yaml, factory_lanes.yaml) yalnız sonucu ölçmek içindir.

  ros2 run forklift_evaluation mission_acceptance --test a1|a2|a3 --world station_test [--trials quick|full]
Komut zinciri açık (control:=true), algılama ve görev yöneticisi ayrı süreçler.

Ölçütler (plan §8):
  A1  girişte yanal ≤ 1 cm ve yaw ≤ 1° (GT), alma başarılı (koli çatalda), komşu koliler < 1 mm
  A2  bırakılan kolinin bölme nominaline ofseti ≤ 1.5 cm, eğim < 2°, taban z 0.05 ± 2 mm; doğru bölme
      (K10: önce orta, sonra sol, sonra sağ), komşular < 1 mm
  A3  ön-yanaşma (istasyon ekseninde predock_distance) ±10 cm / ±5°; şerit sapması RMS ≤ 2 cm
      (factory); kolinin çatalda kayması < 1 cm
  A4  tam görev (task full, merkezden): 3/3 koli mühür rengindeki istasyonun bir bölmesinde, eğim < 2°,
      FAULT ve müdahale yok, ≤ 8 dk
"""

import argparse
import json
import math
import os
import signal
import struct
import subprocess
import threading
import time

import cv2

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from forklift_evaluation.world_gt import EvalSetup, pose_in_front, simt, sl
from forklift_interfaces.msg import AutonomyState
from forklift_interfaces.srv import SetMode

PASS, FAIL, NOT_RUN = 'GEÇTİ', 'KALDI', 'ÇALIŞTIRILAMADI'
# kullanılmayan koliler: istasyonların arasındaki köşeler (ön-yanaşma ve transit yollarının dışında).
# Önceki (0.6, -1.6) yeşil istasyonun sol bölmesinin ön-giriş noktasına düşüyordu: A2'de taşınan koli
# ona sürtünüp çatalda 8 mm kayıyordu.
PARK = [(1.6, -1.6), (1.6, 1.6), (-1.6, 1.6)]
DEST = ['dest_red', 'dest_green', 'dest_blue']
SHOTS = os.path.expanduser('~/cv_forklift_ws/docs/images/cv/f5')
A4_LIMIT_S = 480.0                    # plan §8: tam görev ≤ 8 dk
COLOR = {'dest_red': 'red', 'dest_green': 'green', 'dest_blue': 'blue', 'source': 'neutral'}


class Operator(Node):
    """E-stop kalp atışı, mod / görev servisleri, görev parametreleri, durum aboneliği."""

    def __init__(self):
        super().__init__('a_test_operator', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.estop_pub = self.create_publisher(Bool, '/e_stop', 10)
        self.estop = False                          # None: test yayınlamaz (GUI yayınlıyor)
        self.create_timer(0.1, lambda: self.estop is not None and self.estop_pub.publish(Bool(data=self.estop)))
        self.estop_seen = None                      # /e_stop'ta son görülen (GUI'nin kalp atışı)
        self.create_subscription(Bool, '/e_stop', lambda m: setattr(self, 'estop_seen', bool(m.data)), 10)
        self.state = None
        self.create_subscription(AutonomyState, '/autonomy/state', self._on_state, 10)
        self.mode_cli = self.create_client(SetMode, '/autonomy/set_mode')
        self.param_cli = self.create_client(SetParameters, '/mission_manager/set_parameters')
        self.cli = {n: self.create_client(Trigger, f'/autonomy/{n}') for n in ('start', 'pause', 'resume', 'cancel')}

    def _on_state(self, m):
        self.state = m

    def _wait(self, fut, timeout):
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.02)
        return fut.result() if fut.done() else None

    def set_task(self, task, target='', timeout=10.0):
        if not self.param_cli.wait_for_service(timeout_sec=timeout):
            return False
        req = SetParameters.Request()
        for k, v in (('task', task), ('target', target)):
            req.parameters.append(ParamMsg(name=k, value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=v)))
        r = self._wait(self.param_cli.call_async(req), timeout)
        return r is not None and all(x.successful for x in r.results)

    def call(self, name, timeout=10.0):
        cli = self.mode_cli if name in ('auto', 'manual') else self.cli[name]
        if not cli.wait_for_service(timeout_sec=timeout):
            return False, 'servis yok'
        if name in ('auto', 'manual'):
            req = SetMode.Request()
            req.mode = SetMode.Request.MODE_AUTO if name == 'auto' else SetMode.Request.MODE_MANUAL
        else:
            req = Trigger.Request()
        r = self._wait(cli.call_async(req), timeout)
        return (False, 'yanıt yok') if r is None else (r.success, r.message)


class GuiDriver:
    """Operatör GUI'si yalıtılmış Xephyr :99 içinde; fare olayları yalnız o ekrana gider (sahibin masaüstüne
    hiçbir girdi gitmez). Koordinatlar 1280×800 pencere, 96 dpi (Xephyr 75 dpi'de Qt içeriği küçültüyor)."""
    DISPLAY = ':99'
    ESTOP = (1169, 28)                    # sağ üst e-stop düğmesi (GUI entegrasyon testindeki Theme ölçüleri)
    AUTO = (1170, 273)                    # Otonomi paneli: [Otonom]
    START = (922, 317)                    # [Görevi başlat]

    def __init__(self, logs, hold_s):
        self.logs, self.hold_s, self.procs = logs, hold_s, []
        self.env = dict(os.environ, DISPLAY=self.DISPLAY)

    def open(self):
        self.procs.append(subprocess.Popen(['Xephyr', self.DISPLAY, '-screen', '1600x1000', '-dpi', '96', '-nolisten', 'tcp'],
                                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True))
        time.sleep(2.0)
        self.procs.append(subprocess.Popen(['ros2', 'run', 'forklift_gui', 'forklift_gui'], env=self.env,
                                           stdout=open(os.path.join(self.logs, 'forklift_gui_a4.log'), 'w'),
                                           stderr=subprocess.STDOUT, start_new_session=True))

    def xdo(self, *args):
        subprocess.run(['xdotool', *args], env=self.env, timeout=10, check=True)

    def click(self, xy):
        self.xdo('mousemove', str(xy[0]), str(xy[1]))
        time.sleep(0.2)
        self.xdo('click', '1')

    def release_estop(self):
        self.xdo('mousemove', str(self.ESTOP[0]), str(self.ESTOP[1]))
        time.sleep(0.2)
        self.xdo('mousedown', '1')
        time.sleep(self.hold_s + 0.4)
        self.xdo('mouseup', '1')

    def shot(self, path):
        d = subprocess.run(['xwd', '-root', '-silent', '-display', self.DISPLAY], capture_output=True, timeout=20).stdout
        hs, w, h = (struct.unpack('>I', d[i:i + 4])[0] for i in (0, 16, 20))
        bpl, bpp, ncol = struct.unpack('>I', d[48:52])[0], struct.unpack('>I', d[44:48])[0] // 8, struct.unpack('>I', d[76:80])[0]
        a = np.frombuffer(d[hs + ncol * 12:hs + ncol * 12 + bpl * h], np.uint8).reshape(h, bpl)
        cv2.imwrite(path, a[:, :w * bpp].reshape(h, w, bpp)[:800, :1280, :3])      # xwd BGR(X) → OpenCV BGR

    def close(self):
        for p in reversed(self.procs):
            for sig in (signal.SIGINT, signal.SIGKILL):
                try:
                    os.killpg(os.getpgid(p.pid), sig)
                    p.wait(timeout=10)
                    break
                except Exception:
                    pass
        self.procs = []


class Rig:
    """Bir dünyadaki ortak test düzeneği: gz, operatör, robot ölçüleri, koliler."""

    def __init__(self, es, gz, op):
        self.es, self.gz, self.op = es, gz, op
        self.rg = simt.RobotGeom(es)
        self.parcels = sl.parcels(es.params, es.layout)
        self.P = es.params['parcel']['size']
        self.cx, _, self.cz = sl.parcel_center_in_bay(es.params)
        with open(os.path.join(get_package_share_directory('forklift_autonomy'), 'config', 'autonomy.yaml')) as f:
            self.acfg = yaml.safe_load(f)
        m = self.acfg['mission']
        self.stop_distance = self.rg.carriage_front_x + m['entry_gap']
        self.carry_q = m['carry_q']
        self.predock = m['transit']['predock_distance']
        with open(os.path.join(get_package_share_directory('forklift_perception'), 'config', 'detectors.yaml')) as f:
            # ışınlamadan sonra IMU eğim süzgeci yeni duruşa otursun: jiroskop ışınlamayı görmez (F3 günlüğü);
            # 1 s beklemede koşunun ilk denemesinde bölme yaw'ı ~1° hatalıydı (A1 1.11°)
            self.att_wait = 5.0 * yaml.safe_load(f)['attitude']['tau']

    def restore_parcels(self, skip=()):
        for name, _, (x, y, z, yw) in self.parcels:
            if name not in skip:
                self.gz.set_pose(name, [x, y, z], simt.Rz(yw))

    def park(self, name, k=0):
        px, py = PARK[k % len(PARK)]
        self.gz.set_pose(name, [px, py, 0.105], np.eye(3))

    def bay_parcel_pose(self, stn, i):
        """Bölmedeki nominal koli merkezi (dünya)."""
        B = simt.bay_pose(self.es, stn, i)
        return B * simt.Pose([self.cx, 0.0, self.cz], np.eye(3))

    def load_fork(self, name):
        """Koli çatalda, alma sonrası konumunda: taban + tam giriş mesafesi + bölmedeki nominal x
        (A1'de alma sonrası ölçülen 0.2525 m ile aynı; ışınlama yapay ofset getirmesin)."""
        bf = simt.robot_bf(self.gz)
        local = [self.stop_distance + self.cx, 0.0, self.rg.fork_top0 + self.gz.fork() + self.P['z'] / 2.0 + 0.002]
        pose = bf * simt.Pose(local, np.eye(3))
        self.gz.set_pose(name, pose.p, pose.R)
        self.gz.wait_sim(0.8)

    def run(self, task, target='', limit=150.0, sample=None):
        """Görevi başlat, DONE/FAULT'a kadar izle. sample(state) her 50 ms'de çağrılır."""
        op = self.op
        op.call('manual')
        op.call('cancel')
        if not op.set_task(task, target):
            return {'start_ok': False, 'start_msg': 'parametre ayarlanamadı'}
        op.call('auto')
        ok, msg = op.call('start')
        res = {'start_ok': ok, 'start_msg': msg}
        if not ok:
            return res
        t0, states = time.time(), []
        while time.time() - t0 < limit:
            s = op.state
            if s is not None and (not states or states[-1] != s.state):
                states.append(s.state)
            if sample is not None:
                sample(s.state if s is not None else None)
            if s is not None and s.state in ('DONE', 'FAULT') and len(states) > 1:
                break
            time.sleep(0.05)
        end = op.state
        res.update({'states': states, 'final': end.state if end else None, 'detail': end.detail if end else '',
                    'fault': end.fault if end else '', 'seconds': round(time.time() - t0, 1)})
        op.call('manual')
        op.call('cancel')
        return res

    def deepest_tracker(self, B):
        """Bölme çerçevesinde taşıyıcı önünün en derin konumu (girişte yanal / yaw / eğim)."""
        box = {'d': None}

        def f(_state):
            bf = simt.robot_bf(self.gz)
            if bf is None:
                return
            rel = B.inv() * bf
            yaw_rel = math.atan2(rel.R[1, 0], rel.R[0, 0])
            front = rel.p[0] + self.rg.carriage_front_x * math.cos(yaw_rel)
            c_lat = rel.p[1] + self.rg.carriage_front_x * math.sin(yaw_rel)
            if box['d'] is None or front > box['d']['carriage_front_x']:
                box['d'] = {'carriage_front_x': float(front), 'carriage_lateral': float(c_lat),
                            'yaw_deg': math.degrees(yaw_rel),
                            'pitch_deg': math.degrees(math.atan2(-rel.R[2, 0], rel.R[0, 0]))}
        return box, f


# =============================================================================
#  A1: yanaşma + alma
# =============================================================================
def trial_a1(rig, stn, bay, start, log):
    """Hedef bölme soldan ilk dolu olacak şekilde yerleşim, robot başlangıç pozunda."""
    gz, rg, es = rig.gz, rig.rg, rig.es
    d, lat, yaw_deg = start
    names = {}
    for name, color, (x, y, z, yw) in rig.parcels:
        i = int(name.rsplit('_', 1)[1])
        names[name] = i
        if i > bay:
            rig.park(name, i)
        else:
            gz.set_pose(name, [x, y, z], simt.Rz(yw))
    simt.fork_to(gz, rg, 0.0)
    p_xy, yaw = pose_in_front(stn, d, sl.bay_y(es.params, stn['bays'], bay) + lat, math.radians(yaw_deg))
    simt.place_robot(gz, p_xy, yaw, settle=rig.att_wait)
    target = f'parcel_{stn["name"]}_{bay}'
    neigh = {n: gz.model(n).p.copy() for n, i in names.items() if i < bay and gz.model(n) is not None}
    box, f = rig.deepest_tracker(simt.bay_pose(es, stn, bay))
    res = {'bay': bay, 'start': {'d': d, 'lateral': lat, 'yaw_deg': yaw_deg}}
    res.update(rig.run('pick', sample=f))
    res['deepest'] = box['d']
    bf, pp = simt.robot_bf(gz), gz.model(target)
    rel = bf.inv() * pp
    on_fork = bool(rg.carriage_front_x < rel.p[0] < rg.carriage_front_x + rg.fork_len
                   and abs(rel.p[1]) < 0.05 and rel.p[2] > rg.fork_top0 + 0.03)
    moved = {n: float(np.linalg.norm(gz.model(n).p - p0) * 1000) for n, p0 in neigh.items()}
    res.update({'parcel_in_robot': rel.p.tolist(), 'picked': on_fork, 'neighbours_moved_mm': moved})
    simt.fork_to(gz, rg, 0.0)
    log(f'[A1] bölme {bay} başlangıç {start}: {res.get("final")} ({res.get("detail")}) {res.get("seconds")} s, '
        f'çatalda={on_fork}, giriş {res["deepest"]}')
    return res


def evaluate_a1(trials):
    rows = [t for t in trials if t.get('deepest')]
    ok = [t for t in rows if t.get('final') == 'DONE' and t['picked']]
    done = [t for t in rows if t.get('final') == 'DONE']
    lat = [abs(t['deepest']['carriage_lateral']) for t in done]
    yaw = [abs(t['deepest']['yaw_deg']) for t in done]
    gap = [t['deepest']['carriage_front_x'] for t in done]
    neigh = max((v for t in trials for v in t.get('neighbours_moved_mm', {}).values()), default=0.0)
    s = {'trials': len(trials), 'done_and_picked': len(ok),
         'lateral_mm_max': 1000 * max(lat) if lat else None, 'yaw_deg_max': max(yaw) if yaw else None,
         'carriage_front_x_mm': [round(1000 * min(gap), 1), round(1000 * max(gap), 1)] if gap else None,
         'neighbours_moved_mm_max': neigh,
         'faults': [(t['bay'], t['start'], t.get('detail')) for t in trials if not (t.get('final') == 'DONE' and t.get('picked'))]}
    s['status'] = PASS if (len(ok) == len(trials) and lat and max(lat) <= 0.01 and max(yaw) <= 1.0
                           and neigh < 1.0) else FAIL
    return s


# =============================================================================
#  A2: yüklü yanaşma + bırakma
# =============================================================================
# yerleşim durumu → (önceden dolu bölmeler, beklenen bölme). Bölme 0 istasyonun −y tarafı = robotun sağı.
A2_CASES = {'mid': ((), 1), 'left': ((1,), 2), 'right': ((1, 2), 0)}


def trial_a2(rig, stn, case, start, log):
    gz, rg, es = rig.gz, rig.rg, rig.es
    d, lat, yaw_deg = start
    pre, expect = A2_CASES[case]
    carried, fillers = 'parcel_source_0', ['parcel_source_1', 'parcel_source_2']
    rig.restore_parcels(skip=(carried, *fillers))
    for k, name in enumerate(fillers):
        if k < len(pre):
            P = rig.bay_parcel_pose(stn, pre[k])
            gz.set_pose(name, P.p, P.R)
        else:
            rig.park(name, k + 1)
    rig.park(carried, 0)
    simt.fork_to(gz, rg, rig.carry_q)
    p_xy, yaw = pose_in_front(stn, d, lat, math.radians(yaw_deg))
    simt.place_robot(gz, p_xy, yaw, settle=rig.att_wait)
    rig.load_fork(carried)
    gz.wait_sim(1.0)
    neigh = {n: gz.model(n).p.copy() for n in fillers[:len(pre)]}
    box, f = rig.deepest_tracker(simt.bay_pose(es, stn, expect))
    on_fork_at = {}

    def f2(state):
        f(state)
        # kolinin çatala göre konumu: başta ve indirme başlarken (kayma nerede oluyor)
        if state is not None and state not in on_fork_at:
            bf = simt.robot_bf(gz)
            if bf is not None:
                on_fork_at[state] = [round(float(v) * 1000, 1) for v in (bf.inv() * gz.model(carried)).p[:2]]
    res = {'station': stn['name'], 'case': case, 'expected_bay': expect,
           'start': {'d': d, 'lateral': lat, 'yaw_deg': yaw_deg}}
    res.update(rig.run('drop', sample=f2))
    res['parcel_on_fork_mm'] = on_fork_at
    res['deepest'] = box['d']
    pp = gz.model(carried)
    # kolinin hangi bölmede olduğu ve nominale göre ofseti
    best = None
    for i in range(stn['bays']):
        rel = rig.bay_parcel_pose(stn, i).inv() * pp
        if best is None or np.linalg.norm(rel.p[:2]) < np.linalg.norm(best[1].p[:2]):
            best = (i, rel)
    i, rel = best
    bf = simt.robot_bf(gz)
    on_fork = bool(abs((bf.inv() * pp).p[1]) < 0.05 and 0.1 < (bf.inv() * pp).p[0] < 0.45)
    res.update({'bay': i, 'offset_mm': (rel.p[:2] * 1000).tolist(), 'offset_norm_mm': float(np.linalg.norm(rel.p[:2]) * 1000),
                'tilt_deg': float(simt.tilt_deg(pp.R)), 'base_z_mm': float((pp.p[2] - rig.P['z'] / 2.0) * 1000),
                'parcel_on_fork_after': on_fork,
                'neighbours_moved_mm': {n: float(np.linalg.norm(gz.model(n).p - p0) * 1000) for n, p0 in neigh.items()}})
    simt.fork_to(gz, rg, 0.0)
    log(f'[A2] {stn["name"]} {case} başlangıç {start}: {res.get("final")} ({res.get("detail")}) {res.get("seconds")} s, '
        f'bölme {i} (beklenen {expect}), ofset {res["offset_norm_mm"]:.1f} mm, eğim {res["tilt_deg"]:.2f}°, '
        f'taban {res["base_z_mm"]:.1f} mm')
    return res


def evaluate_a2(trials):
    rows = [t for t in trials if 'offset_norm_mm' in t]
    ok = [t for t in rows if t.get('final') == 'DONE' and t['bay'] == t['expected_bay'] and not t['parcel_on_fork_after']]
    neigh = max((v for t in trials for v in t.get('neighbours_moved_mm', {}).values()), default=0.0)
    off = [t['offset_norm_mm'] for t in ok]
    tilt = [t['tilt_deg'] for t in ok]
    bz = [abs(t['base_z_mm'] - 50.0) for t in ok]
    s = {'trials': len(trials), 'done_correct_bay': len(ok),
         'offset_mm_max': max(off) if off else None, 'tilt_deg_max': max(tilt) if tilt else None,
         'base_z_err_mm_max': max(bz) if bz else None, 'neighbours_moved_mm_max': neigh,
         'entry_lateral_mm_max': max((abs(t['deepest']['carriage_lateral']) * 1000 for t in ok if t.get('deepest')), default=None),
         'entry_yaw_deg_max': max((abs(t['deepest']['yaw_deg']) for t in ok if t.get('deepest')), default=None),
         'faults': [(t.get('station'), t.get('case'), t.get('start'), t.get('detail')) for t in trials if t not in ok]}
    s['status'] = PASS if (len(ok) == len(trials) and off and max(off) <= 15.0 and max(tilt) < 2.0
                           and max(bz) <= 2.0 and neigh < 1.0) else FAIL
    return s


# =============================================================================
#  A3: transit (kaynaktan her hedefe yüklü; bilgi: hedeften kaynağa yüksüz)
# =============================================================================
def trial_a3(rig, src, dst, start, loaded, log, lane_gt):
    gz, rg = rig.gz, rig.rg
    d, lat, yaw_deg = start
    carried = 'parcel_source_0'
    rig.restore_parcels(skip=(carried,) if loaded else ())
    if loaded:
        rig.park(carried, 0)
    simt.fork_to(gz, rg, rig.carry_q if loaded else 0.0)
    p_xy, yaw = pose_in_front(src, d, lat, math.radians(yaw_deg))
    simt.place_robot(gz, p_xy, yaw, settle=rig.att_wait)
    if loaded:
        rig.load_fork(carried)
    gz.wait_sim(1.0)
    bf0 = simt.robot_bf(gz)
    p_rel0 = (bf0.inv() * gz.model(carried)).p.copy() if loaded else None
    S = simt.station_pose(dst)
    lane_dev = []

    def f(state):
        if state == 'LANE_FOLLOW' and lane_gt is not None:
            bf = simt.robot_bf(gz)
            if bf is not None:
                lane_dev.append(float((S.inv() * bf).p[1]))      # şerit istasyon ekseninde (factory_lanes.yaml)
    res = {'from': src['name'], 'to': dst['name'], 'loaded': loaded, 'start': {'d': d, 'lateral': lat, 'yaw_deg': yaw_deg}}
    res.update(rig.run('transit', COLOR[dst['name']], limit=200.0, sample=f))
    bf = simt.robot_bf(gz)
    rel = S.inv() * bf
    res['predock'] = {'distance_err_mm': float((-rel.p[0] - rig.predock) * 1000), 'lateral_mm': float(rel.p[1] * 1000),
                      'yaw_deg': math.degrees(math.atan2(rel.R[1, 0], rel.R[0, 0]))}
    res['lane_rms_mm'] = float(np.sqrt(np.mean(np.square(lane_dev))) * 1000) if lane_dev else None
    res['lane_max_mm'] = float(np.max(np.abs(lane_dev)) * 1000) if lane_dev else None
    res['lane_trace_mm'] = [round(x * 1000, 1) for x in lane_dev[::5]]          # 4 Hz iz (inceleme için)
    if loaded:
        p_rel1 = (bf.inv() * gz.model(carried)).p
        res['slip_mm'] = float(np.linalg.norm((p_rel1 - p_rel0)[:2]) * 1000)
    simt.fork_to(gz, rg, 0.0)
    log(f'[A3] {src["name"]} → {dst["name"]} ({"yüklü" if loaded else "yüksüz"}): {res.get("final")} ({res.get("detail")}) '
        f'{res.get("seconds")} s, ön-yanaşma {res["predock"]}, şerit RMS {res["lane_rms_mm"]}, kayma {res.get("slip_mm")}')
    return res


def evaluate_a3(trials, lane_world):
    main = [t for t in trials if t['loaded']]
    ok = [t for t in main if t.get('final') == 'DONE' and abs(t['predock']['distance_err_mm']) <= 100
          and abs(t['predock']['lateral_mm']) <= 100 and abs(t['predock']['yaw_deg']) <= 5.0
          and (t.get('slip_mm') or 0) < 10.0 and (not lane_world or (t['lane_rms_mm'] is not None and t['lane_rms_mm'] <= 20.0))]
    pe = [t['predock'] for t in main if t.get('final') == 'DONE']
    s = {'trials': len(main), 'ok': len(ok),
         'predock_distance_err_mm_max': max((abs(p['distance_err_mm']) for p in pe), default=None),
         'predock_lateral_mm_max': max((abs(p['lateral_mm']) for p in pe), default=None),
         'predock_yaw_deg_max': max((abs(p['yaw_deg']) for p in pe), default=None),
         'lane_rms_mm_max': max((t['lane_rms_mm'] for t in main if t.get('lane_rms_mm') is not None), default=None),
         'slip_mm_max': max((t.get('slip_mm') or 0 for t in main), default=None),
         'seconds_max': max((t.get('seconds') or 0 for t in main), default=None),
         'faults': [(t['to'], t['start'], t.get('detail')) for t in main if t not in ok],
         'return_info': [(t['from'], t.get('final'), t['predock'], t.get('lane_rms_mm'), t.get('seconds'))
                         for t in trials if not t['loaded']]}
    s['status'] = PASS if len(ok) == len(main) and main else FAIL
    return s


def _find_key(d, key):
    """İç içe sözlükte ilk key değeri (gui_params.yaml'daki release_hold)."""
    if isinstance(d, dict):
        for k, v in d.items():
            if k == key:
                return v
            r = _find_key(v, key)
            if r is not None:
                return r
    return None


# =============================================================================
#  A4: tam görev
# =============================================================================
def trial_a4(rig, start, log, limit, gui=None, shots=None):
    """gui: GuiDriver ise görev operatör GUI'sinden başlatılır (e-stop bırakma, [Otonom], [Görevi başlat])."""
    gz, rg = rig.gz, rig.rg
    rig.restore_parcels()
    simt.fork_to(gz, rg, 0.0)
    x, y, yaw_deg = start
    simt.place_robot(gz, (x, y), math.radians(yaw_deg), settle=rig.att_wait)
    rig.op.call('manual')
    res = {'start': {'x': x, 'y': y, 'yaw_deg': yaw_deg}, 'started_from': 'gui' if gui else 'servis'}
    if gui is not None:
        return _a4_gui(rig, gui, res, log, limit, shots)
    trace = []

    def f(state):
        st = rig.op.state
        if st is not None and (not trace or trace[-1][1] != st.state):
            trace.append((round(time.time() - t0, 1), st.state, st.detail, int(st.parcels_done)))
    t0 = time.time()
    res.update(rig.run('full', limit=limit, sample=f))
    res['trace'] = trace
    res['parcels_done_reported'] = int(rig.op.state.parcels_done) if rig.op.state else None
    out = _a4_parcels(rig)
    res['parcels'] = out
    simt.fork_to(gz, rg, 0.0)
    log(f'[A4] başlangıç {start}: {res.get("final")} ({res.get("detail")}) {res.get("seconds")} s; '
        + '; '.join(f'{o["seal"]} → {o["station"]}/{o["bay"]} ({o["offset_mm"]:.1f} mm, {o["tilt_deg"]:.2f}°)' for o in out))
    return res


def _a4_parcels(rig):
    out = []
    for name, color, _ in rig.parcels:
        pp = rig.gz.model(name)
        best = None
        for st in rig.es.layout['stations']:
            for i in range(st['bays']):
                rel = rig.bay_parcel_pose(st, i).inv() * pp
                if best is None or np.linalg.norm(rel.p[:2]) < np.linalg.norm(best[2].p[:2]):
                    best = (st['name'], i, rel)
        stn, i, rel = best
        out.append({'parcel': name, 'seal': color, 'station': stn, 'bay': i,
                    'correct': bool(stn == f'dest_{color}' and np.linalg.norm(rel.p[:2]) < 0.03),
                    'offset_mm': float(np.linalg.norm(rel.p[:2]) * 1000), 'tilt_deg': float(simt.tilt_deg(pp.R)),
                    'base_z_mm': float((pp.p[2] - rig.P['z'] / 2.0) * 1000)})
    return out


def _a4_gui(rig, gui, res, log, limit, shots):
    """Görevi GUI'den başlat ve bitene kadar izle; adımlar ve ekran görüntüleri sonuca yazılır."""
    op = rig.op
    op.call('cancel')
    if not op.set_task('full'):
        res.update({'final': NOT_RUN, 'detail': 'task parametresi ayarlanamadı'})
        return res
    op.estop = None                                  # e-stop'u artık GUI yayınlıyor (açılışta basılı)
    steps = []
    gui.open()
    try:
        t0 = time.time()
        while time.time() - t0 < 60 and op.estop_seen is not True:
            time.sleep(0.2)
        steps.append(('GUI açıldı, e-stop basılı', op.estop_seen is True))
        time.sleep(3.0)                              # QML yüklensin, otonomi çevrimiçi görünsün
        gui.shot(os.path.join(shots, f'a4_gui_{rig.es.world}_0_open.png'))
        gui.release_estop()
        time.sleep(0.5)
        steps.append(('e-stop bırakıldı (1.9 s basılı tutma)', op.estop_seen is False))
        gui.click(gui.AUTO)
        t1 = time.time()
        while time.time() - t1 < 5 and not (op.state and op.state.mode == AutonomyState.MODE_AUTO):
            time.sleep(0.1)
        steps.append(('[Otonom] tıklandı → mod Otonom', bool(op.state and op.state.mode == AutonomyState.MODE_AUTO)))
        gui.click(gui.START)
        t1 = time.time()
        while time.time() - t1 < 5 and not (op.state and op.state.state not in ('IDLE', 'DONE', 'FAULT')):
            time.sleep(0.1)
        steps.append(('[Görevi başlat] tıklandı → görev başladı', bool(op.state and op.state.state not in ('IDLE', 'DONE', 'FAULT'))))
        res['gui_steps'] = steps
        log(f'[A4] GUI adımları: {steps}')
        if not all(ok for _, ok in steps):
            res.update({'final': FAIL, 'detail': 'GUI adımı başarısız'})
            return res
        t0, states, shot_at = time.time(), [], {60: f'a4_gui_{rig.es.world}_1_running.png'}
        while time.time() - t0 < limit:
            s = op.state
            if s is not None and (not states or states[-1] != s.state):
                states.append(s.state)
            for k, name in list(shot_at.items()):
                if time.time() - t0 > k:
                    gui.shot(os.path.join(shots, name))
                    del shot_at[k]
            if s is not None and s.state in ('DONE', 'FAULT'):
                break
            time.sleep(0.2)
        time.sleep(1.0)
        gui.shot(os.path.join(shots, f'a4_gui_{rig.es.world}_2_done.png'))
        end = op.state
        res.update({'states': states, 'final': end.state if end else None, 'detail': end.detail if end else '',
                    'fault': end.fault if end else '', 'seconds': round(time.time() - t0, 1),
                    'parcels_done_reported': int(end.parcels_done) if end else None})
    finally:
        gui.close()
        op.estop = False
        op.call('manual')
        op.call('cancel')
    res['parcels'] = _a4_parcels(rig)
    log(f'[A4] GUI başlatma {res["start"]}: {res.get("final")} ({res.get("detail")}) {res.get("seconds")} s; '
        + '; '.join(f'{o["seal"]} → {o["station"]}/{o["bay"]} ({o["offset_mm"]:.1f} mm, {o["tilt_deg"]:.2f}°)' for o in res['parcels']))
    return res


def evaluate_a4(trials, limit):
    ok = [t for t in trials if t.get('final') == 'DONE' and all(o['correct'] and o['tilt_deg'] < 2.0 for o in t.get('parcels', []))
          and (t.get('seconds') or 1e9) <= limit and 'PAUSED' not in t.get('states', [])]
    allp = [o for t in trials for o in t.get('parcels', [])]
    s = {'trials': len(trials), 'ok': len(ok),
         'parcels_correct': sum(1 for o in allp if o['correct']), 'parcels': len(allp),
         'offset_mm_max': max((o['offset_mm'] for o in allp if o['correct']), default=None),
         'tilt_deg_max': max((o['tilt_deg'] for o in allp), default=None),
         'seconds': [t.get('seconds') for t in trials],
         'faults': [(t['start'], t.get('final'), t.get('detail')) for t in trials if t not in ok]}
    s['status'] = PASS if len(ok) == len(trials) and trials else FAIL
    return s


# =============================================================================
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--test', default='a1', choices=['a1', 'a2', 'a3', 'a4'])
    ap.add_argument('--world', default='station_test')
    ap.add_argument('--trials', default='quick', help='quick | full | smoke')
    ap.add_argument('--out', default=None, help='varsayılan: docs/logs/cv/f3 (a1) | f4 (a2, a3)')
    ap.add_argument('--gui', action='store_true')
    ap.add_argument('--only', default='', help='a1: bölme,d,yanal,yaw_deg | a2: istasyon,durum,d,yanal,yaw_deg | a3: hedef,d,yanal,yaw_deg')
    a = ap.parse_args()
    out = a.out or os.path.expanduser(f'~/cv_forklift_ws/docs/logs/cv/{ {"a1": "f3", "a2": "f4", "a3": "f4"}.get(a.test, "f5")}')
    os.makedirs(out, exist_ok=True)
    es = EvalSetup(a.world, out, gui=a.gui)
    method = 'lane' if a.world == 'factory' else 'sign'

    if a.test == 'a1':
        grid = {'smoke': [(1, (1.1, 0.05, 5))],
                'quick': [(1, (1.1, 0.05, 5)), (0, (1.2, -0.1, -8)), (2, (0.9, 0.12, 8))],
                'full': [(b, s) for b in (0, 1, 2) for s in ((0.9, 0.0, 0), (1.3, 0.15, 10), (1.3, -0.15, -10),
                                                               (1.0, 0.1, -5), (1.0, -0.1, 5), (1.5, 0.0, 0))]}[a.trials]
        if a.only:
            b, d, lat, yd = a.only.split(',')
            grid = [(int(b), (float(d), float(lat), float(yd)))]
    elif a.test == 'a2':
        starts = ((0.9, 0.0, 0), (1.3, 0.15, 10), (1.3, -0.15, -10), (1.0, 0.1, -5), (1.5, 0.0, 0), (0.8, -0.1, 5))
        grid = {'smoke': [('dest_red', 'mid', (1.0, 0.05, 5))],
                'quick': [('dest_red', 'mid', (1.0, 0.05, 5)), ('dest_green', 'left', (1.2, -0.1, -8)),
                          ('dest_blue', 'right', (0.9, 0.12, 8))],
                # 3 istasyon × 3 doluluk durumu × 2 başlangıç (başlangıçlar sırayla dönüşür)
                'full': [(st, case, starts[(3 * k + j + 3 * r) % len(starts)]) for r in (0, 1)
                         for k, st in enumerate(DEST) for j, case in enumerate(('mid', 'left', 'right'))]}[a.trials]
        if a.only:
            st, case, d, lat, yd = a.only.split(',')
            grid = [(st, case, (float(d), float(lat), float(yd)))]
    elif a.test == 'a4':
        # dünya pozu (x, y, yaw_deg): merkez (bringup'ın varsayılan doğuş pozu) ve kaynağa bakmayan bir poz
        # 'gui': aynı tam görev, operatör GUI'sinden başlatılır (Xephyr :99)
        grid = {'smoke': [(0.0, 0.0, 0)], 'full': [(0.0, 0.0, 0), (-0.3, 0.4, 135)],
                'gui': [(0.0, 0.0, 0, 'gui')]}[a.trials]
    else:
        # A1 akışının bitişi: tam giriş (taşıyıcı önü + giriş boşluğu) + geri çıkış
        with open(os.path.join(get_package_share_directory('forklift_autonomy'), 'config', 'autonomy.yaml')) as f:
            m = yaml.safe_load(f)['mission']
        fc = es.robot['fork_carriage']
        after_pick = fc['x'] + fc['size']['x'] / 2.0 + m['entry_gap'] + m['back_out']
        grid = {'smoke': [('source', 'dest_red', (after_pick, 0.0, 0), True)],
                'quick': [('source', st, (after_pick, 0.0, 0), True) for st in DEST],
                'full': [('source', st, (after_pick, lat, yd), True) for st in DEST
                         for lat, yd in ((0.0, 0.0), (0.03, 2.0), (-0.03, -2.0))]
                + [(st, 'source', (after_pick, 0.0, 0), False) for st in DEST],
                'return': [(st, 'source', (after_pick, 0.0, 0), False) for st in DEST]}[a.trials]
        if a.only:
            st, d, lat, yd = a.only.split(',')
            grid = [('source', st, (float(d), float(lat), float(yd)), True)]

    lines = []

    def log(t):
        print(t, flush=True)
        lines.append(t)
    tag = f'{a.test}{"gui" if a.trials == "gui" else ""}_{a.world}'
    res = {'world': a.world, 'test': a.test, 'transit_method': method,
           'started': time.strftime('%Y-%m-%d %H:%M:%S'), 'trials': []}
    with simt.SimSession(es, tag, control=True) as sess:
        procs = []
        gz = None
        try:
            for pkg, lf, args in (('forklift_perception', 'perception.launch.py', []),
                                  ('forklift_autonomy', 'autonomy.launch.py', [f'transit_method:={method}'])):
                procs.append(subprocess.Popen(['ros2', 'launch', pkg, lf, *args],
                                              stdout=open(os.path.join(es.logs, f'{pkg}_{tag}.log'), 'w'),
                                              stderr=subprocess.STDOUT, start_new_session=True))
            gz = simt.Gz(es)
            simt.wait_world_ready(es, gz, sess, sl.parcels(es.params, es.layout))
            gz.record(False)
            rclpy.init()
            op = Operator()
            th = threading.Thread(target=rclpy.spin, args=(op,), daemon=True)
            th.start()
            # algılamanın IMU seviye kalibrasyonu için robot başlangıçta düz zeminde (orijinde) bekler
            simt.place_robot(gz, (0.0, 0.0), 0.0, settle=4.0)
            rig = Rig(es, gz, op)
            lane_gt = es.world == 'factory' or None
            for g in grid:
                try:
                    if a.test == 'a1':
                        r = trial_a1(rig, es.station('source'), g[0], g[1], log)
                    elif a.test == 'a2':
                        r = trial_a2(rig, es.station(g[0]), g[1], g[2], log)
                    elif a.test == 'a4':
                        gd = None
                        if len(g) > 3:
                            with open(os.path.join(get_package_share_directory('forklift_gui'), 'config', 'gui_params.yaml')) as f:
                                hold = _find_key(yaml.safe_load(f), 'release_hold')
                            gd = GuiDriver(es.logs, hold)
                            os.makedirs(SHOTS, exist_ok=True)
                        r = trial_a4(rig, g[:3], log, A4_LIMIT_S, gd, SHOTS)
                    else:
                        r = trial_a3(rig, es.station(g[0]), es.station(g[1]), g[2], g[3], log, lane_gt)
                except Exception as e:
                    import traceback
                    traceback.print_exc()
                    r = {'grid': repr(g), 'final': NOT_RUN, 'detail': repr(e), 'loaded': True,
                         'predock': {'distance_err_mm': 1e9, 'lateral_mm': 1e9, 'yaw_deg': 1e9}, 'to': str(g),
                         'start': None, 'bay': None}
                res['trials'].append(r)
                with open(os.path.join(out, f'{tag}.json'), 'w') as f:
                    json.dump(res, f, indent=1, ensure_ascii=False)
            rig.restore_parcels()
            key = a.test.upper()
            res[key] = {'a1': evaluate_a1, 'a2': evaluate_a2, 'a4': lambda t: evaluate_a4(t, A4_LIMIT_S)}.get(
                a.test, lambda t: evaluate_a3(t, es.world == 'factory'))(res['trials'])
            log(f'[{key} {a.world}] {res[key]}')
            op.destroy_node()
            rclpy.shutdown()
        finally:
            for p in procs:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGINT)
                    p.wait(timeout=15)
                except Exception:
                    try:
                        os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                    except Exception:
                        pass
            if gz is not None:
                gz.close()
    res['finished'] = time.strftime('%Y-%m-%d %H:%M:%S')
    with open(os.path.join(out, f'{tag}.json'), 'w') as f:
        json.dump(res, f, indent=1, ensure_ascii=False)


if __name__ == '__main__':
    main()
