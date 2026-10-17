#!/usr/bin/env python3
"""
acceptance_test.py — forklift kabul testleri (spesifikasyon bölüm 5, T1–T14)

Kullanım
  ros2 run forklift_gazebo acceptance_test.py                  # tüm testler
  ros2 run forklift_gazebo acceptance_test.py --tests T1,T3,T7 # seçili testler
Seçenekler
  --docs DIR   çıktı dizini (varsayılan: <ws>/docs)
  --ws DIR     workspace kökü (varsayılan: ~/cv_forklift_ws)
  --gui        simülasyon oturumlarını Gazebo GUI ile aç (varsayılan: ekransız)
  --params F   robot parametre dosyası (varsayılan: forklift_description/config/
               forklift_params.yaml); hem simülasyona hem analize aynı dosya verilir

Oturumlar: betik simülasyonu kendisi başlatır (sim.launch.py) ve testleri iki
temiz oturumda koşar; böylece T3 gerçekten spawn anından ölçülür.
  A: T3, T4, T11, T14, T7, T6, T5   (T6, raylardan uzakta, T5'ten önce)
  B: T12, T13, T8, T9, T10          (birlikte koşar; T9 ve T10 T8'in taşıma
                                     bölümündedir, DECISIONS D-16 / D-19)
T1 ve T2 simülasyon gerektirmez.

Gerçek poz (ground truth): gz-transport /world/<dünya>/dynamic_pose/info
(model dünya pozları + link'lerin modele göre pozları, ~60 Hz; DECISIONS D-13).
Çıktı: her test için ölçülen değer, kriter ve GEÇTİ / KALDI / ÇALIŞTIRILAMADI;
docs/test_results.{json,md}, docs/images/*.png, docs/frames_sim.pdf, docs/logs/*.
"""

import argparse
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
import traceback
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, field

import numpy as np
import yaml

PASS, FAIL, NOT_RUN = 'GEÇTİ', 'KALDI', 'ÇALIŞTIRILAMADI'
MODEL = 'forklift'
SIM_TESTS_A = ['T3', 'T4', 'T11', 'T14', 'T7', 'T6', 'T5']
SIM_TESTS_B = ['T12', 'T13', 'T8', 'T9', 'T10']
ALL_TESTS = ['T1', 'T2', 'T3', 'T4', 'T5', 'T6', 'T7', 'T8', 'T9', 'T10', 'T11', 'T12', 'T13', 'T14']

TEST_NAMES = {
    'T1': 'Xacro → URDF, check_urdf, gz sdf -p',
    'T2': 'colcon build --symlink-install',
    'T3': 'Bekleme kararlılığı (spawn sonrası 10 s komutsuz)',
    'T4': 'TF ağacı (view_frames)',
    'T5': 'Düz sürüş 0.2 m/s, 5 s',
    'T6': 'Dönüş 0.5 rad/s, 2π/0.5 s',
    'T7': 'Çatal yüksüz 0 → 0.05 → 0.10 → 0 m',
    'T8': 'Yük alma senaryosu (D-16 seçenek A)',
    'T9': 'Yüklü çatal tutması (0.06 m, 10 s)',
    'T10': 'Ani fren 0.3 m/s → 0, yüklü',
    'T11': 'Kamera yayını',
    'T12': 'Görüş kontrolü (4 kare)',
    'T13': 'Ağırlık merkezi analizi (hesap + sim)',
    'T14': 'Performans (RTF, iki kamera açık)',
}


# =============================================================================
#  Geometri yardımcıları
# =============================================================================
def quat_to_R(q):
    """q = (w, x, y, z) → 3x3 dönüş matrisi."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def pitch_of_R(R):
    """R = Rz(yaw)·Ry(pitch)·Rx(roll) için pitch [rad]; burun aşağı pozitif (REP-103)."""
    return math.asin(max(-1.0, min(1.0, -R[2, 0])))


def tilt_of_R(R):
    """Gövde z ekseninin dünya z ekseniyle açısı [rad]."""
    return math.acos(max(-1.0, min(1.0, R[2, 2])))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Pose:
    """Rijit dönüşüm (p, R)."""

    def __init__(self, p, R):
        self.p, self.R = np.asarray(p, float), np.asarray(R, float)

    @staticmethod
    def from_pq(p, q):
        return Pose(p, quat_to_R(q))

    def __mul__(self, o):
        return Pose(self.p + self.R @ o.p, self.R @ o.R)

    def inv(self):
        return Pose(-self.R.T @ self.p, self.R.T)

    def apply(self, v):
        return self.p + self.R @ np.asarray(v, float)


def T_from_origin(el):
    """URDF <origin xyz rpy> veya SDF <pose> elemanından Pose."""
    if el is None:
        return Pose(np.zeros(3), np.eye(3))
    if el.tag == 'pose':
        v = [float(s) for s in el.text.split()]
        xyz, rpy = v[:3], (v[3:6] if len(v) >= 6 else [0.0, 0.0, 0.0])
    else:
        xyz = [float(s) for s in el.get('xyz', '0 0 0').split()]
        rpy = [float(s) for s in el.get('rpy', '0 0 0').split()]
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    R = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                  [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                  [-sp, cp * sr, cp * cr]])
    return Pose(xyz, R)


# =============================================================================
#  Yapılandırma: tek kaynaklardan okunur (robot YAML, dünya SDF, koli SDF)
# =============================================================================
@dataclass
class Setup:
    ws: str
    docs: str
    desc_share: str
    sim_share: str
    params: dict
    world_name: str
    parcel_size: float
    parcel_mass: float
    parcel_link: str
    parcel_pose: np.ndarray          # koli merkezi, dünya
    rails: list                      # [(merkez xyz dünya, boyut xyz)]
    params_file: str = ''            # kullanılan robot parametre dosyası
    urdf_sim: str = ''               # use_sim:=true URDF metni
    sdf_sim: str = ''                # gz sdf -p çıktısı

    @property
    def base_h(self):                # base_link zemin yüksekliği
        return self.params['drive_wheel']['radius']

    @property
    def carriage_front_x(self):      # taşıyıcı ön yüzü, base_link'e göre
        c = self.params['fork_carriage']
        return c['x'] + c['size']['x'] / 2.0

    @property
    def fork_top_z0(self):           # eklem 0'da çatal üstü, zemin yüksekliği
        f = self.params['fork']
        return f['ground_z_bottom'] + f['size']['z']


def load_setup(ws, docs, params_file=None):
    from ament_index_python.packages import get_package_share_directory
    desc = get_package_share_directory('forklift_description')
    sim = get_package_share_directory('forklift_gazebo')
    params_file = os.path.abspath(params_file or os.path.join(desc, 'config', 'forklift_params.yaml'))
    with open(params_file) as f:
        params = yaml.safe_load(f)
    world = ET.parse(os.path.join(sim, 'worlds', 'lift_test.sdf')).getroot().find('world')
    parcel_pose, rails = None, []
    for inc in world.findall('include'):
        if inc.find('uri').text.strip() == 'model://test_parcel':
            parcel_pose = np.array([float(v) for v in inc.find('pose').text.split()[:3]])
    for m in world.findall('model'):
        if m.get('name') == 'parcel_rails':
            mp = T_from_origin(m.find('pose'))
            for c in m.find('link').findall('collision'):
                cp = mp * T_from_origin(c.find('pose'))
                rails.append((cp.p, np.array([float(v) for v in c.find('geometry/box/size').text.split()])))
    pm = ET.parse(os.path.join(sim, 'models', 'test_parcel', 'model.sdf')).getroot().find('model')
    plink = pm.find('link')
    size = [float(v) for v in plink.find('collision/geometry/box/size').text.split()]
    return Setup(ws=ws, docs=docs, desc_share=desc, sim_share=sim, params=params,
                 world_name=world.get('name'), parcel_size=size[0],
                 parcel_mass=float(plink.find('inertial/mass').text), parcel_link=plink.get('name'),
                 parcel_pose=parcel_pose, rails=rails, params_file=params_file)


def run(cmd, log_path=None, cwd=None, timeout=600):
    """Komutu çalıştırır; (çıkış kodu, stdout, stderr). İsteğe bağlı günlük dosyası."""
    p = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, timeout=timeout)
    if log_path:
        with open(log_path, 'w') as f:
            f.write(f'$ {" ".join(cmd)}\n--- exit {p.returncode}\n--- stdout\n{p.stdout}\n--- stderr\n{p.stderr}')
    return p.returncode, p.stdout, p.stderr


def build_robot_models(st):
    """use_sim:=true URDF ve dönüştürülmüş SDF (T4, T13 için)."""
    xacro_file = os.path.join(st.desc_share, 'urdf', 'forklift.urdf.xacro')
    rc, out, err = run(['xacro', xacro_file, 'use_sim:=true', f'params_file:={st.params_file}'])
    if rc != 0:
        raise RuntimeError(f'xacro başarısız: {err}')
    st.urdf_sim = out
    path = os.path.join(st.docs, 'logs', 'forklift_sim.urdf')
    with open(path, 'w') as f:
        f.write(out)
    rc, out, err = run(['gz', 'sdf', '-p', path])
    if rc != 0:
        raise RuntimeError(f'gz sdf -p başarısız: {err}')
    st.sdf_sim = out


# =============================================================================
#  Sonuç kaydı
# =============================================================================
@dataclass
class Result:
    id: str
    name: str
    status: str
    measured: str
    criterion: str
    details: dict = field(default_factory=dict)


class Report:
    def __init__(self, docs):
        self.docs = docs
        self.results = {}

    def add(self, r):
        self.results[r.id] = r
        print(f'[{r.id}] {r.status:16s} ölçülen: {r.measured} | kriter: {r.criterion}', flush=True)

    def write(self):
        order = [t for t in ALL_TESTS if t in self.results]
        with open(os.path.join(self.docs, 'test_results.json'), 'w') as f:
            json.dump([asdict(self.results[t]) for t in order], f, indent=2, ensure_ascii=False, default=float)
        lines = ['# Kabul testi sonuçları', '',
                 f'Üretim zamanı: {time.strftime("%Y-%m-%d %H:%M:%S")}', '',
                 '| # | Test | Ölçülen | Kriter | Sonuç |', '|---|---|---|---|---|']
        for t in order:
            r = self.results[t]
            lines.append(f'| {r.id} | {r.name} | {r.measured} | {r.criterion} | **{r.status}** |')
        with open(os.path.join(self.docs, 'test_results.md'), 'w') as f:
            f.write('\n'.join(lines) + '\n')


# =============================================================================
#  Simülasyon oturumu: ayrı süreç grubu; kapanışta grubun tamamına sinyal
#  (aksi halde ros2 launch ölse bile gz sim ve köprüler geride kalabiliyor)
# =============================================================================
class SimSession:
    def __init__(self, st, gui, tag):
        self.st, self.gui, self.tag = st, gui, tag
        self.proc = None

    def __enter__(self):
        log = open(os.path.join(self.st.docs, 'logs', f'sim_session_{self.tag}.log'), 'w')
        cmd = ['ros2', 'launch', 'forklift_gazebo', 'sim.launch.py',
               f'gui:={"true" if self.gui else "false"}', f'headless:={"false" if self.gui else "true"}',
               f'params_file:={self.st.params_file}',
               # Testler /cmd_vel'e doğrudan yayınlar; cmd_vel_gate'in sıfırları
               # karışmasın diye komut zinciri kapalı (DECISIONS D-42).
               'control:=false']
        # start_new_session: launch ve tüm alt süreçleri ayrı bir süreç grubunda
        self.proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        print(f'[oturum {self.tag}] başlatıldı (pid {self.proc.pid})', flush=True)
        return self

    def __exit__(self, *exc):
        pgid = os.getpgid(self.proc.pid)
        for sig, wait_s in ((signal.SIGINT, 25), (signal.SIGTERM, 10), (signal.SIGKILL, 5)):
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                break
            t0 = time.time()
            while time.time() - t0 < wait_s and _group_alive(pgid):
                time.sleep(0.2)
            if not _group_alive(pgid):
                break
        self.proc.wait(timeout=5)
        print(f'[oturum {self.tag}] kapandı', flush=True)
        return False


def _group_alive(pgid):
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False


# =============================================================================
#  Gerçek poz: gz-transport aboneliği
# =============================================================================
class GroundTruth:
    def __init__(self, world):
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node as GzNode
        self._lock = threading.Lock()
        self.latest = None
        self._rec = None
        self.first_model_stamp = None
        self.node = GzNode()
        self.topic = f'/world/{world}/dynamic_pose/info'
        if not self.node.subscribe(Pose_V, self.topic, self._cb):
            raise RuntimeError(f'gz aboneliği başarısız: {self.topic}')

    def close(self):
        """Aboneliği yorumlayıcı kapanmadan önce bırakır (gz-transport iş parçacıkları)."""
        if self.node is not None:
            self.node.unsubscribe(self.topic)
            self.node = None

    def _cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        d = {p.name: (np.array([p.position.x, p.position.y, p.position.z]),
                      np.array([p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z]))
             for p in msg.pose}
        with self._lock:
            self.latest = (t, d)
            if MODEL in d and self.first_model_stamp is None:
                self.first_model_stamp = t
            if self._rec is not None:
                self._rec.append((t, d))

    def get(self):
        with self._lock:
            return self.latest

    def start(self):
        with self._lock:
            self._rec = []

    def stop(self):
        with self._lock:
            rec, self._rec = self._rec or [], None
        return rec

    def reset(self):
        with self._lock:
            self.latest, self._rec, self.first_model_stamp = None, None, None


class RobotGT:
    """Gerçek pozdan türetilen robot / koli büyüklükleri."""

    def __init__(self, st):
        self.st = st
        cs = st.params['caster']
        self.caster_c = np.array([cs['x'], 0.0, cs['radius']])      # base_footprint çerçevesinde
        self.caster_r = cs['radius']

    @staticmethod
    def model(d, name):
        p, q = d[name]
        return Pose.from_pq(p, q)

    def base_footprint(self, d):
        return self.model(d, MODEL) * self.model(d, 'base_footprint')

    def base_link(self, d):
        return self.base_footprint(d) * Pose([0, 0, self.st.base_h], np.eye(3))

    def carriage(self, d):
        return self.model(d, MODEL) * self.model(d, 'fork_carriage_link')

    def parcel(self, d):
        return self.model(d, 'test_parcel') * self.model(d, self.st.parcel_link)

    def caster_lift(self, d):
        """Caster küresinin en alt noktasının zemin yüksekliği [m] (≈0 = temas)."""
        return self.base_footprint(d).apply(self.caster_c)[2] - self.caster_r

    def parcel_in_carriage(self, d):
        return (self.carriage(d).inv() * self.parcel(d)).p


# =============================================================================
#  ROS arayüzü (use_sim_time; yürütücü arka plan iş parçacığında)
# =============================================================================
def make_ros_io(st):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image, JointState
    from std_msgs.msg import Float64

    itf = st.params['gazebo']['interface']
    cams = st.params['cameras']

    class RosIO(Node):
        def __init__(self):
            super().__init__('forklift_acceptance_test',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
            self.lock = threading.Lock()
            self.Twist, self.Float64 = Twist, Float64
            self.reset()
            self.cmd_pub = self.create_publisher(Twist, itf['cmd_vel_topic'], 10)
            self.fork_pub = self.create_publisher(Float64, itf['fork_cmd_topic'], 10)
            self.create_subscription(Clock, '/clock', self._on_clock, 10)
            self.create_subscription(Odometry, itf['odom_topic'], self._on_odom, 20)
            self.create_subscription(JointState, itf['joint_states_topic'], self._on_js, 20)
            for name, cam in cams.items():
                self.create_subscription(Image, cam['topic'], self._img_cb(name), qos_profile_sensor_data)
                self.create_subscription(CameraInfo, cam['camera_info_topic'], self._info_cb(name), 10)

        def reset(self):
            with self.lock:
                self.sim_t = None
                self.odom = None   # (t, x, y, yaw, v, w, yaw_unwrapped, frame_id, child_frame_id)
                self.odom_yaw_unwrapped, self._last_yaw = 0.0, None
                self.fork_q, self.fork_rec = None, None
                self.images, self.infos = {}, {}
                self.img_count = {n: 0 for n in cams}
                self.img_stamps = {n: [] for n in cams}

        def _on_clock(self, m):
            with self.lock:
                self.sim_t = m.clock.sec + m.clock.nanosec * 1e-9

        def _on_odom(self, m):
            q = m.pose.pose.orientation
            yaw = yaw_of((q.w, q.x, q.y, q.z))
            with self.lock:
                if self._last_yaw is not None:
                    self.odom_yaw_unwrapped += wrap(yaw - self._last_yaw)
                self._last_yaw = yaw
                self.odom = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x,
                             m.pose.pose.position.y, yaw, m.twist.twist.linear.x, m.twist.twist.angular.z,
                             self.odom_yaw_unwrapped, m.header.frame_id, m.child_frame_id)

        def _on_js(self, m):
            for n, pos in zip(m.name, m.position):
                if n.endswith('fork_lift_joint'):
                    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                    with self.lock:
                        self.fork_q = pos
                        if self.fork_rec is not None:
                            self.fork_rec.append((t, pos))

        def _img_cb(self, name):
            def cb(m):
                with self.lock:
                    self.images[name] = m
                    self.img_count[name] += 1
                    self.img_stamps[name].append((time.monotonic(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
            return cb

        def _info_cb(self, name):
            def cb(m):
                with self.lock:
                    self.infos[name] = m
            return cb

    rclpy.init()
    io = RosIO()
    ex = SingleThreadedExecutor()
    ex.add_node(io)
    threading.Thread(target=ex.spin, daemon=True).start()
    return io, ex


class Driver:
    """Sim zamanına göre komut ve bekleme yardımcıları (açık çevrim + odometri)."""

    def __init__(self, st, io):
        self.st, self.io = st, io
        dd = st.params['gazebo']['diff_drive']
        self.a_lin = dd['max_linear_acceleration']
        self.a_ang = dd['max_angular_acceleration']

    # ---- zaman
    def now(self):
        with self.io.lock:
            return self.io.sim_t

    @staticmethod
    def wait_until(pred, timeout_wall, what):
        t0 = time.time()
        while not pred():
            if time.time() - t0 > timeout_wall:
                raise TimeoutError(f'zaman aşımı: {what}')
            time.sleep(0.005)

    def wait_sim(self, dt, keep_cmd=None):
        t_end = self.now() + dt
        last_pub = 0.0
        while self.now() < t_end:
            if keep_cmd is not None and time.time() - last_pub > 0.1:
                self.cmd(*keep_cmd)
                last_pub = time.time()
            time.sleep(0.002)

    # ---- komutlar
    def cmd(self, v, w):
        m = self.io.Twist()
        m.linear.x, m.angular.z = float(v), float(w)
        self.io.cmd_pub.publish(m)

    def fork(self, h):
        m = self.io.Float64()
        m.data = float(h)
        for _ in range(3):
            self.io.fork_pub.publish(m)

    def odom(self):
        with self.io.lock:
            return self.io.odom

    def fork_q(self):
        with self.io.lock:
            return self.io.fork_q

    def wait_rest(self, timeout=15.0):
        """Odometri hızları ~0 olana kadar (0.3 s sim) bekler."""
        t_ok = None
        t0 = time.time()
        while time.time() - t0 < timeout:
            o = self.odom()
            if abs(o[4]) < 2e-3 and abs(o[5]) < 5e-3:
                t_ok = t_ok or self.now()
                if self.now() - t_ok > 0.3:
                    return
            else:
                t_ok = None
            time.sleep(0.01)
        raise TimeoutError('robot durmadı')

    def drive_for(self, v, w, T):
        self.cmd(v, w)
        self.wait_sim(T, keep_cmd=(v, w))
        self.cmd(0.0, 0.0)

    def drive_distance(self, d, v):
        """Başlangıç yönünde, odometriye göre d [m] (işaretli) ilerler; v > 0.
        Durma komutu, ivme sınırlayıcının fren mesafesi (v²/2a) kadar önce verilir."""
        o0 = self.odom()
        x0, y0, yaw0 = o0[1], o0[2], o0[3]
        sgn = 1.0 if d >= 0 else -1.0
        target = max(abs(d) - v * v / (2.0 * self.a_lin), 0.0)
        self.cmd(sgn * v, 0.0)
        last_pub = time.time()
        while True:
            o = self.odom()
            s = ((o[1] - x0) * math.cos(yaw0) + (o[2] - y0) * math.sin(yaw0)) * sgn
            if s >= target:
                break
            if time.time() - last_pub > 0.1:
                self.cmd(sgn * v, 0.0)
                last_pub = time.time()
            time.sleep(0.002)
        self.cmd(0.0, 0.0)
        self.wait_rest()
        o = self.odom()
        return (o[1] - x0) * math.cos(yaw0) + (o[2] - y0) * math.sin(yaw0)

    def turn_angle(self, theta, w):
        """Odometri yaw'ına göre theta [rad] döner; w > 0."""
        y0 = self.odom()[6]
        sgn = 1.0 if theta >= 0 else -1.0
        target = max(abs(theta) - w * w / (2.0 * self.a_ang), 0.0)
        self.cmd(0.0, sgn * w)
        last_pub = time.time()
        while (self.odom()[6] - y0) * sgn < target:
            if time.time() - last_pub > 0.1:
                self.cmd(0.0, sgn * w)
                last_pub = time.time()
            time.sleep(0.002)
        self.cmd(0.0, 0.0)
        self.wait_rest()
        return self.odom()[6] - y0

    def set_fork_and_wait(self, h, tol=1e-3, timeout=10.0):
        self.fork(h)
        self.wait_until(lambda: self.fork_q() is not None and abs(self.fork_q() - h) < tol,
                        timeout, f'çatal {h} m')
        self.wait_sim(0.3)


# =============================================================================
#  Görüntü yardımcıları
# =============================================================================
def img_to_np(m):
    a = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step)[:, :m.width * 3]
    return a.reshape(m.height, m.width, 3).copy()


def save_png(arr, path):
    from PIL import Image as PILImage
    PILImage.fromarray(arr).save(path)


def classify(arr):
    """Basit renk sınıfları: zemin (gri, düşük doygunluk), koli (kahverengi), mühür (kırmızı)."""
    a = arr.astype(np.int32)
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    spread = a.max(axis=2) - a.min(axis=2)
    seal = (r > 90) & (r > 2 * g) & (r > 2 * b)
    parcel = (~seal) & (r > g) & (g > b) & ((r - b) > 35)
    floor = spread < 22
    return floor, parcel, seal


def _plot(st, fname, title, series, xlabel):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(len(series), 1, figsize=(8, 2.3 * len(series)), sharex=True)
    axes = np.atleast_1d(axes)
    for ax, s in zip(axes, series):
        x, y, label = s[0], s[1], s[2]
        if len(s) > 3:
            ax.step(x, y, where='post')
        else:
            ax.plot(x, y)
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
    axes[0].set_title(title)
    axes[-1].set_xlabel(xlabel)
    fig.tight_layout()
    fig.savefig(os.path.join(st.docs, 'images', fname), dpi=110)
    plt.close(fig)


# =============================================================================
#  T1, T2: simülasyonsuz
# =============================================================================
def t1(st, rep):
    logs = os.path.join(st.docs, 'logs')
    xacro_file = os.path.join(st.desc_share, 'urdf', 'forklift.urdf.xacro')
    rows, ok = [], True
    for use_sim in ('false', 'true'):
        urdf = os.path.join(logs, f'T1_forklift_use_sim_{use_sim}.urdf')
        rc1, out, err1 = run(['xacro', xacro_file, f'use_sim:={use_sim}'], os.path.join(logs, f'T1_xacro_{use_sim}.log'))
        with open(urdf, 'w') as f:
            f.write(out)
        rc2, out2, err2 = run(['check_urdf', urdf], os.path.join(logs, f'T1_check_urdf_{use_sim}.log'))
        rc3, _, err3 = run(['gz', 'sdf', '-p', urdf], os.path.join(logs, f'T1_gz_sdf_{use_sim}.log'))
        warn = [s for s in (err1, err2, err3) if s.strip()] + \
            [ln for ln in out2.splitlines() if re.search(r'warn|error', ln, re.I)]
        good = rc1 == 0 and rc2 == 0 and rc3 == 0 and not warn
        ok &= good
        rows.append(f'use_sim={use_sim}: çıkış kodları {rc1}/{rc2}/{rc3}, uyarı/hata {len(warn)}')
    rep.add(Result('T1', TEST_NAMES['T1'], PASS if ok else FAIL, '; '.join(rows),
                   'Hata ve uyarı yok (çıkış 0, stderr boş)'))


def t2(st, rep):
    log = os.path.join(st.docs, 'logs', 'T2_colcon_build.log')
    rc, out, err = run(['colcon', 'build', '--symlink-install', '--event-handlers', 'console_cohesion+'],
                       log, cwd=st.ws, timeout=900)
    text = out + err
    warns = [ln for ln in text.splitlines() if re.search(r'warn', ln, re.I)]
    stderr_sections = text.count('--- stderr:')
    pkgs = re.search(r'Summary: (\d+) packages? finished', text)
    ok = rc == 0 and not warns and stderr_sections == 0
    rep.add(Result('T2', TEST_NAMES['T2'], PASS if ok else FAIL,
                   f'çıkış {rc}, {pkgs.group(1) if pkgs else "?"} paket, uyarı satırı {len(warns)}, '
                   f'stderr bölümü {stderr_sections}',
                   'Hata ve uyarı yok', {'warnings': warns[:20]}))


# =============================================================================
#  Oturum A
# =============================================================================
def t3(st, rep, gt, drv, rgt):
    """Spawn'dan itibaren 10 s (sim) komutsuz bekleme; gerçek poz."""
    drv.wait_until(lambda: gt.first_model_stamp is not None, 60, 'robot spawn')
    t_spawn = gt.first_model_stamp
    drv.wait_until(lambda: gt.get() and gt.get()[0] >= t_spawn + 10.0, 60, 'T3 10 s')
    rec = [(t, d) for t, d in gt.stop() if t_spawn <= t <= t_spawn + 10.0 and MODEL in d]
    gt.start()
    P = np.array([rgt.base_link(d).p for _, d in rec])
    ts = np.array([t for t, _ in rec]) - t_spawn
    xy = np.linalg.norm(P[:, :2] - P[0, :2], axis=1)
    z_pp = P[:, 2].max() - P[:, 2].min()
    ok = xy.max() < 1e-3 and z_pp < 1e-3
    _plot(st, 't3_idle_stability.png', 'T3 — base_link (gerçek poz), spawn sonrası 10 s',
          [(ts, xy * 1e3, 'x/y kayma [mm]'), (ts, (P[:, 2] - P[0, 2]) * 1e3, 'z − z0 [mm]')], 'sim zamanı [s]')
    rep.add(Result('T3', TEST_NAMES['T3'], PASS if ok else FAIL,
                   f'x/y kayma maks {xy.max() * 1e3:.4f} mm, z tepe-tepe {z_pp * 1e3:.4f} mm '
                   f'({len(rec)} örnek, base_link z0 = {P[0, 2]:.5f} m)',
                   'x/y < 1 mm, z salınımı < 1 mm'))


def t4(st, rep):
    out = os.path.join(st.docs, 'frames_sim')
    rc, _, _ = run(['ros2', 'run', 'tf2_tools', 'view_frames', '--wait-time', '5', '-o', out],
                   os.path.join(st.docs, 'logs', 'T4_view_frames_sim.log'), cwd=st.docs, timeout=60)
    with open(out + '.gv') as f:
        text = f.read()
    edges = re.findall(r'"([^"]+)"\s*->\s*"([^"]+)"', text)
    children = {c for _, c in edges}
    frames = {p for p, _ in edges} | children
    roots = sorted(frames - children)
    links = {ln.get('name') for ln in ET.fromstring(st.urdf_sim).findall('link')}
    expected = links | {st.params['gazebo']['interface']['odom_frame']}
    missing, extra = sorted(expected - frames), sorted(frames - expected)
    optical = sorted(f for f in frames if f.endswith('_optical_frame'))
    ok = rc == 0 and not missing and not extra and len(roots) == 1 and len(optical) == 2
    rep.add(Result('T4', TEST_NAMES['T4'], PASS if ok else FAIL,
                   f'{len(frames)} frame (beklenen {len(expected)}), kök {roots}, optical {optical}, '
                   f'eksik {missing}, fazla {extra}',
                   'Tüm link\'ler + 2 optical frame, tek kök; PDF: docs/frames_sim.pdf'))


def t11(st, rep, drv):
    cams = st.params['cameras']
    with drv.io.lock:
        c0 = dict(drv.io.img_count)
        for n in cams:
            drv.io.img_stamps[n] = []
    w0 = time.monotonic()
    time.sleep(10.0)
    w1 = time.monotonic()
    rows, ok = [], True
    for n, cam in cams.items():
        with drv.io.lock:
            cnt = drv.io.img_count[n] - c0[n]
            img, info = drv.io.images.get(n), drv.io.infos.get(n)
            stamps = list(drv.io.img_stamps[n])
        hz_wall = cnt / (w1 - w0)
        hz_sim = (len(stamps) - 1) / (stamps[-1][1] - stamps[0][1]) if len(stamps) > 1 else 0.0
        expect = f'{n}_optical_frame'
        fx_exp = (cam['image']['width'] / 2.0) / math.tan(cam['horizontal_fov'] / 2.0)
        fx = info.k[0] if info else float('nan')
        good = (hz_wall >= 25.0 and img is not None and img.header.frame_id == expect
                and info is not None and info.header.frame_id == expect and abs(fx - fx_exp) / fx_exp < 0.01)
        ok &= good
        rows.append(f'{n}: {hz_wall:.1f} Hz duvar / {hz_sim:.1f} Hz sim, image frame_id={img.header.frame_id if img else None}, '
                    f'camera_info frame_id={info.header.frame_id if info else None}, fx={fx:.1f} (beklenen {fx_exp:.1f}), '
                    f'{img.width if img else 0}×{img.height if img else 0} {img.encoding if img else ""}')
    rep.add(Result('T11', TEST_NAMES['T11'], PASS if ok else FAIL, '; '.join(rows),
                   'Her kamera ≥ 25 Hz (duvar saati), frame_id optical frame'))


def t14(st, rep, drv):
    w0, s0 = time.monotonic(), drv.now()
    time.sleep(20.0)
    w1, s1 = time.monotonic(), drv.now()
    rtf = (s1 - s0) / (w1 - w0)
    rep.add(Result('T14', TEST_NAMES['T14'], PASS if rtf >= 0.9 else FAIL,
                   f'RTF {rtf:.3f} (20 s; ekransız sunucu, iki kamera abone; /clock / duvar saati)',
                   'RTF ≥ 0.9 (iki kamera açık)'))


def t7(st, rep, drv):
    lj = st.params['fork_lift_joint']
    drv.set_fork_and_wait(0.0)
    with drv.io.lock:
        drv.io.fork_rec = []
    rows, ok, events = [], True, []
    for target in (0.05, 0.10, 0.0):
        q_start = drv.fork_q()
        t_cmd = drv.now()
        drv.fork(target)
        events.append((t_cmd, target))
        drv.wait_sim(4.0)
        with drv.io.lock:
            seg = [(t, q) for t, q in drv.io.fork_rec if t >= t_cmd]
        ts = np.array([t for t, _ in seg]) - t_cmd
        qs = np.array([q for _, q in seg])
        outside = np.where(np.abs(qs - target) > 5e-3)[0]
        if len(outside) == 0:
            settle = 0.0
        elif outside[-1] + 1 < len(ts):
            settle = float(ts[outside[-1] + 1])
        else:
            settle = float('inf')
        up = target > q_start
        overshoot = max(0.0, (qs.max() - target) if up else (target - qs.min()))
        final_err = abs(qs[-1] - target)
        good = settle <= 3.0 and overshoot < 10e-3 and final_err <= 5e-3
        ok &= good
        rows.append(f'{q_start:.3f}→{target:.2f} m: ±5 mm\'ye {settle:.2f} s, aşım {overshoot * 1e3:.2f} mm, '
                    f'son hata {final_err * 1e3:.2f} mm')
    with drv.io.lock:
        rec, drv.io.fork_rec = drv.io.fork_rec, None
    t0 = events[0][0]
    _plot(st, 't7_fork_unloaded.png', 'T7 — çatal yüksüz (fork_lift_joint, /joint_states)',
          [(np.array([t for t, _ in rec]) - t0, np.array([q for _, q in rec]) * 1e3, 'konum [mm]'),
           (np.array([t for t, _ in events] + [rec[-1][0]]) - t0,
            np.array([q for _, q in events] + [events[-1][1]]) * 1e3, 'hedef [mm]', 'step')],
          'sim zamanı [s]')
    mode = 'hız modu' if st.params['gazebo']['fork_controller']['use_velocity_commands'] else 'PID modu'
    rep.add(Result('T7', TEST_NAMES['T7'], PASS if ok else FAIL,
                   f'[{mode}, hız sınırı {lj["velocity"]} m/s] ' + '; '.join(rows),
                   'Her hedefe 3 s içinde ± 5 mm, aşım < 10 mm'))


def t6(st, rep, gt, drv):
    drv.wait_rest()
    y0_odom = drv.odom()[6]
    gt.start()
    drv.drive_for(0.0, 0.5, 2 * math.pi / 0.5)
    drv.wait_rest()
    drv.wait_sim(0.5)
    rec = gt.stop()
    gt.start()
    yaws = np.unwrap([yaw_of(d[MODEL][1]) for _, d in rec if MODEL in d])
    dyaw_gt = math.degrees(yaws[-1] - yaws[0])
    dyaw_odom = math.degrees(drv.odom()[6] - y0_odom)
    ok = abs(dyaw_odom - 360.0) <= 5.0 and abs(dyaw_gt - 360.0) <= 5.0
    rep.add(Result('T6', TEST_NAMES['T6'], PASS if ok else FAIL,
                   f'odometri {dyaw_odom:.2f}°, gerçek {dyaw_gt:.2f}°', '360° ± 5° (odometri ve gerçek)'))


def t5(st, rep, gt, drv, rgt):
    drv.wait_rest()
    o0, g0 = drv.odom(), rgt.base_link(gt.get()[1])
    drv.drive_for(0.2, 0.0, 5.0)
    drv.wait_rest()
    drv.wait_sim(0.5)
    o1, g1 = drv.odom(), rgt.base_link(gt.get()[1])
    d_odom = math.hypot(o1[1] - o0[1], o1[2] - o0[2])
    d_gt = float(np.linalg.norm(g1.p[:2] - g0.p[:2]))
    diff = abs(d_odom - d_gt) / d_gt
    lat = abs((g1.p[:2] - g0.p[:2]) @ np.array([-g0.R[1, 0], g0.R[0, 0]]))
    ok = abs(d_odom - 1.0) <= 0.05 and diff < 0.05
    rep.add(Result('T5', TEST_NAMES['T5'], PASS if ok else FAIL,
                   f'odometri {d_odom:.4f} m, gerçek {d_gt:.4f} m, fark %{diff * 100:.2f}, yanal sapma {lat * 1e3:.1f} mm',
                   'Odometri 1.00 m ± %5; gerçek pozla fark < %5'))


# =============================================================================
#  Oturum B: T12, T13, T8 (+T9, T10)
# =============================================================================
def com_analytic(st, q_fork, parcel_gap=None):
    """URDF (YAML) kütle dağılımından ağırlık merkezi; base_link çerçevesinde."""
    root = ET.fromstring(st.urdf_sim)
    links = {ln.get('name'): ln for ln in root.findall('link')}
    parent_of = {j.find('child').get('link'): j for j in root.findall('joint')}

    def T_root(link):
        if link not in parent_of:
            return Pose(np.zeros(3), np.eye(3))
        j = parent_of[link]
        T = T_root(j.find('parent').get('link')) * T_from_origin(j.find('origin'))
        if j.get('type') == 'prismatic':
            axis = np.array([float(v) for v in j.find('axis').get('xyz').split()])
            T = T * Pose(axis * q_fork, np.eye(3))
        return T
    M, mc = 0.0, np.zeros(3)
    for name, ln in links.items():
        i = ln.find('inertial')
        if i is None:
            continue
        m = float(i.find('mass').get('value'))
        M += m
        mc += m * (T_root(name) * T_from_origin(i.find('origin'))).p
    if parcel_gap is not None:
        a = st.parcel_size
        pc = np.array([st.carriage_front_x + parcel_gap + a / 2.0, 0.0, st.fork_top_z0 + q_fork + a / 2.0])
        M += st.parcel_mass
        mc += st.parcel_mass * pc
    c = mc / M
    c[2] -= st.base_h          # base_footprint (zemin) → base_link
    return M, c


def com_sim(st, d, rgt, with_parcel):
    """gz'nin yüklediği birleştirilmiş SDF kütleleri + canlı link/koli pozları; base_link çerçevesinde."""
    sdf = ET.fromstring(st.sdf_sim).find('model')
    model = rgt.model(d, MODEL)
    M, mc = 0.0, np.zeros(3)
    for ln in sdf.findall('link'):
        i = ln.find('inertial')
        m = float(i.find('mass').text)
        M += m
        mc += m * (model * rgt.model(d, ln.get('name')) * T_from_origin(i.find('pose'))).p
    if with_parcel:
        M += st.parcel_mass
        mc += st.parcel_mass * rgt.parcel(d).p
    return M, rgt.base_link(d).inv().apply(mc / M)


def support_margins(st, c):
    """Destek marjları: x aralığı [caster x, 0] (ön, arka) ve gerçek destek üçgeninin kenarları."""
    sep = st.params['drive_wheel']['separation'] / 2.0
    xc = st.params['caster']['x']
    A, B, C = (0.0, sep), (0.0, -sep), (xc, 0.0)

    def dist(P, Q, R):
        (x1, y1), (x2, y2) = P, Q
        return ((x2 - x1) * (R[1] - y1) - (y2 - y1) * (R[0] - x1)) / math.hypot(x2 - x1, y2 - y1)
    return -c[0], c[0] - xc, min(dist(B, A, c[:2]), dist(A, C, c[:2]), dist(C, B, c[:2]))


def session_b(st, rep, gt, drv, rgt):
    imgdir = os.path.join(st.docs, 'images')
    drv.wait_rest()
    drv.wait_sim(1.0)
    frames = {}

    def grab(name, tag):
        with drv.io.lock:
            c0 = drv.io.img_count[name]
        drv.wait_until(lambda: drv.io.img_count[name] >= c0 + 3, 10, f'{name} karesi')
        with drv.io.lock:
            arr = img_to_np(drv.io.images[name])
        save_png(arr, os.path.join(imgdir, f't12_{name}_{tag}.png'))
        frames[(name, tag)] = arr

    # ---- T12 yüksüz kareler, T13 yüksüz (robot orijinde, koli 1.5 m önde)
    grab('front_camera', 'unloaded_1p5m')
    grab('lane_camera', 'unloaded')
    com_un = (com_analytic(st, 0.0), com_sim(st, gt.get()[1], rgt, False))

    # ---- T8 yaklaşma: açık çevrim, bilinen pozlar (koli SDF'ten) + odometri
    gap = 0.01                                                   # DECISIONS D-17
    x_target = st.parcel_pose[0] - st.parcel_size / 2.0 - st.carriage_front_x - gap
    bl0 = rgt.base_link(gt.get()[1])
    d_app = drv.drive_distance(x_target - bl0.p[0], 0.10)
    bl_pick = rgt.base_link(gt.get()[1])
    o_pick = drv.odom()

    # ---- kaldır
    h_carry = 0.06
    drv.set_fork_and_wait(h_carry)
    drv.wait_sim(1.0)
    parcel_lift = rgt.parcel(gt.get()[1])

    # ---- T9: yüklü 10 s tutma
    with drv.io.lock:
        drv.io.fork_rec = []
    gt.start()
    drv.wait_sim(10.0)
    rec9 = gt.stop()
    with drv.io.lock:
        js9, drv.io.fork_rec = drv.io.fork_rec, None
    q9 = np.array([q for _, q in js9])
    cz = np.array([(rgt.base_link(dd).inv() * rgt.carriage(dd)).p[2] for _, dd in rec9])
    sag_joint = max(0.0, h_carry - q9.min())
    sag_gt = max(0.0, cz[0] - cz.min())
    rep.add(Result('T9', TEST_NAMES['T9'], PASS if max(sag_joint, sag_gt) < 5e-3 else FAIL,
                   f'eklem sarkması {sag_joint * 1e3:.3f} mm, taşıyıcı (gerçek poz) sarkması {sag_gt * 1e3:.3f} mm; '
                   f'koli tabanı z = {parcel_lift.p[2] - st.parcel_size / 2:.4f} m',
                   'Sarkma < 5 mm'))

    # ---- T12 yüklü kareler, T13 yüklü
    grab('lane_camera', 'loaded')
    grab('front_camera', 'loaded')
    d = gt.get()[1]
    com_lo = (com_analytic(st, h_carry, gap), com_sim(st, d, rgt, True))
    p_rel0 = rgt.parcel_in_carriage(d)

    # ---- T8 taşıma: geri 1 m → +90° → [T10] → −90° → ileri (kaldırma noktası)
    gt.start()
    t_tr0 = drv.now()
    drv.drive_distance(-1.0, 0.15)
    drv.turn_angle(math.pi / 2, 0.5)
    # T10: yüklü, ileri 0.3 m/s; sabit hıza ulaştıktan sonra basamak komutla dur (DECISIONS D-19)
    o10 = drv.odom()
    drv.cmd(0.3, 0.0)
    last = time.time()
    while math.hypot(drv.odom()[1] - o10[1], drv.odom()[2] - o10[2]) < 0.25:
        if time.time() - last > 0.1:
            drv.cmd(0.3, 0.0)
            last = time.time()
        time.sleep(0.002)
    v_brake = drv.odom()[4]
    t_brake = drv.now()
    drv.cmd(0.0, 0.0)
    drv.wait_rest()
    drv.wait_sim(0.5)
    t_brake_end = drv.now()
    s_out = math.hypot(drv.odom()[1] - o10[1], drv.odom()[2] - o10[2])
    drv.drive_distance(-s_out, 0.15)
    drv.turn_angle(-math.pi / 2, 0.5)
    o = drv.odom()
    drv.drive_distance((o_pick[1] - o[1]) * math.cos(o[3]) + (o_pick[2] - o[2]) * math.sin(o[3]), 0.15)
    t_tr1 = drv.now()
    bl_return = rgt.base_link(gt.get()[1])
    rec8 = gt.stop()

    # ---- indir, 0.3 m geri çık
    drv.set_fork_and_wait(0.0)
    drv.wait_sim(0.5)
    drv.drive_distance(-0.30, 0.10)
    drv.wait_sim(1.0)
    parcel_end = rgt.parcel(gt.get()[1])

    # ---- T8 değerlendirme
    rel = np.array([rgt.parcel_in_carriage(dd) for _, dd in rec8])
    slip = np.linalg.norm(rel - p_rel0, axis=1)
    pz_bottom = np.array([rgt.parcel(dd).p[2] for _, dd in rec8]) - st.parcel_size / 2.0
    ptilt = np.array([math.degrees(tilt_of_R(rgt.parcel(dd).R)) for _, dd in rec8])
    tilt_end = math.degrees(tilt_of_R(parcel_end.R))
    z_end = parcel_end.p[2] - st.parcel_size / 2.0
    rail_top = max(c[2] + s[2] / 2.0 for c, s in st.rails)
    on_rails = (abs(z_end - rail_top) < 3e-3 and abs(parcel_end.p[0] - st.parcel_pose[0]) < 0.05
                and abs(parcel_end.p[1] - st.parcel_pose[1]) < 0.03)
    dropped = bool(pz_bottom.min() < rail_top - 0.01)
    ok8 = slip.max() < 0.01 and not dropped and tilt_end < 2.0 and on_rails
    ts8 = np.array([t for t, _ in rec8]) - t_tr0
    _plot(st, 't8_parcel_slip.png', 'T8 — koli/taşıyıcı bağıl kayma (taşıma boyunca, gerçek poz)',
          [(ts8, slip * 1e3, 'bağıl kayma [mm]'), (ts8, ptilt, 'koli eğimi [°]'), (ts8, pz_bottom * 1e3, 'koli tabanı z [mm]')],
          'taşıma başından itibaren sim zamanı [s]')
    rep.add(Result('T8', TEST_NAMES['T8'], PASS if ok8 else FAIL,
                   f'yaklaşma {d_app:.3f} m → base_link x={bl_pick.p[0]:.4f} m, y={bl_pick.p[1] * 1e3:+.1f} mm '
                   f'(hedef x={x_target:.3f}); dönüşte x={bl_return.p[0]:.4f}, y={bl_return.p[1] * 1e3:+.1f} mm; '
                   f'taşıma {t_tr1 - t_tr0:.1f} s, maks bağıl kayma {slip.max() * 1e3:.2f} mm, maks koli eğimi {ptilt.max():.2f}°; '
                   f'düşme {"VAR" if dropped else "yok"}; bırakınca koli eğimi {tilt_end:.2f}°, taban z={z_end:.4f} m '
                   f'(ray üstü {rail_top:.3f}), konum x={parcel_end.p[0]:.4f} y={parcel_end.p[1] * 1e3:+.1f} mm',
                   'Taşıma boyunca bağıl kayma < 1 cm; koli düşmüyor; indirince raylarda dik'))

    # ---- T10 değerlendirme (fren penceresi, gerçek poz)
    win = [(t, dd) for t, dd in rec8 if t_brake - 0.5 <= t <= t_brake_end]
    tw = np.array([t for t, _ in win]) - t_brake
    lift = np.array([rgt.caster_lift(dd) for _, dd in win])
    pitch = np.array([math.degrees(pitch_of_R(rgt.base_footprint(dd).R)) for _, dd in win])
    lift0 = float(np.median(lift[tw < 0])) if np.any(tw < 0) else float(lift[0])
    pitch0 = float(np.median(pitch[tw < 0])) if np.any(tw < 0) else float(pitch[0])
    slip10 = np.linalg.norm(np.array([rgt.parcel_in_carriage(dd) for _, dd in win]) - p_rel0, axis=1)
    lift_rel, dpitch = lift - lift0, pitch - pitch0
    ok10 = lift_rel.max() < 1e-3 and np.abs(dpitch).max() < 1.0 and slip10.max() < 0.01
    _plot(st, 't10_brake.png', 'T10 — yüklü ani fren (0.3 m/s → 0, basamak komut)',
          [(tw, lift_rel * 1e3, 'caster kalkması [mm]'), (tw, dpitch, 'Δpitch [°]'), (tw, slip10 * 1e3, 'koli kayması [mm]')],
          'fren komutundan itibaren sim zamanı [s]')
    rep.add(Result('T10', TEST_NAMES['T10'], PASS if ok10 else FAIL,
                   f'fren anında v={v_brake:.3f} m/s; caster kalkması maks {lift_rel.max() * 1e3:.3f} mm '
                   f'(mutlak en alt nokta z min/maks {lift.min() * 1e3:.3f}/{lift.max() * 1e3:.3f} mm); '
                   f'|Δpitch| maks {np.abs(dpitch).max():.3f}°; koli kayması maks {slip10.max() * 1e3:.2f} mm',
                   'Devrilme yok, arka teker temas kesmiyor (caster kalkması < 1 mm, gerçek poz; D-18)'))

    # ---- T12 değerlendirme
    rows12, ok12 = [], True
    for tag in ('unloaded_1p5m', 'loaded'):
        _, _, seal = classify(frames[('front_camera', tag)])
        n_seal = int(seal.sum())
        ys, xs = np.nonzero(seal)
        box = (int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())) if n_seal else None
        if tag == 'unloaded_1p5m':
            ok12 &= n_seal > 500
        rows12.append(f'front {tag}: mühür {n_seal} px, kutu {box}')
    for tag in ('unloaded', 'loaded'):
        floor, parcel, _ = classify(frames[('lane_camera', tag)])
        h, w = floor.shape
        center = floor[h // 2 - 10:h // 2 + 11, w // 2 - 10:w // 2 + 11].mean()
        col = parcel[h // 2:, w // 2]
        edge = int(h // 2 + np.argmax(col)) if col.any() else None
        ok12 &= center > 0.9
        rows12.append(f'lane {tag}: merkez 21×21 zemin %{center * 100:.0f}, orta sütunda koli üst kenarı satırı '
                      f'{edge if edge is not None else "yok"}')
    rep.add(Result('T12', TEST_NAMES['T12'], PASS if ok12 else FAIL, '; '.join(rows12),
                   'front koliyi 1.5 m\'den görüyor (mühür görünür); lane yüksüz/yüklü merkez bölge zemin',
                   {'frames': [f'docs/images/t12_{n}_{t}.png' for n, t in frames]}))

    # ---- T13 değerlendirme
    rows13, ok13 = [], True
    for tag, ((Ma, ca), (Ms, cs)) in (('yüksüz', com_un), ('yüklü', com_lo)):
        fa, ra, ta = support_margins(st, ca)
        fs, rs, ts_ = support_margins(st, cs)
        ok13 &= min(fa, ra, fs, rs) >= 0.03
        rows13.append(f'{tag}: hesap x={ca[0]:+.4f} m, zemin z={ca[2] + st.base_h:.4f} m (M={Ma:.2f} kg); '
                      f'sim x={cs[0]:+.4f} m, zemin z={cs[2] + st.base_h:.4f} m (M={Ms:.2f} kg); '
                      f'x-marjı ön {min(fa, fs) * 100:.2f} cm / arka {min(ra, rs) * 100:.2f} cm; '
                      f'destek üçgeni marjı {min(ta, ts_) * 100:.2f} cm')
    rep.add(Result('T13', TEST_NAMES['T13'], PASS if ok13 else FAIL, '; '.join(rows13),
                   'Destek poligonuna (x ∈ [-0.28, 0]) marj ≥ 3 cm'))


# =============================================================================
#  Ana akış
# =============================================================================
def main():
    # Arka planda başlatılan süreçler SIGINT'i yok sayarak devralabilir; Python
    # işleyicisi kurulunca exec sonrası alt süreçler SIGINT'i varsayılan alır
    # (ros2 launch'ın temiz kapanması için gerekli).
    signal.signal(signal.SIGINT, signal.default_int_handler)
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tests', default=','.join(ALL_TESTS))
    ap.add_argument('--ws', default=os.path.expanduser('~/cv_forklift_ws'))
    ap.add_argument('--docs', default=None)
    ap.add_argument('--gui', action='store_true')
    ap.add_argument('--params', default=None)
    a = ap.parse_args()
    wanted = [t.strip().upper() for t in a.tests.split(',') if t.strip()]
    docs = a.docs or os.path.join(a.ws, 'docs')
    for sub in ('', 'images', 'logs'):
        os.makedirs(os.path.join(docs, sub), exist_ok=True)

    st = load_setup(a.ws, docs, a.params)
    print(f'parametre dosyası: {st.params_file}', flush=True)
    rep = Report(docs)
    print(f'testler: {wanted}', flush=True)

    def guarded(tid, fn, *args):
        if tid not in wanted:
            return
        try:
            fn(*args)
        except Exception as e:  # noqa: BLE001 — istisna rapora ÇALIŞTIRILAMADI olarak yazılır
            traceback.print_exc()
            rep.add(Result(tid, TEST_NAMES[tid], NOT_RUN, f'istisna: {type(e).__name__}: {e}', '-'))

    guarded('T1', t1, st, rep)
    guarded('T2', t2, st, rep)
    need_a = [t for t in SIM_TESTS_A if t in wanted]
    need_b = [t for t in SIM_TESTS_B if t in wanted]
    if need_a or need_b:
        build_robot_models(st)
        io, ex = make_ros_io(st)
        gt = GroundTruth(st.world_name)
        drv = Driver(st, io)
        rgt = RobotGT(st)

        def ready():
            g = gt.get()
            return (g is not None and MODEL in g[1] and drv.odom() is not None and drv.fork_q() is not None
                    and drv.now() is not None and all(io.img_count[n] > 0 for n in st.params['cameras']))

        for tag, need in (('A', need_a), ('B', need_b)):
            if not need:
                continue
            gt.reset()
            io.reset()
            gt.start()
            with SimSession(st, a.gui, tag):
                try:
                    drv.wait_until(ready, 120, f'oturum {tag} hazır')
                    if tag == 'A':
                        guarded('T3', t3, st, rep, gt, drv, rgt)
                        guarded('T4', t4, st, rep)
                        guarded('T11', t11, st, rep, drv)
                        guarded('T14', t14, st, rep, drv)
                        guarded('T7', t7, st, rep, drv)
                        guarded('T6', t6, st, rep, gt, drv)
                        guarded('T5', t5, st, rep, gt, drv, rgt)
                    else:
                        session_b(st, rep, gt, drv, rgt)
                except Exception as e:  # noqa: BLE001
                    traceback.print_exc()
                    for t in need:
                        if t not in rep.results:
                            rep.add(Result(t, TEST_NAMES[t], NOT_RUN, f'oturum {tag} hatası: {type(e).__name__}: {e}', '-'))
            gt.stop()
        ex.shutdown()
        io.destroy_node()
        gt.close()
        import rclpy
        rclpy.try_shutdown()
    rep.write()
    print('\n==== ÖZET ====')
    for t in [t for t in ALL_TESTS if t in rep.results]:
        r = rep.results[t]
        print(f'{t:4s} {r.status:16s} {r.name}')
    return 0 if rep.results and all(r.status == PASS for r in rep.results.values()) else 1


if __name__ == '__main__':
    code = main()
    # gz-transport Python bağlamasının arka plan iş parçacıkları yorumlayıcı
    # kapanışında yıkılırken ara sıra std::terminate çağırıyor (çıkış kodu 250).
    # Sonuçlar bu noktada yazılmış olduğundan çıktılar boşaltılıp süreç
    # doğrudan sonlandırılır; çıkış kodu testlerin sonucunu yansıtır.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
