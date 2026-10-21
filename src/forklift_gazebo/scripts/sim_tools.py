"""
sim_tools.py — simülasyon testleri için ortak gz yardımcıları

station_acceptance.py (S testleri) ve forklift_evaluation (C/A testleri) içe aktarır.
İçerik: poz matematiği (Pose, Rz, Ry, ...), istasyon/bölme çerçeveleri, simülasyon
oturumu (SimSession), gz-transport arayüzü (Gz: gerçek poz, sim zamanı, kameralar,
komutlar, servisler), robot geometrisi ve yerleştirme yardımcıları.

YALNIZ TEST İÇİN: gz pozları ground truth'tur; robotun algı/otonomi kodu bu
modülü kullanmaz (docs/PLAN_CV_AUTONOMY.md, kural R2).

Fonksiyonlara verilen `st` nesnesinde şu alanlar beklenir:
  world (dünya adı), gui (bool), logs (log dizini), robot (forklift_params.yaml),
  params (station_params.yaml)
"""

import math
import os
import signal
import subprocess
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import station_layout as sl  # noqa: E402


# =============================================================================
#  Poz yardımcıları
# =============================================================================
def quat_to_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return np.array([w, x, y, z])


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def tilt_deg(R):
    """Gövde z ekseninin düşeyle açısı [°]."""
    return math.degrees(math.acos(max(-1.0, min(1.0, R[2, 2]))))


def yaw_of_R(R):
    return math.atan2(R[1, 0], R[0, 0])


class Pose:
    def __init__(self, p, R):
        self.p, self.R = np.asarray(p, float), np.asarray(R, float)

    @classmethod
    def from_pq(cls, p, q):
        return cls(p, quat_to_R(q))

    def __mul__(self, o):
        return Pose(self.p + self.R @ o.p, self.R @ o.R)

    def inv(self):
        return Pose(-self.R.T @ self.p, self.R.T)

    def apply(self, v):
        return self.p + self.R @ np.asarray(v, float)


def station_pose(st_entry):
    x, y, yaw = st_entry['pose']
    return Pose([x, y, 0.0], Rz(yaw))


def bay_pose(st, st_entry, i):
    """bay_i çerçevesinin dünya pozu."""
    return station_pose(st_entry) * Pose([0, sl.bay_y(st.params, st_entry['bays'], i), 0], np.eye(3))


# =============================================================================
#  Simülasyon oturumu: ayrı süreç grubu; kapanışta grubun tamamına sinyal
#  (aksi halde gz sim sunucusu geride kalabiliyor)
# =============================================================================
class SimSession:
    """station_sim.launch.py'yi ayrı süreç grubunda başlatır (st.world dünyasında).

    control=False: komut zinciri yok, test /cmd_vel'e doğrudan yayın yapar (D-42).
    control=True : zincir açık; otonomi testleri /cmd_vel/auto ve /e_stop kullanır.
    extra: launch'a eklenecek 'ad:=değer' argümanları (ör. stations_file).
    """

    def __init__(self, st, tag, control=False, extra=()):
        self.st, self.tag, self.proc = st, tag, None
        self.control, self.extra = control, list(extra)
        self.log_path = os.path.join(st.logs, f'sim_session_{tag}.log')

    def __enter__(self):
        log = open(self.log_path, 'w')
        cmd = ['ros2', 'launch', 'forklift_gazebo', 'station_sim.launch.py',
               f'station_world:={self.st.world}',
               f'gui:={"true" if self.st.gui else "false"}',
               f'headless:={"false" if self.st.gui else "true"}',
               f'control:={"true" if self.control else "false"}'] + self.extra
        self.proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        print(f'[oturum {self.tag}] başlatıldı (pid {self.proc.pid})', flush=True)
        return self

    def spawned(self):
        with open(self.log_path) as f:
            return 'stations.launch:' in f.read()

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
#  gz-transport arayüzü: gerçek poz, sim zamanı, kameralar, komutlar, servisler
# =============================================================================
class Gz:
    MODEL = 'forklift'

    def __init__(self, st):
        from gz.msgs10.double_pb2 import Double
        from gz.msgs10.image_pb2 import Image
        from gz.msgs10.model_pb2 import Model
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.twist_pb2 import Twist
        from gz.msgs10.world_stats_pb2 import WorldStatistics
        from gz.transport13 import Node as GzNode
        self.st, self.w = st, st.world
        self.Twist, self.Double = Twist, Double
        self.lock = threading.Lock()
        self.node = GzNode()
        self.poses, self.pose_t = {}, None
        self.sim_t = None
        self.fork_q = None
        self.images, self.img_count = {}, {}
        self._rec = None
        itf = st.robot['gazebo']['interface']
        self.cam_topics = {n: c['topic'] for n, c in st.robot['cameras'].items()}
        self.extra_cams = {}
        subs = [(Pose_V, f'/world/{self.w}/dynamic_pose/info', self._on_pose),
                (WorldStatistics, f'/world/{self.w}/stats', self._on_stats),
                (Model, itf['joint_states_topic'], self._on_js)]
        for n, t in self.cam_topics.items():
            subs.append((Image, t, self._img_cb(n)))
        for typ, topic, cb in subs:
            if not self.node.subscribe(typ, topic, cb):
                raise RuntimeError(f'gz aboneliği başarısız: {topic}')
        self.Image = Image
        self.cmd_pub = self.node.advertise(itf['cmd_vel_topic'], Twist)
        self.fork_pub = self.node.advertise(itf['fork_cmd_topic'], Double)

    # ------------------------------------------------------------ abonelikler
    def _on_pose(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        d = {p.name: (np.array([p.position.x, p.position.y, p.position.z]),
                      np.array([p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z]))
             for p in msg.pose}
        with self.lock:
            self.poses, self.pose_t = d, t
            if self._rec is not None:
                self._rec.append((t, d))

    def _on_stats(self, msg):
        with self.lock:
            self.sim_t = msg.sim_time.sec + msg.sim_time.nsec * 1e-9

    def _on_js(self, msg):
        for j in msg.joint:
            if j.name.endswith('fork_lift_joint'):
                with self.lock:
                    self.fork_q = j.axis1.position

    def _img_cb(self, name):
        def cb(msg):
            arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)[:, :, :3].copy()
            t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
            with self.lock:
                self.images[name] = (t, arr)
                self.img_count[name] = self.img_count.get(name, 0) + 1
        return cb

    def subscribe_camera(self, name, topic):
        self.node.subscribe(self.Image, topic, self._img_cb(name))

    # ------------------------------------------------------------- okumalar
    def now(self):
        with self.lock:
            return self.sim_t

    def model(self, name):
        """Modelin dünya pozu (dynamic_pose: yalnız dinamik modeller)."""
        with self.lock:
            v = self.poses.get(name)
        return None if v is None else Pose.from_pq(*v)

    def link(self, model, link):
        with self.lock:
            m, lk = self.poses.get(model), self.poses.get(link)
        return None if m is None or lk is None else Pose.from_pq(*m) * Pose.from_pq(*lk)

    def carriage(self):
        return self.link(self.MODEL, 'fork_carriage_link')

    def fork(self):
        with self.lock:
            return self.fork_q

    def image(self, name, newer_than=None, timeout=5.0):
        """Sim zamanı newer_than'dan sonra yakalanmış kare (RGB)."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.lock:
                v = self.images.get(name)
            if v is not None and (newer_than is None or v[0] > newer_than):
                return v[1]
            time.sleep(0.02)
        return None

    def record(self, on):
        with self.lock:
            if on:
                self._rec = []
                return None
            rec, self._rec = self._rec or [], None
            return rec

    # -------------------------------------------------------------- zaman
    def wait_sim(self, dt, timeout=None):
        w0 = time.time()
        limit = timeout if timeout is not None else 10 * dt + 10
        while self.now() is None:
            if time.time() - w0 > limit:
                raise TimeoutError('sim zamanı gelmiyor (/stats)')
            time.sleep(0.05)
        t0 = self.now()
        while self.now() is None or self.now() - t0 < dt:
            if time.time() - w0 > limit:
                raise TimeoutError(f'sim zamanı {dt} s ilerlemedi')
            time.sleep(0.01)

    def wait_until(self, cond, timeout, what):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if cond():
                return
            time.sleep(0.05)
        raise TimeoutError(what)

    # ----------------------------------------------------------- komutlar
    def cmd(self, v, w=0.0):
        m = self.Twist()
        m.linear.x, m.angular.z = v, w
        self.cmd_pub.publish(m)

    def set_fork(self, q):
        m = self.Double()
        m.data = q
        self.fork_pub.publish(m)

    def _req(self, service, req, req_t, rep_t, timeout_ms=5000, attempts=3):
        """Servis isteği AYRI SÜREÇTEN (gz service CLI) gönderilir (D-49).

        Bu süreçteki sürekli abonelikler (poz 60 Hz, eklem 50 Hz, iki kamera 30 Hz)
        varken gz-transport Python Node.request() yanıtları zaman aşımına düşüyor
        (create/blocking hiç dönmedi, set_pose ara sıra 5 s bekledi); ayrı süreçte
        aynı istekler ~0.3 s'de dönüyor.
        """
        from google.protobuf import text_format
        text = text_format.MessageToString(req, as_one_line=True)
        cmd = ['gz', 'service', '-s', service, '--reqtype', f'gz.msgs.{req_t.__name__}',
               '--reptype', f'gz.msgs.{rep_t.__name__}', '--timeout', str(timeout_ms), '--req', text]
        tried = []
        for _ in range(attempts):
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_ms / 1000.0 + 20)
            if 'data: true' in p.stdout:
                return True
            tried.append((p.stdout + p.stderr).strip()[:120] or 'yanıt yok')
            time.sleep(0.5)
        raise RuntimeError(f'{service} başarısız: {tried}')

    def set_pose(self, name, p, R):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.pose_pb2 import Pose as PoseMsg
        m = PoseMsg()
        m.name = name
        m.position.x, m.position.y, m.position.z = (float(v) for v in p)
        q = R_to_quat(np.asarray(R))
        m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = (float(v) for v in q)
        self._req(f'/world/{self.w}/set_pose', m, PoseMsg, Boolean)

    def spawn(self, name, sdf, p, R, timeout_ms=60000):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        m = EntityFactory()
        m.sdf, m.name, m.allow_renaming = sdf, name, False
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in p)
        q = R_to_quat(np.asarray(R))
        m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z = \
            (float(v) for v in q)
        # blocking: model gerçekten eklenince döner; ad çakışmasında data=false
        # (async /create her durumda true döner, Faz 4'te ölçüldü)
        # tek deneme, uzun zaman aşımı: tekrar denemek ad çakışması (data=false) verir
        self._req(f'/world/{self.w}/create/blocking', m, EntityFactory, Boolean, timeout_ms, attempts=1)

    def remove(self, name):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.entity_pb2 import Entity
        m = Entity()
        m.name, m.type = name, Entity.MODEL
        self._req(f'/world/{self.w}/remove/blocking', m, Entity, Boolean, attempts=1)

    def close(self):
        with self.lock:
            self._rec = None
        for n in list(self.cam_topics.values()):
            self.node.unsubscribe(n)


def save_rgb(arr, path):
    import cv2
    cv2.imwrite(path, cv2.cvtColor(arr, cv2.COLOR_RGB2BGR))


def wait_world_ready(st, gz, sess, parcels, timeout=120):
    """Robot ve tüm koliler dünyada, stations.launch zincirini bitirmiş."""
    names = [gz.MODEL] + [p[0] for p in parcels]

    def ready():
        with gz.lock:
            have = set(gz.poses)
        return gz.now() is not None and all(n in have for n in names) and sess.spawned()
    gz.wait_until(ready, timeout, f'dünya hazır değil ({names})')
    # Başlangıçta sunucu sensör/render başlatmasıyla meşgul olabilir: sim zamanı
    # gerçekten akana kadar bekle. Kayıt şimdiden başlar; S3 onu sürdürür, böylece
    # S3 penceresi spawn bitiminden başlar.
    gz.record(True)
    gz.wait_sim(1.0, timeout=timeout)


# =============================================================================
#  Poz ve robot yardımcıları (S6/S7)
# =============================================================================
# Kamera bağlantı çerçevesinden (x ileri, y sol, z yukarı) OpenCV optik
# çerçeveye (x sağ, y aşağı, z ileri): sütunlar link çerçevesinde.
R_LINK_OPT = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


class RobotGeom:
    """Robot ölçüleri forklift_params.yaml'dan (base_footprint çerçevesinde)."""

    def __init__(self, st):
        r = st.robot
        fc, fk, cam = r['fork_carriage'], r['fork'], r['cameras']['front_camera']
        self.carriage_front_x = fc['x'] + fc['size']['x'] / 2.0
        self.fork_top0 = fk['ground_z_bottom'] + fk['size']['z']
        self.fork_len = fk['size']['x']
        self.cam = cam
        self.fork_v = r['fork_lift_joint']['velocity']

    def front_cam_opt(self, bf):
        """Ön kamera optik çerçevesinin dünya pozu (bf: base_footprint dünya pozu)."""
        c = self.cam
        return bf * Pose([c['x'], c['y'], c['ground_z']], Ry(c['pitch']) @ R_LINK_OPT)


def robot_bf(gz):
    bf = gz.link(gz.MODEL, 'base_footprint')
    return bf if bf is not None else gz.model(gz.MODEL)


def place_robot(gz, p_xy, yaw, z=0.003, settle=0.5):
    gz.cmd(0.0)
    gz.set_pose(gz.MODEL, [p_xy[0], p_xy[1], z], Rz(yaw))
    gz.wait_sim(settle)


def fork_to(gz, rg, q, timeout=None):
    """Çatal hedefi; |q - hedef| < 1 mm olunca döner. (ulaştı mı, son q)"""
    gz.set_fork(q)
    t_lim = timeout if timeout is not None else 2.0 + 2.0 * 0.15 / rg.fork_v
    t0 = gz.now()
    while gz.now() - t0 < t_lim:
        f = gz.fork()
        if f is not None and abs(f - q) < 0.001:
            gz.wait_sim(0.3)
            return True, gz.fork()
        time.sleep(0.02)
    return False, gz.fork()
