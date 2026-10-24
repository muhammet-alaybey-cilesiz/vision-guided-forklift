"""
node.py — mission_manager: mod (Manuel / Otonom), görev durum makinesi, komut yayını

Girdiler yalnız gerçek robotta da olanlar (plan R1): algılama mesajları, /odom, TF, /joint_states,
/e_stop ve operatör komutları. Ölçümler görüntünün çekildiği andaki TF (odom ← base) ile
odometri çerçevesine taşınır.

Mod kuralları (plan §5, D-57):
  MANUAL : /cmd_vel/auto ve çatal komutu yok; görev çalışıyorsa DURAKLADI'ya geçer.
  AUTO   : /cmd_vel/auto sürekli rate_hz (dururken sıfır). Görev başlatma: e-stop bırakılmış olmalı.
  E-stop yükselen kenarı ya da teleop devralma: DURAKLADI, çatal ölçülen yükseklikte tutulur (D-32).
  Devam: yalnız açık komutla (/autonomy/resume, K7); yanaşmanın ortasından ise gözlemden yeniden.

Görev türü 'task' parametresinden (pick | drop | transit | full; transit için 'target' rengi), başlatma
anında okunur. Operatör GUI'sinde her zaman full; testler parametreyi değiştirir.
"""

import math
import os

import rclpy
from rclpy.executors import ExternalShutdownException
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from forklift_autonomy.core.mission import TASKS, Mission
from forklift_autonomy.core.se2 import SE2
from forklift_autonomy.core.tracker import BayTracker, LaneTracker, SignTracker
from forklift_interfaces.msg import (AutonomyState, BayTarget, BayTargetArray, Lane, LaneArray, LoadState, ParcelArray,
                                     StationSignArray)
from forklift_interfaces.srv import SetMode


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _t(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        for n in ('config_file', 'robot_params_file'):
            self.declare_parameter(n, rclpy.Parameter.Type.STRING)
        with open(self.get_parameter('config_file').value) as f:
            self.cfg = yaml.safe_load(f)
        with open(self.get_parameter('robot_params_file').value) as f:
            rp = yaml.safe_load(f)
        c, tp = self.cfg, self.cfg['topics']
        self.declare_parameter('transit_method', c['transit_method'])
        self.declare_parameter('task', c['mission']['default_task'])
        self.declare_parameter('target', '')
        fc, fk, cam = rp['fork_carriage'], rp['fork'], rp['cameras']['front_camera']
        carriage_front_x = fc['x'] + fc['size']['x'] / 2.0
        self.robot = {'stop_distance': carriage_front_x + c['mission']['entry_gap'],
                      'fork_min': rp['fork_lift_joint']['lower'], 'fork_max': rp['fork_lift_joint']['upper']}
        self.self_parcel_x = fk['x_min'] + fk['size']['x'] + c['self_parcel_margin']
        tc = c['tracker']
        self.front_view = ((cam['x'], cam['y']), cam['horizontal_fov'] / 2.0 - math.radians(tc['parcel_view_margin_deg']),
                           tc['parcel_view_range'])
        self.tracker = BayTracker(tc)
        self.signs = SignTracker(tc)
        self.lane = LaneTracker(tc)
        self.mission = Mission(c['mission'], self.robot, self.tracker, self.signs, self.lane,
                               self.get_parameter('transit_method').value)
        self.last_cmd = (0.0, 0.0)
        self.mode = AutonomyState.MODE_MANUAL
        self.paused_from = None
        self.pose = None
        self.q = None
        self.load = None
        self.estop, self.estop_t = True, None
        self.teleop_t = None
        self.fork_sent = None
        self.log_seen = 0
        self.yaw_rate = 0.0

        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.create_subscription(Odometry, tp['odom'], self._on_odom, qos_profile_sensor_data)
        self.create_subscription(JointState, tp['joint_states'], self._on_js, qos_profile_sensor_data)
        self.create_subscription(BayTargetArray, tp['bay_targets'], self._on_bays, 5)
        self.create_subscription(ParcelArray, tp['parcels'], self._on_parcels, 5)
        self.create_subscription(LoadState, tp['load'], self._on_load, 5)
        self.create_subscription(StationSignArray, tp['signs'], self._on_signs, 5)
        self.create_subscription(LaneArray, tp['lanes'], self._on_lanes, 5)
        self.create_subscription(Bool, tp['e_stop'], self._on_estop, 10)
        self.create_subscription(Twist, tp['teleop'], self._on_teleop, 10)
        rel = QoSProfile(depth=1)
        self.cmd_pub = self.create_publisher(Twist, tp['cmd_vel'], rel)
        self.fork_pub = self.create_publisher(Float64, tp['fork'], rel)
        self.state_pub = self.create_publisher(AutonomyState, tp['state'], 5)
        self.create_service(SetMode, '/autonomy/set_mode', self._srv_mode)
        for name, fn in (('start', self._srv_start), ('pause', self._srv_pause), ('resume', self._srv_resume),
                         ('cancel', self._srv_cancel)):
            self.create_service(Trigger, f'/autonomy/{name}', fn)
        self.create_timer(1.0 / c['rate_hz'], self._tick)
        self.create_timer(1.0 / c['state_rate_hz'], self._publish_state)
        self.get_logger().info(f'görev yöneticisi hazır (mod: Manuel, transit: {self.mission.method})')

    # ------------------------------------------------------------------ girdiler
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, m):
        p = m.pose.pose
        self.pose = SE2(p.position.x, p.position.y, _yaw(p.orientation))
        self.yaw_rate = abs(m.twist.twist.angular.z)

    def _on_js(self, m):
        for n, v in zip(m.name, m.position):
            if n.endswith(self.cfg['fork_joint']):
                self.q = float(v)

    def _odom_at(self, stamp):
        """Görüntü anındaki robot pozu (odom ← base); bulunamazsa en son."""
        fr = self.cfg['frames']
        for t in (Time.from_msg(stamp), Time()):
            try:
                tf = self.tf.lookup_transform(fr['odom'], fr['base'], t)
                tr, q = tf.transform.translation, tf.transform.rotation
                return SE2(tr.x, tr.y, _yaw(q))
            except Exception:
                continue
        return None

    def _on_bays(self, m):
        # hızlı dönerken ölçüm alınmaz: görüntü anı ile TF arasındaki küçük zaman farkı bile yanal
        # konumu büyük ölçüde kaydırır (0.5 m'de 0.1 s × 0.6 rad/s ≈ 30 mm)
        if not self.mission.active or self.yaw_rate > self.cfg['max_meas_yaw_rate']:
            return
        T = self._odom_at(m.header.stamp)
        if T is None:
            return
        entries = []
        for b in m.bays:
            xy = T.apply((b.entry_in_base.x, b.entry_in_base.y))
            src = 'pair' if b.source == BayTarget.SOURCE_TARGET_PAIR else 'label'
            anchor = T.apply((b.anchor_in_base.x, b.anchor_in_base.y))
            entries.append((xy, b.entry_in_base.theta + T.yaw, bool(b.yaw_valid), src, anchor))
        self.tracker.add_entries(_t(m.header.stamp), entries)

    def _on_parcels(self, m):
        if not self.mission.active or self.yaw_rate > self.cfg['max_meas_yaw_rate']:
            return
        T = self._odom_at(m.header.stamp)
        if T is None:
            return
        # çatalın üstündeki (bizim taşıdığımız) koli bölmeye ait değildir
        ps = [(T.apply((p.position_in_base.x, p.position_in_base.y)), p.seal_color)
              for p in m.parcels if p.position_valid and p.position_in_base.x > self.self_parcel_x]
        cam, half, (rmin, rmax) = self.front_view
        self.tracker.add_parcels(_t(m.header.stamp), ps, view=(T.apply(cam), T.yaw, half, rmin, rmax))

    def _on_signs(self, m):
        if not self.mission.active or self.yaw_rate > self.cfg['max_meas_yaw_rate']:
            return
        T = self._odom_at(m.header.stamp)
        if T is None:
            return
        rmax = self.cfg['tracker']['sign_max_range']
        for s in m.signs:
            # algının menzili dışındaki tabela yanlış pozitiftir (factory: 4.9 m'de "nötr" bir nesne)
            if s.pose_valid and math.hypot(s.station_in_base.x, s.station_in_base.y) <= rmax:
                p = s.station_in_base
                self.signs.add(_t(m.header.stamp), s.color, T.apply((p.x, p.y)), p.theta + T.yaw, math.hypot(p.x, p.y))

    def _on_lanes(self, m):
        color = self.lane.color
        if not self.mission.active or not color or self.yaw_rate > self.cfg['max_meas_yaw_rate']:
            return
        T, cands = None, []
        for ln in m.lanes:
            if ln.color != color or ln.kind != Lane.KIND_FLOOR or len(ln.points) < 2:
                continue
            T = T or self._odom_at(m.header.stamp)
            if T is None:
                return
            pts = [T.apply((p.x, p.y)) for p in ln.points]
            bar = None
            if ln.stop_bar_visible:
                # durma çizgisi şeridin uzak ucunda: uç noktadan şerit yönünde stop_bar_distance'a uzat
                e = ln.points[-1]
                h = ln.heading
                k = (ln.stop_bar_distance - e.x) / max(math.cos(h), 0.5)
                bar = T.apply((e.x + k * math.cos(h), e.y + k * math.sin(h)))
            cands.append((pts, [math.hypot(p.x, p.y) for p in ln.points], bar))
        if cands:
            self.lane.add_candidates(_t(m.header.stamp), cands, self.pose.yaw if self.pose else 0.0)

    def _on_load(self, m):
        self.load = bool(m.loaded)

    def _on_estop(self, m):
        rising = bool(m.data) and not self.estop
        self.estop, self.estop_t = bool(m.data), self.now()
        if rising:
            self._pause('e-stop')

    def _on_teleop(self, m):
        if abs(m.linear.x) > 1e-6 or abs(m.angular.z) > 1e-6:
            self.teleop_t = self.now()
            if self.mode == AutonomyState.MODE_AUTO and self.mission.active and self.paused_from is None:
                self._pause('operatör devraldı (teleop)')

    def estop_engaged(self):
        return self.estop or self.estop_t is None or self.now() - self.estop_t > self.cfg['e_stop_stale_s']

    # ------------------------------------------------------------------ servisler
    def _srv_mode(self, req, res):
        if req.mode not in (AutonomyState.MODE_MANUAL, AutonomyState.MODE_AUTO):
            res.success, res.message = False, 'bilinmeyen mod'
            return res
        if req.mode == AutonomyState.MODE_MANUAL and self.mission.active and self.paused_from is None:
            self._pause('Manuel moda geçildi')
        self.mode = req.mode
        res.success, res.message = True, 'Otonom' if req.mode == AutonomyState.MODE_AUTO else 'Manuel'
        self.get_logger().info(f'mod: {res.message}')
        return res

    def _srv_start(self, req, res):
        if self.mode != AutonomyState.MODE_AUTO:
            res.success, res.message = False, 'önce Otonom moda geçin'
        elif self.estop_engaged():
            res.success, res.message = False, 'e-stop basılı'
        elif self.mission.active:
            res.success, res.message = False, 'görev zaten çalışıyor'
        elif self.pose is None:
            res.success, res.message = False, 'odometri yok'
        else:
            task, target = self.get_parameter('task').value, self.get_parameter('target').value
            if task not in TASKS or (task == 'transit' and not target):
                res.success, res.message = False, f'geçersiz görev: {task} {target}'
                return res
            self.paused_from = None
            self.mission.start(self.now(), task, target)
            self.log_seen = 0
            res.success, res.message = True, 'görev başladı'
            self.get_logger().info(f'görev başladı ({task} {target})'.strip())
        return res

    def _srv_pause(self, req, res):
        ok = self.mission.active and self.paused_from is None
        if ok:
            self._pause('operatör duraklattı')
        res.success, res.message = ok, 'duraklatıldı' if ok else 'çalışan görev yok'
        return res

    def _srv_resume(self, req, res):
        if self.paused_from is None:
            res.success, res.message = False, 'duraklatılmış görev yok'
        elif self.mode != AutonomyState.MODE_AUTO:
            res.success, res.message = False, 'önce Otonom moda geçin'
        elif self.estop_engaged():
            res.success, res.message = False, 'e-stop basılı'
        else:
            st = self.mission.resume_state(self.paused_from)
            if st != self.paused_from:
                self.mission._go(st, self.now(), f'devam: {self.paused_from} yarıda kaldı, yeniden gözlem')
            self.mission.t_state = self.now()          # durumun zaman aşımı devamdan başlar
            self.paused_from = None
            res.success, res.message = True, 'devam ediliyor'
        return res

    def _srv_cancel(self, req, res):
        was = self.mission.active
        self.mission.reset()
        self.paused_from = None
        self._hold_fork()
        res.success, res.message = True, 'görev iptal edildi' if was else 'görev yoktu'
        return res

    def _pause(self, why):
        if not self.mission.active or self.paused_from is not None:
            return
        self.paused_from = self.mission.state
        self.mission.log.append((self.now(), 'PAUSED', why))
        self._hold_fork()
        self.get_logger().warn(f'duraklatıldı: {why}')

    def _hold_fork(self):
        if self.q is not None:
            self.fork_pub.publish(Float64(data=float(self.q)))
            self.fork_sent = None                       # devamda hedef yeniden gönderilir

    # ------------------------------------------------------------------ döngü
    def _tick(self):
        if self.mode != AutonomyState.MODE_AUTO:
            return
        v = w = 0.0
        if self.mission.active and self.paused_from is None and self.pose is not None:
            if self.estop_engaged():
                self._pause('e-stop')
            else:
                v, w, fork = self.mission.step(self.now(), self.pose, self.q, self.load)
                self.load = None
                if fork is not None and fork != self.fork_sent:
                    fork = min(max(fork, self.robot['fork_min']), self.robot['fork_max'])
                    self.fork_pub.publish(Float64(data=float(fork)))
                    self.fork_sent = fork
                if self.mission.state in ('DONE', 'FAULT'):
                    v = w = 0.0
                    self.get_logger().info(f'görev bitti: {self.mission.detail}')   # yalnız geçişte (aktif değil artık)
        # ivme sınırı (yüklü dönüş, plan §10.1); durma komutu da sınırlanır, gate e-stop'u ayrıca keser
        dt = 1.0 / self.cfg['rate_hz']
        lv, lw = self.last_cmd
        al = self.cfg['accel_limits']
        v = lv + max(-al['v'] * dt, min(al['v'] * dt, v - lv))
        w = lw + max(-al['w'] * dt, min(al['w'] * dt, w - lw))
        self.last_cmd = (v, w)
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.cmd_pub.publish(msg)

    def _publish_state(self):
        # görev günlüğü (geçişler ve sebepleri) ROS loguna: sonradan inceleme için
        for t, state, why in self.mission.log[self.log_seen:]:
            b = self.mission.bay
            where = '' if b is None else f' [bölme giriş {b.entry[0]:.3f},{b.entry[1]:.3f} θ {math.degrees(b.theta):.2f}°]'
            self.get_logger().info(f'{state}: {why}{where}')
        self.log_seen = len(self.mission.log)
        m = AutonomyState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.mode = self.mode
        if self.paused_from is not None:
            m.state, m.detail = 'PAUSED', f'Duraklatıldı ({self.mission.log[-1][2]})'
        else:
            m.state, m.detail = self.mission.state, self.mission.detail
        b = self.mission.bay
        m.target_color = self.mission.target_color
        m.bay_lateral = float('nan') if b is None or self.pose is None else float(
            -math.sin(b.theta) * (b.entry[0] - self.pose.x) + math.cos(b.theta) * (b.entry[1] - self.pose.y))
        m.fault = self.mission.fault
        m.parcels_done = self.mission.parcels_done
        m.parcels_total = self.mission.parcels_total
        self.state_pub.publish(m)


def main():
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    os.environ.setdefault('PYTHONUNBUFFERED', '1')
    main()
