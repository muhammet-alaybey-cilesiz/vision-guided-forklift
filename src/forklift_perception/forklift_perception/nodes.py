"""
nodes.py — algılama ROS düğümleri (ince katman: çekirdek forklift_perception.core)

front_perception: /front_camera → /perception/signs, /perception/parcels, /cv/detections
lane_perception : /lane_camera  → /perception/bay_targets, /perception/lanes, /perception/load
Yalnız gerçek robotta da olan girdiler (plan R1): görüntü, camera_info, TF, /joint_states.
Kamera içsel parametreleri calibration_dir/sim_<kamera>.yaml'dan (D-59), yoksa camera_info'dan.
Parametrelerin kodda varsayılanı yok (D-29): config/perception.yaml.
"""

import math
import os
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose2D
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from forklift_interfaces.msg import (BayTarget, BayTargetArray, Lane, LaneArray, LoadState, Parcel,
                                     ParcelArray, StationSign, StationSignArray)
from forklift_perception.core import calibration as cal
from forklift_perception.core import pipelines
from forklift_perception.core.camera_model import CameraModel

TR = {'red': 'kırmızı', 'green': 'yeşil', 'blue': 'mavi', 'neutral': 'kaynak', 'yellow': 'sarı'}


def _quat_to_R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _rgb(msg):
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
    a = a.reshape(msg.height, msg.width, 3)
    return a[:, :, ::-1].copy() if msg.encoding == 'bgr8' else a


class PerceptionNode(Node):
    def __init__(self, name):
        super().__init__(name)
        names = ['base_frame', 'catalog_file', 'robot_params_file', 'detectors_file', 'calibration_dir',
                 'publish_debug_images', 'image_topic', 'info_topic', 'camera_name'] + self.extra_params()
        for n in names:
            self.declare_parameter(n, rclpy.Parameter.Type.STRING if n not in ('publish_debug_images',)
                                   else rclpy.Parameter.Type.BOOL)
        p = {n: self.get_parameter(n).value for n in names}
        missing = [n for n in names if p[n] in (None, '') and n not in ('calibration_dir',)]
        if missing:
            raise RuntimeError(f'eksik parametre: {missing} (config/perception.yaml)')
        self.p = p
        self.cat, self.front, self.lane = pipelines.build(p['catalog_file'], p['robot_params_file'],
                                                          p['detectors_file'])
        self.cam = None
        self.info = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(CameraInfo, p['info_topic'], self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, p['image_topic'], self._on_image, qos_profile_sensor_data)
        self.debug_pub = self.create_publisher(Image, f'/perception/debug/{p["camera_name"].split("_")[0]}', 1) \
            if p['publish_debug_images'] else None
        self.stats = {'frames': 0, 'proc_s': 0.0}
        self.create_timer(10.0, self._report)

    def extra_params(self):
        return []

    def _on_info(self, msg):
        self.info = msg
        if self.cam is None:
            self._try_model()

    def _try_model(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.p['base_frame'], self.info.header.frame_id, Time())
        except Exception:
            return
        q, t = tf.transform.rotation, tf.transform.translation
        cam = CameraModel.from_camera_info(self.info, _quat_to_R(q.x, q.y, q.z, q.w), [t.x, t.y, t.z])
        path = os.path.join(self.p['calibration_dir'], f'sim_{self.p["camera_name"]}.yaml') \
            if self.p['calibration_dir'] else ''
        if path and os.path.exists(path):
            K, D, _, _ = cal.load_yaml(path)
            cam = cam.with_intrinsics(K, D)
            self.get_logger().info(f'kalibrasyon: {path} (cx {K[0, 2]:.2f}, cy {K[1, 2]:.2f})')
        else:
            self.get_logger().warn('kalibrasyon dosyası yok: camera_info kullanılıyor (D-59)')
        self.cam = cam

    def _on_image(self, msg):
        if self.cam is None:
            if self.info is not None:
                self._try_model()
            return
        t0 = time.perf_counter()
        self.process(_rgb(msg), msg.header)
        self.stats['frames'] += 1
        self.stats['proc_s'] += time.perf_counter() - t0

    def _report(self):
        n = self.stats['frames']
        if n:
            self.get_logger().info(f'{n / 10.0:.1f} kare/s, ortalama {1000 * self.stats["proc_s"] / n:.1f} ms')
        self.stats = {'frames': 0, 'proc_s': 0.0}

    def header(self, h):
        h2 = type(h)()
        h2.stamp = h.stamp
        h2.frame_id = self.p['base_frame']
        return h2


class FrontPerception(PerceptionNode):
    def __init__(self):
        super().__init__('front_perception')
        self.sign_pub = self.create_publisher(StationSignArray, '/perception/signs', 5)
        self.parcel_pub = self.create_publisher(ParcelArray, '/perception/parcels', 5)
        self.det_pub = self.create_publisher(Detection2DArray, '/cv/detections', qos_profile_sensor_data)

    def process(self, rgb, header):
        r = self.front.process(rgb, self.cam)
        hb = self.header(header)
        sm = StationSignArray(header=hb)
        dets = Detection2DArray()
        dets.header = header                     # görüntü çerçevesi (GUI kaplaması)
        for s in r['signs']:
            m = StationSign(color=s.color, confidence=float(s.confidence), pose_valid=True,
                            station_in_base=Pose2D(x=float(s.station_xy[0]), y=float(s.station_xy[1]),
                                                   theta=float(s.station_yaw)))
            m.corners_px = [Point(x=float(u), y=float(v)) for u, v in s.corners_px[:4]]
            sm.signs.append(m)
            dets.detections.append(_det(s.corners_px, f'tabela:{TR.get(s.color, s.color)}', s.confidence))
        pm = ParcelArray(header=hb)
        for p in r['parcels']:
            m = Parcel(confidence=float(p.confidence), seal_color=p.seal_color,
                       seal_confidence=float(p.seal_confidence), position_valid=True,
                       position_in_base=Point(x=float(p.position[0]), y=float(p.position[1]), z=float(p.position[2])))
            u0, v0, u1, v1 = p.bbox
            m.bbox.center.position.x, m.bbox.center.position.y = (u0 + u1) / 2.0, (v0 + v1) / 2.0
            m.bbox.size_x, m.bbox.size_y = float(u1 - u0), float(v1 - v0)
            pm.parcels.append(m)
            box = np.array([[u0, v0], [u1, v0], [u1, v1], [u0, v1]], float)
            dets.detections.append(_det(box, f'koli:{TR.get(p.seal_color, p.seal_color)}', p.confidence))
        self.sign_pub.publish(sm)
        self.parcel_pub.publish(pm)
        self.det_pub.publish(dets)
        if self.debug_pub is not None:
            img = rgb.copy()
            for d in r['signs']:
                cv2.polylines(img, [np.round(d.corners_px).astype(np.int32)], True, (0, 255, 0), 2)
            for p in r['parcels']:
                cv2.rectangle(img, p.bbox[:2], p.bbox[2:], (255, 160, 0), 2)
            self.debug_pub.publish(_img_msg(img, header))


class LanePerception(PerceptionNode):
    def __init__(self):
        super().__init__('lane_perception')
        self.q = 0.0
        self.create_subscription(JointState, self.p['joint_states_topic'], self._on_js, qos_profile_sensor_data)
        self.target_pub = self.create_publisher(BayTargetArray, '/perception/bay_targets', 5)
        self.lane_pub = self.create_publisher(LaneArray, '/perception/lanes', 5)
        self.load_pub = self.create_publisher(LoadState, '/perception/load', 5)

    def extra_params(self):
        return ['joint_states_topic', 'fork_joint']

    def _on_js(self, msg):
        for n, pos in zip(msg.name, msg.position):
            if n.endswith(self.p['fork_joint']):
                self.q = float(pos)

    def process(self, rgb, header):
        r = self.lane.process(rgb, self.cam, self.q)
        hb = self.header(header)
        bt = BayTargetArray(header=hb)
        for p in r['pairs']:
            m = BayTarget(source=BayTarget.SOURCE_TARGET_PAIR, confidence=float(p.confidence), yaw_valid=True,
                          distance=float(p.distance), baseline_measured=float(p.baseline),
                          entry_in_base=Pose2D(x=float(p.entry_xy[0]), y=float(p.entry_xy[1]), theta=float(p.theta)))
            m.targets_px = [Point(x=float(p.left.uv[0]), y=float(p.left.uv[1])),
                            Point(x=float(p.right.uv[0]), y=float(p.right.uv[1]))]
            bt.bays.append(m)
        for lb in r['labels']:
            m = BayTarget(source=BayTarget.SOURCE_BAY_LABEL, confidence=float(lb.confidence),
                          yaw_valid=bool(lb.yaw_valid), distance=float(lb.distance), baseline_measured=0.0,
                          entry_in_base=Pose2D(x=float(lb.entry_xy[0]), y=float(lb.entry_xy[1]), theta=float(lb.theta)))
            pt = Point(x=float(lb.label.uv[0]), y=float(lb.label.uv[1]))
            m.targets_px = [pt, pt]
            bt.bays.append(m)
        la = LaneArray(header=hb)
        for s in r['lanes']:
            m = Lane(color=s.color, kind=int(s.kind), confidence=float(s.confidence),
                     lateral_offset=float(s.lateral_offset), heading=float(s.heading),
                     end_visible=bool(s.end_visible), end_distance=float(s.far),
                     stop_bar_visible=not math.isnan(s.stop_bar_distance),
                     stop_bar_distance=float(s.stop_bar_distance) if not math.isnan(s.stop_bar_distance) else 0.0)
            m.points = [Point(x=float(x), y=float(y), z=0.0) for x, y in s.points]
            la.lanes.append(m)
        self.target_pub.publish(bt)
        self.lane_pub.publish(la)
        self.load_pub.publish(LoadState(header=hb, loaded=bool(r['loaded']), kraft_fraction=float(r['kraft_fraction'])))
        if self.debug_pub is not None:
            img = rgb.copy()
            for p in r['pairs']:
                for t in (p.left, p.right):
                    cv2.circle(img, tuple(int(v) for v in t.uv), 6, (255, 0, 255), 2)
            for lb in r['labels']:
                cv2.circle(img, tuple(int(v) for v in lb.label.uv), 6, (255, 255, 0), 2)
            self.debug_pub.publish(_img_msg(img, header))


def _det(corners, label, score):
    c = np.asarray(corners, float)
    u0, v0 = c.min(axis=0)
    u1, v1 = c.max(axis=0)
    d = Detection2D()
    d.bbox.center.position.x, d.bbox.center.position.y = float((u0 + u1) / 2), float((v0 + v1) / 2)
    d.bbox.size_x, d.bbox.size_y = float(u1 - u0), float(v1 - v0)
    h = ObjectHypothesisWithPose()
    h.hypothesis.class_id, h.hypothesis.score = label, float(score)
    d.results.append(h)
    d.id = label
    return d


def _img_msg(rgb, header):
    m = Image(header=header, height=rgb.shape[0], width=rgb.shape[1], encoding='rgb8', step=rgb.shape[1] * 3)
    m.data = rgb.tobytes()
    return m


def _run(cls):
    rclpy.init()
    node = cls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def front_main():
    _run(FrontPerception)


def lane_main():
    _run(LanePerception)
