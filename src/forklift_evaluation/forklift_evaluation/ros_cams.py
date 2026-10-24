"""
ros_cams.py — değerlendirme sürecinin ROS tarafı: kamera görüntüleri, camera_info, TF, odometri

Robotun algısıyla aynı yoldan okur (ROS topic'leri + TF), böylece GT izdüşümü
kamera modelini ve TF'i de doğrular. rclpy kendi iş parçacığında döner; gz-transport
abonelikleri (sim_tools.Gz) aynı süreçte çalışır.
"""

import math
import threading
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu
from tf2_ros import Buffer, TransformListener

from forklift_perception.core.attitude import AttitudeEstimator
from forklift_perception.core.camera_model import CameraModel

BASE_FRAME = 'base_footprint'


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def quat_to_R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def image_to_rgb(msg):
    """sensor_msgs/Image (rgb8 / bgr8) → RGB numpy dizisi (kopya)."""
    if msg.encoding not in ('rgb8', 'bgr8'):
        raise ValueError(f'desteklenmeyen kodlama: {msg.encoding}')
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
    a = a.reshape(msg.height, msg.width, 3)
    return (a[:, :, ::-1] if msg.encoding == 'bgr8' else a).copy()


class RosCams:
    """cams: {ad: (görüntü topic'i, camera_info topic'i)}."""

    def __init__(self, cams, node_name='forklift_evaluation'):
        if not rclpy.ok():
            rclpy.init()
        self.node = Node(node_name, parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.lock = threading.Lock()
        self.images, self.infos, self.counts = {}, {}, {}
        self.odom_msg = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        for name, (img_t, info_t) in cams.items():
            self.node.create_subscription(Image, img_t, self._img_cb(name), qos_profile_sensor_data)
            self.node.create_subscription(CameraInfo, info_t, self._info_cb(name), qos_profile_sensor_data)
        self.node.create_subscription(Odometry, '/odom', self._odom_cb, qos_profile_sensor_data)
        # IMU → algılamanın kestiricisinin aynısı (detectors.yaml attitude ile aynı sabitler verilir)
        self.att = None
        self.node.create_subscription(Imu, '/imu', self._imu_cb, qos_profile_sensor_data)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _img_cb(self, name):
        def cb(msg):
            with self.lock:
                self.images[name] = msg
                self.counts[name] = self.counts.get(name, 0) + 1
        return cb

    def _info_cb(self, name):
        def cb(msg):
            with self.lock:
                self.infos[name] = msg
        return cb

    def start_attitude(self, tau, level_samples, acc_tol=None, gyro_tol=None):
        """Kestiriciyi başlatır: robot şu an düz zeminde olmalı (seviye kalibrasyonu)."""
        with self.lock:
            self.att = AttitudeEstimator(tau, level_samples, acc_tol, gyro_tol)

    def attitude(self):
        with self.lock:
            return None if self.att is None else self.att.value()

    def _imu_cb(self, m):
        with self.lock:
            if self.att is not None:
                a, w = m.linear_acceleration, m.angular_velocity
                self.att.update(stamp_s(m.header.stamp), (a.x, a.y, a.z), (w.x, w.y, w.z))

    def _odom_cb(self, msg):
        with self.lock:
            self.odom_msg = msg

    # ------------------------------------------------------------- okumalar
    def image(self, name, newer_than=None, timeout=10.0):
        """(damga [s], RGB) — damgası newer_than'dan büyük ilk kare; yoksa None."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.lock:
                m = self.images.get(name)
            if m is not None and (newer_than is None or stamp_s(m.header.stamp) > newer_than):
                return stamp_s(m.header.stamp), image_to_rgb(m)
            time.sleep(0.02)
        return None

    def camera(self, name, timeout=10.0):
        """camera_info + TF (base_footprint ← optik çerçeve) → CameraModel."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.lock:
                info = self.infos.get(name)
            if info is not None:
                try:
                    tf = self.tf_buffer.lookup_transform(BASE_FRAME, info.header.frame_id, Time())
                except Exception:
                    tf = None
                if tf is not None:
                    q, p = tf.transform.rotation, tf.transform.translation
                    return CameraModel.from_camera_info(info, quat_to_R(q.x, q.y, q.z, q.w), [p.x, p.y, p.z])
            time.sleep(0.05)
        raise TimeoutError(f'{name}: camera_info veya TF gelmedi')

    def odom(self):
        """(damga, x, y, yaw) — /odom'un son mesajı; yoksa None."""
        with self.lock:
            m = self.odom_msg
        if m is None:
            return None
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return stamp_s(m.header.stamp), p.x, p.y, yaw

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
