#!/usr/bin/env python3
# Copyright 2026 muhammet61
# SPDX-License-Identifier: Apache-2.0
"""forklift_gui entegrasyon testleri (spesifikasyon §6.2, I1-I12).

Betik kendi oturumlarını açar: ekransız simülasyon, komut zinciri
(twist_mux + cmd_vel_gate) ve gerçek GUI süreci. GUI'ye X11 üzerinden gerçek
tuş ve fare olayı gönderilir (xdotool XTEST); sonuçlar ROS topic'lerinden,
GUI'nin /rosout olaylarından ve Qt günlük kategorilerinden ölçülür (D-39).

Kullanım:
  ros2 run forklift_gui integration_test.py                 # tümü
  ros2 run forklift_gui integration_test.py --tests I2,I5   # seçili

Test sırasında GUI penceresi açılır; fare ve klavye kullanılır. Her tuştan
önce etkin pencerenin GUI olduğu doğrulanır ve X tuş otomatik tekrarı test
boyunca kapatılır (çıkışta eski hâline döner).
"""

import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from PIL import Image
from rcl_interfaces.msg import Log
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2D
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose
import yaml

GUI_NODE = 'forklift_gui'
LOGICAL_WIDTH = 1280        # ApplicationWindow varsayılan genişliği (Main.qml)
ESTOP_CENTER_FROM_RIGHT = 111  # buton merkezi, sağ kenardan [mantıksal px] (Theme)
ESTOP_CENTER_Y = 28         # TopBar yüksekliğinin yarısı [mantıksal px]
STOPPED_SPEED = 0.01        # [m/s] bu hızın altı "durdu"
# Varsayılan düzende (1280×800, Main.qml SplitView tercihleri) alt sekmeler
# ve veri seti panelinin ilk düğmesi [mantıksal px]; tıklama GUI olayıyla
# doğrulanır.
DATASET_TAB = (1129, 597)
LOG_TAB = (842, 597)
RECORD_BUTTON = (470, 671)
DETECTION_COLOR = (255, 59, 107)  # Theme.detection
ALL_TESTS = ['I1', 'I2', 'I3', 'I4', 'I5', 'I6', 'I7', 'I8', 'I9', 'I10', 'I11', 'I12']


def run(cmd, timeout=60, check=False):
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, check=check)


def wait_for(predicate, timeout, period=0.02):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(period)
    return predicate()


def cpu_ticks(pid):
    # /proc/<pid>/stat: ')' sonrası alanlar 3'ten başlar; utime 14, stime 15.
    fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
    return int(fields[11]) + int(fields[12])


def rss_mb(pid):
    for line in Path(f'/proc/{pid}/status').read_text().splitlines():
        if line.startswith('VmRSS:'):
            return int(line.split()[1]) / 1024.0
    return float('nan')


def magenta_boxes(png_path):
    """Return bounding boxes of the two detection-colour clusters, left to right."""
    img = np.asarray(Image.open(png_path).convert('RGB')).astype(int)
    mask = np.abs(img - np.array(DETECTION_COLOR)).sum(axis=2) < 40
    ys, xs = np.nonzero(mask)
    if xs.size < 20:
        return []
    order = np.argsort(xs)
    split = int(np.argmax(np.diff(xs[order])))
    boxes = []
    for idx in (order[:split + 1], order[split + 1:]):
        boxes.append((int(xs[idx].min()), int(ys[idx].min()),
                      int(xs[idx].max()), int(ys[idx].max())))
    return boxes


class Proc:
    """Kendi süreç grubunda çalışan alt süreç; çıktısı günlük dosyasına."""

    def __init__(self, name, cmd, log_dir, env=None):
        self.name = name
        self.log_path = log_dir / f'{name}.log'
        self._log = open(self.log_path, 'w')
        self.popen = subprocess.Popen(
            cmd, stdout=self._log, stderr=subprocess.STDOUT, start_new_session=True, env=env)

    @property
    def pid(self):
        return self.popen.pid

    def alive(self):
        return self.popen.poll() is None

    def send_signal(self, sig):
        try:
            os.killpg(self.popen.pid, sig)
        except ProcessLookupError:
            pass

    def wait(self, timeout):
        try:
            return self.popen.wait(timeout)
        except subprocess.TimeoutExpired:
            return None

    def stop(self, sig=signal.SIGINT, timeout=20.0):
        if self.alive():
            self.send_signal(sig)
            if self.wait(timeout) is None:
                self.send_signal(signal.SIGKILL)
                self.wait(5.0)
        self._log.flush()
        return self.popen.returncode

    def output(self):
        self._log.flush()
        return self.log_path.read_text(errors='replace')


class Probe(Node):
    """Topic'leri duvar saatiyle (time.monotonic) zaman damgalayan gözlemci."""

    def __init__(self, topics):
        super().__init__('gui_integration_probe')
        self.lock = threading.Lock()
        self.odom = deque(maxlen=50000)     # (t, x, y, yaw, v, w)
        self.fork = deque(maxlen=50000)     # (t, yükseklik)
        self.cmd_vel = deque(maxlen=50000)  # (t, v, w)  kapı çıkışı
        self.teleop = deque(maxlen=50000)   # (t, v, w)  GUI çıkışı
        self.estop = deque(maxlen=50000)    # (t, data)
        self.events = deque(maxlen=20000)   # (t, düğüm, seviye, metin)
        self.clock_times = deque(maxlen=5000)
        self.clock_samples = deque(maxlen=200000)  # (t, sim zamanı)
        self.fork_joint = topics['fork_joint']
        self.detection_pub = self.create_publisher(Detection2DArray, topics['detections'], 10)
        self.detection_boxes = []
        self.create_timer(0.1, self._publish_detections)
        self.create_subscription(Odometry, topics['odom'], self._on_odom, qos_profile_sensor_data)
        self.create_subscription(
            JointState, topics['joint_states'], self._on_joints, qos_profile_sensor_data)
        self.create_subscription(Twist, topics['cmd_vel_out'], self._on_cmd_vel, 100)
        self.create_subscription(Twist, topics['teleop'], self._on_teleop, 100)
        self.create_subscription(Bool, topics['e_stop'], self._on_estop, 100)
        self.create_subscription(Log, '/rosout', self._on_rosout, 1000)
        self.create_subscription(Clock, topics['clock'], self._on_clock, qos_profile_sensor_data)

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self.lock:
            self.odom.append((time.monotonic(), msg.pose.pose.position.x,
                              msg.pose.pose.position.y, yaw, msg.twist.twist.linear.x,
                              msg.twist.twist.angular.z))

    def _on_joints(self, msg):
        if self.fork_joint in msg.name:
            i = msg.name.index(self.fork_joint)
            with self.lock:
                self.fork.append((time.monotonic(), msg.position[i]))

    def _on_cmd_vel(self, msg):
        with self.lock:
            self.cmd_vel.append((time.monotonic(), msg.linear.x, msg.angular.z))

    def _on_teleop(self, msg):
        with self.lock:
            self.teleop.append((time.monotonic(), msg.linear.x, msg.angular.z))

    def _on_estop(self, msg):
        with self.lock:
            self.estop.append((time.monotonic(), msg.data))

    def _on_rosout(self, msg):
        with self.lock:
            self.events.append((time.monotonic(), msg.name, msg.level, msg.msg))

    def _on_clock(self, msg):
        now = time.monotonic()
        with self.lock:
            self.clock_times.append(now)
            self.clock_samples.append((now, msg.clock.sec + msg.clock.nanosec * 1e-9))

    def _publish_detections(self):
        boxes = self.detection_boxes
        if not boxes:
            return
        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'front_camera_optical_frame'
        for cx, cy, w, h, label, score in boxes:
            d = Detection2D()
            d.bbox.center.position.x = float(cx)
            d.bbox.center.position.y = float(cy)
            d.bbox.size_x = float(w)
            d.bbox.size_y = float(h)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = label
            hyp.hypothesis.score = float(score)
            d.results.append(hyp)
            msg.detections.append(d)
        self.detection_pub.publish(msg)

    # --- sorgular ---------------------------------------------------------
    def last_odom(self):
        with self.lock:
            return self.odom[-1] if self.odom else None

    def last_fork(self):
        with self.lock:
            return self.fork[-1][1] if self.fork else None

    def last_estop(self):
        with self.lock:
            return self.estop[-1][1] if self.estop else None

    def snapshot(self, name):
        with self.lock:
            return list(getattr(self, name))

    def gui_event(self, pattern, since):
        rx = re.compile(pattern)
        for t, node, _level, text in self.snapshot('events'):
            if t >= since and node == GUI_NODE and rx.search(text):
                return t, text
        return None

    def odom_flowing(self, within=1.0):
        last = self.last_odom()
        return last is not None and time.monotonic() - last[0] < within

    def stopped_since(self, since, duration=0.3):
        """Return True if |v| stayed below STOPPED_SPEED for 'duration' after 'since'."""
        samples = [s for s in self.snapshot('odom') if s[0] >= since]
        if not samples or samples[-1][0] - samples[0][0] < duration:
            return False
        tail = [s for s in samples if s[0] >= samples[-1][0] - duration]
        return all(abs(s[4]) < STOPPED_SPEED for s in tail)


class GuiWindow:
    """GUI penceresine XTEST ile tuş ve fare olayı; önce pencere etkinleştirilir."""

    def __init__(self, pid):
        self.pid = pid
        self.win = None

    def find(self, timeout=30.0):
        def found():
            out = run(['xdotool', 'search', '--onlyvisible', '--pid', str(self.pid)])
            ids = out.stdout.split()
            if ids:
                self.win = ids[0]
            return bool(ids)
        if not wait_for(found, timeout, period=0.2):
            raise RuntimeError('GUI penceresi bulunamadı')
        return self.win

    def geometry(self):
        out = run(['xwininfo', '-id', self.win]).stdout
        val = {}
        for key, label in (('x', 'Absolute upper-left X'), ('y', 'Absolute upper-left Y'),
                           ('w', 'Width'), ('h', 'Height')):
            m = re.search(rf'{label}:\s+(-?\d+)', out)
            val[key] = int(m.group(1))
        return val

    def activate(self):
        for _ in range(5):
            run(['xdotool', 'windowactivate', '--sync', self.win], timeout=10)
            active = run(['xdotool', 'getactivewindow']).stdout.strip()
            if active == self.win:
                return
            time.sleep(0.2)
        raise RuntimeError('GUI penceresi etkinleştirilemedi; tuş gönderilmedi')

    def key(self, *keys):
        self.activate()
        run(['xdotool', 'key', '--delay', '60', *keys])

    def keydown(self, key):
        self.activate()
        run(['xdotool', 'keydown', key])

    def keyup(self, key):
        # Bırakma, odak nerede olursa olsun gönderilir (takılı tuş kalmasın).
        run(['xdotool', 'keyup', key])

    def raw(self, *args):
        # Etkinleştirme önceden yapılmış olmalı (zaman ölçümü yalnız tuşu kapsasın).
        run(['xdotool', *args])

    def click(self, lx, ly):
        self.activate()
        g = self.geometry()
        dpr = g['w'] / LOGICAL_WIDTH
        run(['xdotool', 'mousemove', str(g['x'] + int(lx * dpr)), str(g['y'] + int(ly * dpr)),
             'click', '1'])

    def hold_estop(self, seconds):
        """Hold the e-stop button with the mouse (release needs a deliberate hold)."""
        self.activate()
        g = self.geometry()
        dpr = g['w'] / LOGICAL_WIDTH
        x = g['x'] + g['w'] - int(ESTOP_CENTER_FROM_RIGHT * dpr)
        y = g['y'] + int(ESTOP_CENTER_Y * dpr)
        run(['xdotool', 'mousemove', str(x), str(y)])
        time.sleep(0.2)
        run(['xdotool', 'mousedown', '1'])
        time.sleep(seconds)
        run(['xdotool', 'mouseup', '1'])


class Runner:
    def __init__(self, args):
        self.args = args
        self.prefix = Path(get_package_prefix('forklift_gui'))
        self.ws = self.prefix.parents[1]
        self.docs = Path(args.docs) if args.docs else self.ws / 'docs'
        self.log_dir = self.docs / 'logs' / 'gui'
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.images = self.docs / 'images'
        self.images.mkdir(parents=True, exist_ok=True)
        self.results = {}
        self.procs = []
        self.gui_exe = str(self.prefix / 'lib' / 'forklift_gui' / 'forklift_gui')
        gui_share = Path(get_package_share_directory('forklift_gui'))
        safety_share = Path(get_package_share_directory('forklift_safety'))
        params = yaml.safe_load((gui_share / 'config' / 'gui_params.yaml').read_text())
        self.params = params[GUI_NODE]['ros__parameters']
        gate = yaml.safe_load((safety_share / 'config' / 'cmd_vel_gate.yaml').read_text())
        gate = gate['cmd_vel_gate']['ros__parameters']
        t = self.params['topics']
        self.topics = {
            'odom': t['odom'], 'joint_states': t['joint_states'], 'clock': t['clock'],
            'teleop': t['cmd_vel'], 'e_stop': t['e_stop'], 'detections': t['detections'],
            'cmd_vel_out': gate['output_topic'], 'fork_joint': self.params['fork']['joint_name'],
        }
        self.gate = gate
        self.autorepeat_was_on = None

    # --- altyapı ---------------------------------------------------------
    def record(self, test_id, name, status, measured, criterion, detail=''):
        self.results[test_id] = {
            'name': name, 'status': status, 'measured': measured,
            'criterion': criterion, 'detail': detail}
        print(f'[{test_id}] {status}: {name} | {measured} | kriter: {criterion}', flush=True)

    def start(self, name, cmd, env=None):
        proc = Proc(name, cmd, self.log_dir, env=env)
        self.procs.append(proc)
        return proc

    def start_sim(self):
        # control:=false → komut zinciri start_control ile ayrı süreç (I7 önce zinciri,
        # sonra sim'i başlatır; D-42 öncesi senaryolar aynen korunur).
        return self.start('sim', ['ros2', 'launch', 'forklift_gazebo', 'sim.launch.py',
                                  'gui:=false', 'headless:=true', 'control:=false'])

    def start_control(self):
        return self.start('control', ['ros2', 'launch', 'forklift_safety', 'control.launch.py'])

    def start_gui(self, name='gui', overrides=(), extra_args=(), env_extra=None):
        cmd = [self.gui_exe, *extra_args]
        if overrides:
            cmd += ['--ros-args'] + [x for o in overrides for x in ('-p', o)]
        env = dict(os.environ)
        env.update(env_extra or {})
        return self.start(name, cmd, env=env)

    def stop_all(self):
        for proc in reversed(self.procs):
            proc.stop()
        self.procs.clear()

    def set_autorepeat(self, on):
        run(['xset', 'r', 'on' if on else 'off'])

    def wait_gui_ready(self, probe, gui, since, timeout=30.0):
        win = GuiWindow(gui.pid)
        win.find(timeout)
        # GUI heartbeat'i (e-stop true) akmaya başladığında düğüm hazırdır.
        if not wait_for(lambda: probe.last_estop() is not None and
                        probe.snapshot('estop')[-1][0] > since, timeout):
            raise RuntimeError('GUI /e_stop yayınlamıyor')
        return win

    def release_estop(self, probe, win):
        hold = float(self.params['safety']['release_hold'])
        win.hold_estop(hold + 0.4)
        if not wait_for(lambda: probe.last_estop() is False, 2.0):
            raise RuntimeError('E-stop serbest bırakılamadı (buton konumu?)')

    def wait_sim(self, probe, timeout=90.0):
        if not wait_for(probe.odom_flowing, timeout, period=0.2):
            raise RuntimeError('Simülasyon başlamadı (odometri yok)')

    # --- testler -----------------------------------------------------------
    def test_i1(self):
        pkgs = ['forklift_gui', 'forklift_safety', 'forklift_bringup']
        build = run(['colcon', 'build', '--symlink-install', '--packages-select', *pkgs,
                     '--cmake-clean-first', '--event-handlers', 'console_direct+',
                     '--cmake-args', '-DCMAKE_BUILD_TYPE=RelWithDebInfo'],
                    timeout=1800)
        out = build.stdout + build.stderr
        (self.log_dir / 'I1_build.log').write_text(out)
        warnings = [ln for ln in out.splitlines() if re.search(r'warning:', ln, re.I)]
        test = run(['colcon', 'test', '--packages-select', *pkgs], timeout=1800)
        (self.log_dir / 'I1_test.log').write_text(test.stdout + test.stderr)
        totals = {}
        for pkg in pkgs:
            res = run(['colcon', 'test-result', '--test-result-base',
                       str(self.ws / 'build' / pkg), '--all'], timeout=120)
            (self.log_dir / f'I1_test_result_{pkg}.log').write_text(res.stdout)
            m = re.search(r'(\d+) tests, (\d+) errors, (\d+) failures, (\d+) skipped',
                          res.stdout)
            totals[pkg] = tuple(int(x) for x in m.groups()) if m else None
        ok = (build.returncode == 0 and not warnings and
              all(v is not None and v[1] == 0 and v[2] == 0 for v in totals.values()))
        measured = (f'build çıkış {build.returncode}, uyarı {len(warnings)}; testler: ' +
                    ', '.join(f'{p} {v[0]} test/{v[1]} hata/{v[2]} başarısız/{v[3]} atlandı'
                              if v else f'{p} sonuç yok' for p, v in totals.items()))
        self.record('I1', 'Build + lint + qmllint', 'GEÇTİ' if ok else 'KALDI', measured,
                    'colcon build uyarısız; clang-format, cpplint, qmllint ve tüm testler temiz')

    def test_i3(self, probe, win):
        target = self.params['fork']['preset_heights'][2]
        tol = self.params['fork']['reached_tolerance']
        timeout = self.params['fork']['reach_timeout']
        t0 = time.monotonic()
        win.key('3')
        reached = wait_for(lambda: (probe.last_fork() is not None and
                                    abs(probe.last_fork() - target) <= tol), timeout + 2.0)
        t_reach = time.monotonic() - t0
        event = None
        wait_for(lambda: probe.gui_event(r'Çatal hedefe ulaştı', t0) is not None, 3.0)
        event = probe.gui_event(r'Çatal hedefe ulaştı', t0)
        time.sleep(1.0)
        final = probe.last_fork()
        ok = reached and t_reach <= timeout and event is not None and abs(final - target) <= tol
        self.record(
            'I3', 'Çatal preset "Taşıma"', 'GEÇTİ' if ok else 'KALDI',
            f'{t_reach:.2f} s içinde ±{tol * 1000:.0f} mm; son yükseklik {final * 1000:.1f} mm; '
            f'GUI olayı: {event[1] if event else "YOK"}',
            f'{timeout:.0f} s içinde {target:.3f} ± {tol:.3f} m ve GUI "ulaşıldı"')

    def test_i2(self, probe, win):
        hold = 3.0
        start = probe.last_odom()
        win.activate()
        t0 = time.monotonic()
        win.raw('keydown', 'w')
        time.sleep(hold)
        win.raw('keyup', 'w')
        t_up = time.monotonic()
        wait_for(lambda: probe.stopped_since(t_up, 0.5), 6.0)
        end = probe.last_odom()
        dist = math.hypot(end[1] - start[1], end[2] - start[2])
        cmds = [s for s in probe.snapshot('teleop') if t0 <= s[0] <= t_up + 0.3]
        peak = max((s[1] for s in cmds), default=0.0)
        moving = [s for s in cmds if s[1] > 0.0]
        span = (moving[-1][0] - moving[0][0]) if len(moving) > 1 else 0.0
        ok = 0.54 <= dist <= 0.66
        self.record(
            'I2', 'Sürüş: 0.2 m/s, 3 s (W basılı)', 'GEÇTİ' if ok else 'KALDI',
            f'odom {dist:.3f} m (komut tepe {peak:.3f} m/s; tuş basılı {t_up - t0:.2f} s, '
            f'sıfır olmayan komut {span:.2f} s)',
            '0.6 m ± %10 (0.54–0.66 m)',
            'teleop.max_linear 0.2 ve scale_default 1.0 launch geçersiz kılmasıyla (D-39)')

    def test_i4(self, probe, win):
        win.keydown('s')
        time.sleep(1.5)
        before = probe.last_odom()
        win.activate()
        t_key = time.monotonic()
        win.raw('key', 'space')
        win.keyup('s')
        time.sleep(1.5)
        est = [s for s in probe.snapshot('estop') if s[0] >= t_key and s[1]]
        zeros = [s for s in probe.snapshot('cmd_vel')
                 if s[0] >= t_key and s[1] == 0.0 and s[2] == 0.0]
        stopped = probe.stopped_since(t_key, 0.3)
        if not est or not zeros:
            self.record('I4', 'E-stop gecikmesi', 'KALDI', 'e-stop ya da sıfır komut görülmedi',
                        '≤ 100 ms')
            return
        # GUI önce sıfır teleop, sonra /e_stop=true yayınlar (D-31); sıfır kilitten
        # önce gelebilir, o durumda gecikme 0 sayılır.
        lat_estop = max(0.0, zeros[0][0] - est[0][0]) * 1000.0
        order = 'sıfır önce' if zeros[0][0] <= est[0][0] else 'kilit önce'
        lat_key = (zeros[0][0] - t_key) * 1000.0
        ok = lat_estop <= 100.0 and stopped and abs(before[4]) > 0.1
        self.record(
            'I4', 'Hareket halindeyken e-stop → /cmd_vel sıfır', 'GEÇTİ' if ok else 'KALDI',
            f'/e_stop=true → /cmd_vel sıfır {lat_estop:.1f} ms ({order}); Space tuşundan '
            f'{lat_key:.1f} ms; öncesi v={before[4]:.3f} m/s; robot durdu: {stopped}',
            '≤ 100 ms')

    def test_i5(self, probe, win, gui):
        win.keydown('s')
        time.sleep(1.5)
        before = probe.last_odom()
        t_kill = time.monotonic()
        os.kill(gui.pid, signal.SIGKILL)
        win.keyup('s')
        stopped = wait_for(lambda: (probe.last_odom()[0] > t_kill and
                                    abs(probe.last_odom()[4]) < STOPPED_SPEED), 5.0)
        stop_odom = next((s for s in probe.snapshot('odom')
                          if s[0] > t_kill and abs(s[4]) < STOPPED_SPEED), None)
        zero = next((s for s in probe.snapshot('cmd_vel')
                     if s[0] > t_kill and s[1] == 0.0 and s[2] == 0.0), None)
        if not stopped or stop_odom is None:
            self.record('I5', 'GUI çökmesi (kill -9)', 'KALDI', 'robot 5 s içinde durmadı',
                        '≤ 0.6 s')
            return
        t_stop = stop_odom[0] - t_kill
        t_zero = zero[0] - t_kill if zero else float('nan')
        ok = t_stop <= 0.6 and abs(before[4]) > 0.1
        self.record(
            'I5', 'GUI çökmesi (kill -9) sırasında sürüş', 'GEÇTİ' if ok else 'KALDI',
            f'durma {t_stop:.3f} s (kapı sıfırı {t_zero:.3f} s); öncesi v={before[4]:.3f} m/s',
            '≤ 0.6 s')

    def test_i7_i8(self, probe):
        # I7: önce GUI (+ komut zinciri), sonra sim.
        control = self.start_control()
        t_gui = time.monotonic()
        gui = self.start_gui('gui_i7')
        win = self.wait_gui_ready(probe, gui, t_gui)
        time.sleep(3.0)
        waiting = probe.gui_event(r'ROS bağlantısı: bekleniyor', t_gui)
        premature = probe.gui_event(r'Simülasyon saati \(/clock\): normal', t_gui)
        t_sim = time.monotonic()
        sim = self.start_sim()
        ok_event = wait_for(lambda: (
            probe.gui_event(r'Simülasyon saati \(/clock\): normal', t_sim) is not None and
            probe.gui_event(r'Genel sağlık: normal', t_sim) is not None and
            probe.gui_event(r'ROS bağlantısı: normal', t_sim) is not None), 120.0, period=0.2)
        clocks = [t for t in probe.snapshot('clock_times') if t >= t_sim]
        healthy = probe.gui_event(r'Genel sağlık: normal', t_sim)
        detail = ''
        if healthy and clocks:
            detail = (f'sim başlatma → genel sağlık normal {healthy[0] - t_sim:.1f} s; '
                      f'ilk /clock → normal {healthy[0] - clocks[0]:.1f} s')
        ok = waiting is not None and premature is None and ok_event
        self.record('I7', 'Sim yok → var: otomatik bağlanma', 'GEÇTİ' if ok else 'KALDI',
                    f'başta "bekleniyor": {waiting is not None}; {detail}',
                    'GUI sim olmadan açılır, bekler; sim gelince sağlık yeşile döner')

        # I8a: pencereyi kapatma (pencere yöneticisi, Alt+F4).
        results = []
        t0 = time.monotonic()
        win.key('alt+F4')
        rc = gui.wait(5.0)
        results.append(('pencere kapatma', rc, time.monotonic() - t0,
                        'düzenli kapanış tamam' in gui.output()))
        # I8b/c: SIGINT (ros2 launch Ctrl-C) ve SIGTERM.
        for sig, label in ((signal.SIGINT, 'SIGINT'), (signal.SIGTERM, 'SIGTERM')):
            t_start = time.monotonic()
            g = self.start_gui(f'gui_i8_{label.lower()}')
            self.wait_gui_ready(probe, g, t_start)
            time.sleep(2.0)
            t0 = time.monotonic()
            g.send_signal(sig)
            rc = g.wait(5.0)
            results.append((label, rc, time.monotonic() - t0,
                            'düzenli kapanış tamam' in g.output()))
        ok = all(rc == 0 and dt <= 2.0 and clean for _, rc, dt, clean in results)
        self.record('I8', 'Kapanış', 'GEÇTİ' if ok else 'KALDI',
                    '; '.join(f'{lbl}: çıkış {rc}, {dt:.2f} s, düzenli={clean}'
                              for lbl, rc, dt, clean in results),
                    'crash yok, süreç 2 s içinde biter')
        sim.stop()
        control.stop()

    def session_drive(self, probe, selected):
        sim = self.start_sim()
        control = self.start_control()
        self.wait_sim(probe)
        t_gui = time.monotonic()
        gui = self.start_gui('gui_drive', overrides=['teleop.max_linear:=0.2',
                                                     'teleop.scale_default:=1.0'])
        win = self.wait_gui_ready(probe, gui, t_gui)
        wait_for(lambda: probe.gui_event(r'Genel sağlık: normal', t_gui) is not None, 20.0)
        self.release_estop(probe, win)
        if 'I3' in selected:
            self.test_i3(probe, win)
        if 'I2' in selected:
            self.test_i2(probe, win)
        if 'I4' in selected:
            self.test_i4(probe, win)
            self.release_estop(probe, win)
        if 'I5' in selected:
            self.test_i5(probe, win, gui)
        gui.stop()
        control.stop()
        sim.stop()

    # --- Faz 4: kameralar, veri seti, kaplama, performans ---------------------
    def measure_window(self, probe, gui, seconds):
        """Return RTF, GUI CPU %, RSS and camera FPS samples over a window."""
        out0 = len(gui.output()) if gui else 0
        ticks0 = cpu_ticks(gui.pid) if gui else 0
        t0 = time.monotonic()
        time.sleep(seconds)
        t1 = time.monotonic()
        clocks = [c for c in probe.snapshot('clock_samples') if c[0] >= t0]
        rtf = ((clocks[-1][1] - clocks[0][1]) / (clocks[-1][0] - clocks[0][0])
               if len(clocks) > 1 else float('nan'))
        result = {'rtf': rtf}
        if gui:
            clk = os.sysconf('SC_CLK_TCK')
            result['cpu'] = (cpu_ticks(gui.pid) - ticks0) / clk / (t1 - t0) * 100.0
            result['rss'] = rss_mb(gui.pid)
            fps = {}
            for cam, value in re.findall(r'forklift\.gui\.camera: (\w+) fps=([\d.]+)',
                                         gui.output()[out0:]):
                fps.setdefault(cam, []).append(float(value))
            result['fps'] = fps
        return result

    def test_i10(self, probe, win, dataset_root):
        win.click(*DATASET_TAB)
        time.sleep(0.5)
        t0 = time.monotonic()
        win.click(*RECORD_BUTTON)
        if not wait_for(lambda: probe.gui_event(r'Sürekli kayıt başladı', t0) is not None, 3.0):
            self.record('I10', 'Sürekli veri seti kaydı', 'KALDI',
                        'kayıt başlamadı (düğme konumu?)', '10 s × 2 Hz ≈ 20 çift')
            return
        start = probe.gui_event(r'Sürekli kayıt başladı', t0)[0]
        time.sleep(10.0 - (time.monotonic() - start))
        t1 = time.monotonic()
        win.click(*RECORD_BUTTON)
        wait_for(lambda: probe.gui_event(r'Sürekli kayıt durdu', t1) is not None, 3.0)
        stop = probe.gui_event(r'Sürekli kayıt durdu', t1)
        time.sleep(2.0)  # bekleyen PNG yazımları
        sessions = [d for d in Path(dataset_root).iterdir() if d.is_dir()]
        if len(sessions) != 1:
            self.record('I10', 'Sürekli veri seti kaydı', 'KALDI',
                        f'{len(sessions)} oturum dizini', '1 oturum, ≈ 20 çift')
            return
        session = sessions[0]
        cams = [c['id'] if isinstance(c, dict) else c for c in self.params['cameras']['ids']]
        counts = {c: len(list((session / c).glob('*.png'))) for c in cams}
        lines = [ln for ln in (session / 'metadata.jsonl').read_text().splitlines() if ln]
        meta = [json.loads(ln) for ln in lines]
        complete = all(all((session / m['images'][c]['file']).exists() for c in cams)
                       for m in meta)
        shutil.copy(session / 'metadata.jsonl', self.log_dir / 'I10_metadata.jsonl')
        duration = (stop[0] - start) if stop else float('nan')
        pairs = min(counts.values())
        expected = duration * float(self.params['dataset']['continuous_rate'])
        ok = (abs(pairs - 20) <= 2 and len(meta) == pairs and complete and
              len(set(counts.values())) == 1)
        self.record(
            'I10', 'Sürekli veri seti kaydı (10 s, 2 Hz)', 'GEÇTİ' if ok else 'KALDI',
            f'{pairs} kare çifti ({counts}), {len(meta)} metadata satırı, '
            f'kayıt süresi {duration:.2f} s (beklenen ≈ {expected:.1f} + ilk kare)',
            '≈ 20 kare çifti ve 20 metadata satırı')

    def test_i9(self, probe):
        # İki kutu: yatayda ayrık, etiketleri kutudan dar (ölçüm için).
        boxes = [(130.0, 100.0, 160.0, 96.0, 'seal_blue', 0.81),
                 (420.0, 300.0, 192.0, 120.0, 'seal_red', 0.93)]
        probe.detection_boxes = [tuple(b) for b in boxes]
        png = self.images / 'gui_detection_overlay.png'
        t0 = time.monotonic()
        gui = self.start_gui('gui_i9', extra_args=['--screenshot', str(png),
                                                   '--screenshot-delay', '9'])
        self.wait_gui_ready(probe, gui, t0)
        rc = gui.wait(40.0)
        probe.detection_boxes = []
        found = magenta_boxes(png) if rc == 0 and png.exists() else []
        if len(found) != 2:
            self.record('I9', 'Tespit kaplaması', 'KALDI',
                        f'ekran görüntüsü çıkış {rc}, bulunan küme {len(found)}',
                        'kutular doğru konumda; ekran görüntüsü')
            return
        (bx0, _, bx1, by1), (ax0, _, ax1, ay1) = found
        b, a = boxes
        sx = ((ax0 + ax1) / 2 - (bx0 + bx1) / 2) / (a[0] - b[0])
        sy = (ay1 - by1) / ((a[1] + a[3] / 2) - (b[1] + b[3] / 2))
        wa = (ax1 - ax0 + 1) / (a[2] * sx)
        wb = (bx1 - bx0 + 1) / (b[2] * sx)
        ox = (ax0 + ax1) / 2 - a[0] * sx
        oy = ay1 - (a[1] + a[3] / 2) * sy
        ok = abs(sy / sx - 1) < 0.03 and abs(wa - 1) < 0.05 and abs(wb - 1) < 0.05
        self.record(
            'I9', 'Tespit kaplaması (sahte /cv/detections)', 'GEÇTİ' if ok else 'KALDI',
            f'ölçek x {sx:.3f} / y {sy:.3f} px/px (oran {sy / sx:.3f}); genişlik oranı '
            f'{wa:.3f}, {wb:.3f}; içerik başlangıcı ({ox:.0f}, {oy:.0f}) px; '
            f'{png.relative_to(self.ws)}',
            'kutular görüntüyle aynı doğrusal eşlemede (en-boy korunur), ekran görüntüsü')

    def screenshot_estop(self, probe):
        png = self.images / 'gui_main_estop.png'
        t0 = time.monotonic()
        gui = self.start_gui('gui_shot_estop', extra_args=['--screenshot', str(png),
                                                           '--screenshot-delay', '8'])
        self.wait_gui_ready(probe, gui, t0)
        return gui.wait(40.0) == 0 and png.exists()

    def screenshot_driving(self, probe, dataset_root):
        png = self.images / 'gui_main_sim.png'
        t0 = time.monotonic()
        gui = self.start_gui('gui_shot_drive',
                             overrides=[f'dataset.root_dir:={dataset_root}'],
                             extra_args=['--screenshot', str(png), '--screenshot-delay', '20'])
        win = self.wait_gui_ready(probe, gui, t0)
        self.release_estop(probe, win)
        win.key('3')
        win.click(*DATASET_TAB)
        win.click(*RECORD_BUTTON)
        win.keydown('w')
        win.keydown('a')
        # Ekran görüntüsü hareket sırasında alınır; süreç kendisi çıkar.
        rc = gui.wait(40.0)
        win.keyup('a')
        win.keyup('w')
        return rc == 0 and png.exists()

    def session_camera(self, probe, selected):
        sim = self.start_sim()
        control = self.start_control()
        self.wait_sim(probe)
        time.sleep(5.0)
        base = self.measure_window(probe, None, 20.0) if 'I11' in selected else None
        dataset_root = Path(tempfile.mkdtemp(prefix='forklift_dataset_test_'))
        t_gui = time.monotonic()
        gui = self.start_gui(
            'gui_camera', overrides=[f'dataset.root_dir:={dataset_root}'],
            env_extra={'QT_LOGGING_RULES': 'forklift.gui.camera.debug=true'})
        win = self.wait_gui_ready(probe, gui, t_gui)
        time.sleep(5.0)
        if 'I6' in selected or 'I11' in selected:
            m = self.measure_window(probe, gui, 20.0)
            fps = m['fps']
            if 'I6' in selected:
                summary = {c: (sum(v) / len(v), min(v)) for c, v in fps.items() if v}
                ok = len(summary) == len(self.params['cameras']['ids']) and all(
                    mean >= 25.0 for mean, _ in summary.values())
                self.record('I6', 'İki kamera görüntüsü, GUI FPS', 'GEÇTİ' if ok else 'KALDI',
                            '; '.join(f'{c}: ort {mean:.1f}, min {mn:.1f} FPS'
                                      for c, (mean, mn) in summary.items()),
                            'iki görüntü de gösteriliyor, FPS ≥ 25')
            if 'I11' in selected:
                drop = (1.0 - m['rtf'] / base['rtf']) * 100.0
                ok = drop < 10.0
                self.record(
                    'I11', 'Performans (iki kamera açık)', 'GEÇTİ' if ok else 'KALDI',
                    f'GUI CPU {m["cpu"]:.1f} % (tek çekirdek), RSS {m["rss"]:.0f} MB; '
                    f'RTF GUI yok {base["rtf"]:.3f}, GUI var {m["rtf"]:.3f} '
                    f'(düşüş {drop:.1f} %)',
                    'CPU raporlanır; RTF düşüşü < %10')
        if 'I10' in selected:
            self.test_i10(probe, win, dataset_root)
        gui.stop()
        if 'I9' in selected:
            self.test_i9(probe)
        if self.args.screenshots:
            shots = {'e-stop': self.screenshot_estop(probe),
                     'sürüş': self.screenshot_driving(probe, dataset_root)}
            print(f'Ekran görüntüleri: {shots}', flush=True)
        shutil.rmtree(dataset_root, ignore_errors=True)
        control.stop()
        sim.stop()

    # --- Faz 5: mock mod ve rapor görüntüleri ---------------------------------
    def test_i12(self, probe):
        png = self.images / 'gui_mock.png'
        t0 = time.monotonic()
        gui = self.start_gui('gui_i12', extra_args=['--mock', '--screenshot', str(png),
                                                    '--screenshot-delay', '8'],
                             env_extra={'QT_LOGGING_RULES': 'forklift.gui.camera.debug=true'})
        GuiWindow(gui.pid).find()
        time.sleep(4.0)
        # ROS olmadan: mock modda rclcpp::init yok; düğüm grafikte görünmez,
        # /e_stop yayınlanmaz.
        in_graph = GUI_NODE in probe.get_node_names()
        estops = [e for e in probe.snapshot('estop') if e[0] >= t0]
        rc = gui.wait(30.0)
        fps = {}
        for cam, value in re.findall(r'forklift\.gui\.camera: (\w+) fps=([\d.]+)', gui.output()):
            fps.setdefault(cam, []).append(float(value))
        means = {c: sum(v[2:]) / max(1, len(v[2:])) for c, v in fps.items()}
        ok = (rc == 0 and png.exists() and not in_graph and not estops and
              len(means) == len(self.params['cameras']['ids']) and
              all(m >= 25.0 for m in means.values()))
        self.record(
            'I12', 'Mock mod (ROS olmadan)', 'GEÇTİ' if ok else 'KALDI',
            f'çıkış {rc}; ROS grafiğinde düğüm: {in_graph}; /e_stop mesajı: {len(estops)}; '
            f'sahte kamera FPS: ' + ', '.join(f'{c} {m:.1f}' for c, m in means.items()) +
            f'; {png.relative_to(self.ws)}',
            '--mock ile ROS olmadan açılır, paneller sahte veriyle dolar; ekran görüntüsü')

    def screenshots_mock(self):
        shots = {}
        for name, action in (('gui_mock_help.png', 'help'), ('gui_mock_log.png', 'log')):
            png = self.images / name
            gui = self.start_gui(f'gui_shot_{action}', extra_args=[
                '--mock', '--screenshot', str(png), '--screenshot-delay', '7'])
            win = GuiWindow(gui.pid)
            win.find()
            time.sleep(1.5)
            if action == 'help':
                win.key('F1')
            else:
                win.click(*LOG_TAB)
            shots[name] = gui.wait(30.0) == 0 and png.exists()
        print(f'Mock ekran görüntüleri: {shots}', flush=True)

    def write_results(self):
        out = self.docs / 'gui_test_results.json'
        previous = json.loads(out.read_text()) if out.exists() else {}
        previous.update(self.results)
        out.write_text(json.dumps(previous, ensure_ascii=False, indent=2))
        lines = ['# forklift_gui entegrasyon testleri', '',
                 f'Son koşu: {time.strftime("%Y-%m-%d %H:%M:%S")} '
                 f'(`integration_test.py --tests {",".join(sorted(self.results))}`)', '',
                 '| # | Test | Ölçülen | Kriter | Sonuç |', '|---|---|---|---|---|']
        for key in ALL_TESTS:
            if key in previous:
                r = previous[key]
                lines.append(f'| {key} | {r["name"]} | {r["measured"]} | {r["criterion"]} | '
                             f'**{r["status"]}** |')
        (self.docs / 'gui_test_results.md').write_text('\n'.join(lines) + '\n')

    def main(self):
        selected = ALL_TESTS
        if self.args.tests:
            selected = [t.strip() for t in self.args.tests.split(',')]
        if run(['pgrep', '-f', 'gz sim']).returncode == 0:
            print('Başka bir gz sim çalışıyor; aynı GZ_PARTITION paylaşılmasın diye durduruldu.')
            return 2
        self.autorepeat_was_on = 'auto repeat:  on' in run(['xset', 'q']).stdout
        self.set_autorepeat(False)
        rclpy.init()
        probe = Probe(self.topics)
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(probe)
        spin = threading.Thread(target=executor.spin, daemon=True)
        spin.start()
        try:
            if 'I1' in selected:
                self.test_i1()
            if any(t in selected for t in ('I2', 'I3', 'I4', 'I5')):
                self.session_drive(probe, selected)
            if any(t in selected for t in ('I6', 'I9', 'I10', 'I11')) or self.args.screenshots:
                self.session_camera(probe, selected)
            if any(t in selected for t in ('I7', 'I8')):
                self.test_i7_i8(probe)
            if 'I12' in selected:
                self.test_i12(probe)
            if self.args.screenshots:
                self.screenshots_mock()
        except Exception as exc:  # her hata kayda geçsin, süreçler kapansın
            print(f'HATA: {exc}', flush=True)
            self.results.setdefault('HATA', {'name': 'betik', 'status': 'HATA',
                                             'measured': str(exc), 'criterion': '-'})
        finally:
            for key in ('w', 'a', 's', 'd'):
                run(['xdotool', 'keyup', key])
            self.stop_all()
            if self.autorepeat_was_on:
                self.set_autorepeat(True)
            executor.shutdown()
            probe.destroy_node()
            rclpy.shutdown()
        self.write_results()
        failed = [k for k, r in self.results.items() if r['status'] != 'GEÇTİ']
        return 1 if failed else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--tests', default='', help='virgülle: I1,I2,... (boş = tümü)')
    parser.add_argument('--docs', default='', help='sonuç dizini (varsayılan: <ws>/docs)')
    parser.add_argument('--screenshots', action='store_true',
                        help='rapor için sim ekran görüntülerini de al (e-stop, sürüş)')
    return Runner(parser.parse_args()).main()


if __name__ == '__main__':
    sys.exit(main())
