"""
perf_probe.py — C8 (canlı performans) ve C0'ın çalışma anı kısmı

Simülasyon (station_sim.launch.py, ekransız) ve algılama düğümleri (perception.launch.py)
birlikte çalışırken ölçer:
  * RTF (gz /stats), algılama çıkış hızları (/perception/*, /cv/detections), düğüm CPU'su (/proc)
  * algılama düğümlerinin abone olduğu topic'ler: izinli küme (plan R1) dışında abonelik yok mu
  ros2 run forklift_evaluation perf_probe --world station_test [--seconds 30]
Çıktı: <out>/c8_<world>.json
"""

import argparse
import json
import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from forklift_evaluation.world_gt import EvalSetup, simt
from forklift_interfaces.msg import BayTargetArray, LaneArray, LoadState, ParcelArray, StationSignArray
from vision_msgs.msg import Detection2DArray

OUTPUTS = {'/perception/signs': StationSignArray, '/perception/parcels': ParcelArray,
           '/cv/detections': Detection2DArray, '/perception/bay_targets': BayTargetArray,
           '/perception/lanes': LaneArray, '/perception/load': LoadState}
ALLOWED_SUBS = {'/front_camera/image_raw', '/front_camera/camera_info', '/lane_camera/image_raw',
                '/lane_camera/camera_info', '/joint_states', '/tf', '/tf_static', '/clock', '/parameter_events'}


def proc_cpu(pids):
    """Toplam CPU zamanı [s] (utime + stime) /proc'tan."""
    tot = 0.0
    hz = os.sysconf('SC_CLK_TCK')
    for pid in pids:
        try:
            with open(f'/proc/{pid}/stat') as f:
                parts = f.read().rsplit(')', 1)[1].split()
            tot += (int(parts[11]) + int(parts[12])) / hz
        except (FileNotFoundError, IndexError):
            pass
    return tot


def find_pids(name):
    out = subprocess.run(['pgrep', '-f', name], capture_output=True, text=True).stdout.split()
    return [int(p) for p in out]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', default='station_test')
    ap.add_argument('--seconds', type=float, default=30.0)
    ap.add_argument('--out', default=os.path.expanduser('~/cv_forklift_ws/docs/logs/cv/f2'))
    ap.add_argument('--screenshot', default='', help='operatör GUI ekran görüntüsü (PNG yolu)')
    a = ap.parse_args()
    es = EvalSetup(a.world, a.out)
    res = {'world': a.world, 'seconds': a.seconds}
    with simt.SimSession(es, f'c8_{a.world}', control=False) as sess:
        gz = None
        perc = subprocess.Popen(['ros2', 'launch', 'forklift_perception', 'perception.launch.py'],
                                stdout=open(os.path.join(es.logs, f'perception_{a.world}.log'), 'w'),
                                stderr=subprocess.STDOUT, start_new_session=True)
        try:
            gz = simt.Gz(es)
            simt.wait_world_ready(es, gz, sess, [])
            gz.record(False)
            simt.place_robot(gz, (0.3, 0.0), 0.0, settle=0.5)      # kaynak istasyona bakan poz
            rclpy.init()
            node = Node('perf_probe', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
            counts = {t: 0 for t in OUTPUTS}
            last = {}

            def cb(topic):
                def f(msg):
                    counts[topic] += 1
                    last[topic] = msg
                return f
            for t, typ in OUTPUTS.items():
                node.create_subscription(typ, t, cb(t), qos_profile_sensor_data)
            # ısınma: algılama düğümleri kamera modelini kurana kadar
            t0 = time.time()
            while time.time() - t0 < 30 and not all(counts[t] > 0 for t in OUTPUTS):
                rclpy.spin_once(node, timeout_sec=0.1)
            pids = find_pids('forklift_perception/front_perception') + find_pids('forklift_perception/lane_perception')
            for t in counts:
                counts[t] = 0
            cpu0, sim0, wall0 = proc_cpu(pids), gz.now(), time.time()
            while time.time() - wall0 < a.seconds:
                rclpy.spin_once(node, timeout_sec=0.05)
            wall = time.time() - wall0
            res['rtf'] = (gz.now() - sim0) / wall
            res['rates_hz'] = {t: c / wall for t, c in counts.items()}
            res['cpu_cores'] = (proc_cpu(pids) - cpu0) / wall
            res['pids'] = pids
            names = ('/perception/signs', '/perception/parcels', '/perception/bay_targets')
            sig, par, bay = (last.get(t) for t in names)
            res['sample'] = {'signs': [m.color for m in sig.signs] if sig else None,
                             'parcels': [m.seal_color for m in par.parcels] if par else None,
                             'bays': len(bay.bays) if bay else None}
            subs = {}
            for nn in ('front_perception', 'lane_perception'):
                info = node.get_subscriber_names_and_types_by_node(nn, '/')
                subs[nn] = sorted(t for t, _ in info)
            res['subscriptions'] = subs
            extra = sorted({t for v in subs.values() for t in v} - ALLOWED_SUBS)
            res['c0_runtime'] = {'allowed': sorted(ALLOWED_SUBS), 'outside_allowed': extra,
                                 'status': 'GEÇTİ' if not extra and subs['front_perception'] else 'KALDI'}
            if a.screenshot:
                # operatör GUI'si: ön kamera + /cv/detections kaplaması (GUI'nin kendi --screenshot seçeneği;
                # masaüstüne girdi gönderilmez)
                for _ in range(20):
                    rclpy.spin_once(node, timeout_sec=0.05)
                g = subprocess.run(['ros2', 'run', 'forklift_gui', 'forklift_gui', '--screenshot', a.screenshot,
                                    '--screenshot-delay', '10'], capture_output=True, text=True, timeout=120)
                res['screenshot'] = {'path': a.screenshot, 'exists': os.path.exists(a.screenshot),
                                     'returncode': g.returncode}
            low = min(res['rates_hz'].values())
            res['c8_status'] = 'GEÇTİ' if res['rtf'] >= 0.9 and low >= 15.0 else 'KALDI'
            node.destroy_node()
            rclpy.shutdown()
        finally:
            try:
                os.killpg(os.getpgid(perc.pid), signal.SIGINT)
                perc.wait(timeout=15)
            except Exception:
                os.killpg(os.getpgid(perc.pid), signal.SIGKILL)
            if gz is not None:
                gz.close()
    with open(os.path.join(a.out, f'c8_{a.world}.json'), 'w') as f:
        json.dump(res, f, indent=1, ensure_ascii=False)
    print(json.dumps(res, indent=1, ensure_ascii=False))


if __name__ == '__main__':
    main()
