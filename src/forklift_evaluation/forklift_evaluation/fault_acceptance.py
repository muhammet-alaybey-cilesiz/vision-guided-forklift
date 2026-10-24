"""
fault_acceptance.py — A5: hata enjeksiyonu (plan §7.2, §8)

Her senaryo bir görevi başlatır, belirli bir anda hatayı enjekte eder ve görev yöneticisinin tepkisini
GT ile ölçer. Robot yalnız kendi algısıyla sürer; enjeksiyon ve ölçüm test tarafındadır (gz ışınlama,
görsel örtü, operatör komutları).

  ros2 run forklift_evaluation fault_acceptance --world station_test [--only E1,E4]

  E1  transitte e-stop          ≤ 0.6 s'de durma; DURAKLADI; e-stop bırakılınca kendiliğinden devam yok;
                                [Devam] ile görev biter
  E2  girişte teleop            DURAKLADI; devamda yeniden gözlem; alma başarılı
  E3  hizalamada Manuel mod     DURAKLADI, robot durur; Otonom + Devam ile bırakma başarılı
  E4  hedefi 3 s örtme          RECOVER, sonra alma başarılı
  E5  hedefi kalıcı örtme       FAULT (kurtarma tükendi), robot duruyor
  E6  taşırken koliyi kaldırma  FAULT (yük kaybı), robot duruyor
  E7  kaldırmadan önce koliyi bölmeden alma   yeniden deneme, sonraki koli alınır
  E8  yerleri değişmiş istasyonlar (kırmızı ↔ mavi)   tam görev; koliler renge göre doğru istasyonda
"""

import argparse
import json
import math
import os
import signal
import subprocess
import threading
import time

import numpy as np
import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist

from forklift_evaluation.mission_acceptance import A4_LIMIT_S, FAIL, NOT_RUN, PASS, Operator, Rig, _a4_parcels
from forklift_evaluation.world_gt import EvalSetup, pose_in_front, simt, sl

ALL = ['E1', 'E2', 'E3', 'E4', 'E5', 'E6', 'E7', 'E8']
STOP_V, STOP_W = 0.005, 0.01              # durmuş sayılma: doğrusal [m/s], açısal [rad/s]
ESTOP_STOP_S = 0.6                        # plan §8: e-stop'ta durma süresi


class Motion:
    """GT robot hızı (gz pozlarından, sim saatiyle)."""

    def __init__(self, gz):
        self.gz, self.prev = gz, None
        self.v = self.w = 0.0

    def update(self):
        bf, t = simt.robot_bf(self.gz), self.gz.now()
        if bf is None:
            return
        yaw = math.atan2(bf.R[1, 0], bf.R[0, 0])
        if self.prev is not None and t > self.prev[0] + 1e-3:
            dt = t - self.prev[0]
            self.v = float(np.linalg.norm(bf.p[:2] - self.prev[1])) / dt
            self.w = abs(math.atan2(math.sin(yaw - self.prev[2]), math.cos(yaw - self.prev[2]))) / dt
        self.prev = (t, bf.p[:2].copy(), yaw)

    @property
    def stopped(self):
        return self.v < STOP_V and self.w < STOP_W


def occluder_sdf(name, width, height):
    """Yalnız görsel (çarpışmasız, statik) gri pano: kameraların hedeflere bakışını keser."""
    return f"""<sdf version='1.9'><model name='{name}'><static>true</static><link name='l'>
<visual name='v'><pose>0 0 {height / 2} 0 0 0</pose><geometry><box><size>0.01 {width} {height}</size></box></geometry>
<material><ambient>0.35 0.35 0.35 1</ambient><diffuse>0.35 0.35 0.35 1</diffuse></material></visual>
</link></model></sdf>"""


class Faults:
    def __init__(self, rig, log):
        self.rig, self.gz, self.op, self.log = rig, rig.gz, rig.op, log
        self.teleop_pub = self.op.create_publisher(Twist, '/cmd_vel/teleop', 10)
        with open(os.path.join(get_package_share_directory('forklift_autonomy'), 'config', 'autonomy.yaml')) as f:
            m = yaml.safe_load(f)['mission']
        fc = rig.es.robot['fork_carriage']
        self.after_pick = fc['x'] + fc['size']['x'] / 2.0 + m['entry_gap'] + m['back_out']

    # ------------------------------------------------------------ hazırlık
    def at_source(self, d=1.0, lat=0.0, yaw_deg=0.0, keep=(0, 1, 2)):
        rig = self.rig
        for name, _, (x, y, z, yw) in rig.parcels:
            i = int(name.rsplit('_', 1)[1])
            if i in keep:
                self.gz.set_pose(name, [x, y, z], simt.Rz(yw))
            else:
                rig.park(name, i)
        simt.fork_to(self.gz, rig.rg, 0.0)
        src = rig.es.station('source')
        p_xy, yaw = pose_in_front(src, d, lat, math.radians(yaw_deg))
        simt.place_robot(self.gz, p_xy, yaw, settle=rig.att_wait)

    def loaded_at(self, station, d):
        rig, carried = self.rig, 'parcel_source_0'
        rig.restore_parcels(skip=(carried,))
        rig.park(carried, 0)
        simt.fork_to(self.gz, rig.rg, rig.carry_q)
        p_xy, yaw = pose_in_front(rig.es.station(station), d)
        simt.place_robot(self.gz, p_xy, yaw, settle=rig.att_wait)
        rig.load_fork(carried)
        self.gz.wait_sim(1.0)
        return carried

    def occlude(self, station, on, name='a5_occluder'):
        """Bütün istasyon önüne (ön çizginin 8 cm önü) 1.0 m genişlik, 0.6 m yükseklikte görsel pano."""
        if on:
            st = self.rig.es.station(station)
            x, y, _ = sl.compose(st['pose'], -0.08, 0.0)
            self.gz.spawn(name, occluder_sdf(name, 1.0, 0.6), [x, y, 0.0], simt.Rz(st['pose'][2]))
        else:
            self.gz.remove(name)

    # ------------------------------------------------------------ senaryolar
    def e1_estop(self):
        carried = self.loaded_at('source', self.after_pick)
        mo, ev = Motion(self.gz), {}

        def f(state):
            mo.update()
            t = self.gz.now()
            if 'press' not in ev and state in ('TRANSIT', 'LANE_FOLLOW') and mo.v > 0.08:
                self.op.estop = True
                ev['press'], ev['v_at_press'] = t, mo.v
            if 'press' in ev and 'paused' not in ev and state == 'PAUSED':
                ev['paused'] = t - ev['press']
            if 'press' in ev and 'stop' not in ev and mo.stopped:
                ev['stop'] = t
            elif 'stop' in ev and 'release' not in ev and t - ev['press'] > 3.0:
                self.op.estop = False                          # bırak: kendiliğinden devam etmemeli
                ev['release'] = t
            elif 'release' in ev and 'resume' not in ev:
                if not mo.stopped or state != 'PAUSED':
                    ev['moved_after_release'] = True
                if t - ev['release'] > 3.0:
                    ev['resume'] = t
                    ev['resume_reply'] = self.op.call('resume')
        res = self.rig.run('transit', 'red', limit=240.0, sample=f)
        stop_s = ev['stop'] - ev['press'] if 'stop' in ev else None
        ok = (stop_s is not None and stop_s <= ESTOP_STOP_S and ev.get('paused') is not None and ev['paused'] < 1.0
              and not ev.get('moved_after_release') and res.get('final') == 'DONE'
              and 'PAUSED' in res.get('states', []))
        return dict(res, events={k: v for k, v in ev.items()}, stop_s=stop_s, carried=carried, ok=ok,
                    expect=f'e-stop → ≤ {ESTOP_STOP_S} s durma, DURAKLADI; bırakınca kendiliğinden devam yok; Devam → DONE')

    def e2_teleop(self):
        self.at_source(1.0)
        ev = {}

        def f(state):
            t = self.gz.now()
            if 'teleop' not in ev and state == 'ENTER':
                ev['teleop'] = t
                tw = Twist()
                tw.linear.x = -0.05
                for _ in range(5):                             # 0.5 s operatör sürüşü (geri)
                    self.teleop_pub.publish(tw)
                    time.sleep(0.1)
                self.teleop_pub.publish(Twist())
            elif 'teleop' in ev and 'paused' not in ev and state == 'PAUSED':
                ev['paused'] = t - ev['teleop']
            elif 'paused' in ev and 'resume' not in ev and t - ev['teleop'] > 3.0:
                ev['resume'] = t
                ev['resume_reply'] = self.op.call('resume')
        res = self.rig.run('pick', limit=150.0, sample=f)
        picked = self._on_fork('parcel_source_2')
        ok = 'paused' in ev and res.get('final') == 'DONE' and picked and 'OBSERVE' in res['states'][res['states'].index('PAUSED'):]
        return dict(res, events=ev, picked=picked, ok=ok, expect='teleop → DURAKLADI; Devam → yeniden gözlem; alma başarılı')

    def e3_mode(self):
        self.loaded_at('dest_red', 1.2)
        mo, ev = Motion(self.gz), {}

        def f(state):
            mo.update()
            t = self.gz.now()
            if 'manual' not in ev and state == 'ALIGN' and mo.v + mo.w > 0.05:
                ev['manual'] = t
                ev['manual_reply'] = self.op.call('manual')
            elif 'manual' in ev and 'stop' not in ev and mo.stopped:
                ev['stop'] = t - ev['manual']
            elif 'stop' in ev and 'resume' not in ev and t - ev['manual'] > 3.0:
                if not mo.stopped:
                    ev['moved_in_manual'] = True
                ev['auto_reply'] = self.op.call('auto')
                ev['resume_reply'] = self.op.call('resume')
                ev['resume'] = t
        res = self.rig.run('drop', limit=180.0, sample=f)
        parcels = self._parcel_in_bay('parcel_source_0', 'dest_red', 1)
        ok = ('stop' in ev and ev['stop'] < 1.5 and not ev.get('moved_in_manual') and res.get('final') == 'DONE'
              and parcels['in_bay'])
        return dict(res, events=ev, drop=parcels, ok=ok, expect='Manuel → DURAKLADI, robot durur; Otonom + Devam → bırakma')

    def _occlusion(self, seconds):
        self.at_source(1.0)
        ev = {}

        def f(state):
            t = self.gz.now()
            if 'on' not in ev and state == 'ENTER':
                self.occlude('source', True)
                ev['on'] = t
            elif 'on' in ev and 'off' not in ev and seconds is not None and t - ev['on'] > seconds:
                self.occlude('source', False)
                ev['off'] = t
        res = self.rig.run('pick', limit=180.0, sample=f)
        if 'on' in ev and 'off' not in ev:
            self.occlude('source', False)
        mo = self._settled_motion()
        return res, ev, mo.stopped

    def e4_occlude_brief(self):
        res, ev, _ = self._occlusion(3.0)
        picked = self._on_fork('parcel_source_2')
        ok = 'RECOVER' in res.get('states', []) and res.get('final') == 'DONE' and picked
        return dict(res, events=ev, picked=picked, ok=ok, expect='3 s örtme → RECOVER; sonra alma başarılı')

    def e5_occlude_permanent(self):
        res, ev, stopped = self._occlusion(None)
        ok = res.get('final') == 'FAULT' and 'kurtarma' in (res.get('fault') or '') and stopped
        return dict(res, events=ev, stopped=stopped, ok=ok, expect='kalıcı örtme → FAULT (kurtarma tükendi), robot duruyor')

    def e6_parcel_lost(self):
        carried = self.loaded_at('source', self.after_pick)
        ev = {}

        def f(state):
            t = self.gz.now()
            if 'removed' not in ev and state in ('TRANSIT', 'LANE_FOLLOW'):
                self.rig.park(carried, 1)
                ev['removed'] = t
            elif 'removed' in ev and 'fault' not in ev and state == 'FAULT':
                ev['fault'] = t - ev['removed']
        res = self.rig.run('transit', 'red', limit=150.0, sample=f)
        mo = self._settled_motion()
        # FAULT enjeksiyondan sonra gelmeli (önce gelen yanlış alarmdır)
        ok = ('removed' in ev and 'fault' in ev and res.get('final') == 'FAULT'
              and 'yük kaybı' in (res.get('fault') or '') and mo.stopped)
        return dict(res, events=ev, stopped=mo.stopped, ok=ok, expect='koli kaldırılır → FAULT (yük kaybı), robot duruyor')

    def e7_pick_failure(self):
        self.at_source(1.0)
        ev = {}

        def f(state):
            if 'removed' not in ev and state == 'LIFT':
                self.rig.park('parcel_source_2', 2)          # hedef (en soldaki) koli kaldırılmadan alınır
                ev['removed'] = self.gz.now()
        res = self.rig.run('pick', limit=200.0, sample=f)
        picked = self._on_fork('parcel_source_1')
        retried = res.get('states', []).count('OBSERVE') >= 2
        ok = 'removed' in ev and res.get('final') == 'DONE' and picked and retried
        return dict(res, events=ev, picked_next=picked, ok=ok,
                    expect='alma başarısız → yeniden deneme; soldan ilk dolu (yeşil) alınır')

    def e8_swapped(self):
        rig = self.rig
        rig.restore_parcels()
        simt.fork_to(self.gz, rig.rg, 0.0)
        simt.place_robot(self.gz, (0.0, 0.0), 0.0, settle=rig.att_wait)
        res = rig.run('full', limit=A4_LIMIT_S + 120.0)
        out = _a4_parcels(rig)
        ok = res.get('final') == 'DONE' and all(o['correct'] for o in out)
        return dict(res, parcels=out, ok=ok, expect='kırmızı ↔ mavi yer değiştirmiş; tam görev, koliler renge göre doğru')

    # ------------------------------------------------------------ ölçüm yardımcıları
    def _settled_motion(self):
        """FAULT'tan sonra yavaşlama (ivme sınırı) bitsin; hız 0.2 s aralıklı iki pozdan."""
        self.gz.wait_sim(1.5)
        mo = Motion(self.gz)
        mo.update()
        self.gz.wait_sim(0.2)
        mo.update()
        return mo

    def _on_fork(self, name):
        rg = self.rig.rg
        rel = simt.robot_bf(self.gz).inv() * self.gz.model(name)
        return bool(rg.carriage_front_x < rel.p[0] < rg.carriage_front_x + rg.fork_len
                    and abs(rel.p[1]) < 0.05 and rel.p[2] > rg.fork_top0 + 0.03)

    def _parcel_in_bay(self, name, station, bay):
        rel = self.rig.bay_parcel_pose(self.rig.es.station(station), bay).inv() * self.gz.model(name)
        off = float(np.linalg.norm(rel.p[:2]) * 1000)
        return {'offset_mm': off, 'tilt_deg': float(simt.tilt_deg(self.gz.model(name).R)), 'in_bay': bool(off < 15.0)}


def swapped_layout(es, out):
    """Kırmızı ve mavi istasyonların pozları yer değiştirmiş stations.yaml (zemin şeritleri değişmez)."""
    lay = yaml.safe_load(open(es.stations_file))
    st = {s['name']: s for s in lay['stations']}
    st['dest_red']['pose'], st['dest_blue']['pose'] = st['dest_blue']['pose'], st['dest_red']['pose']
    path = os.path.join(out, 'stations_swapped.yaml')
    with open(path, 'w') as f:
        yaml.safe_dump(lay, f, sort_keys=False)
    return path


def run_session(world, scenarios, out, stations_file, method, log, res):
    es = EvalSetup(world, out, stations_file=stations_file)
    tag = f'a5_{world}' + ('_swapped' if stations_file else '')
    extra = [f'stations_file:={stations_file}'] if stations_file else []
    with simt.SimSession(es, tag, control=True, extra=extra) as sess:
        procs, gz = [], None
        try:
            for pkg, lf, args in (('forklift_perception', 'perception.launch.py', []),
                                  ('forklift_autonomy', 'autonomy.launch.py', [f'transit_method:={method}'])):
                procs.append(subprocess.Popen(['ros2', 'launch', pkg, lf, *args],
                                              stdout=open(os.path.join(es.logs, f'{pkg}_{tag}.log'), 'w'),
                                              stderr=subprocess.STDOUT, start_new_session=True))
            gz = simt.Gz(es)
            simt.wait_world_ready(es, gz, sess, sl.parcels(es.params, es.layout))
            gz.record(False)
            if not rclpy.ok():
                rclpy.init()
            op = Operator()
            ex = SingleThreadedExecutor()               # oturum başına ayrı yürütücü (E8 ikinci oturum)
            ex.add_node(op)
            threading.Thread(target=ex.spin, daemon=True).start()
            simt.place_robot(gz, (0.0, 0.0), 0.0, settle=4.0)
            fx = Faults(Rig(es, gz, op), log)
            for name in scenarios:
                try:
                    r = getattr(fx, {'E1': 'e1_estop', 'E2': 'e2_teleop', 'E3': 'e3_mode', 'E4': 'e4_occlude_brief',
                                     'E5': 'e5_occlude_permanent', 'E6': 'e6_parcel_lost', 'E7': 'e7_pick_failure',
                                     'E8': 'e8_swapped'}[name])()
                except Exception as e:
                    import traceback
                    traceback.print_exc()
                    r = {'final': NOT_RUN, 'detail': repr(e), 'ok': False}
                op.estop = False
                op.call('manual')
                op.call('cancel')
                simt.fork_to(gz, fx.rig.rg, 0.0)
                r['status'] = PASS if r.get('ok') else (NOT_RUN if r.get('final') == NOT_RUN else FAIL)
                res['scenarios'][f'{name} {world}'] = r
                log(f'[A5] {name} {world}: {r["status"]} — {r.get("final")} ({r.get("detail")}) '
                    f'{ {k: v for k, v in r.items() if k in ("stop_s", "events", "picked", "picked_next", "drop", "stopped")} }')
                with open(os.path.join(out, f'a5_{world}.json'), 'w') as f:
                    json.dump(res, f, indent=1, ensure_ascii=False, default=str)
            fx.rig.restore_parcels()
            ex.shutdown()
            op.destroy_node()
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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', default='station_test')
    ap.add_argument('--only', default=','.join(ALL))
    ap.add_argument('--out', default=os.path.expanduser('~/cv_forklift_ws/docs/logs/cv/f5'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    want = [s.strip() for s in a.only.split(',') if s.strip()]
    method = 'lane' if a.world == 'factory' else 'sign'
    res = {'world': a.world, 'started': time.strftime('%Y-%m-%d %H:%M:%S'), 'scenarios': {}}

    def log(t):
        print(t, flush=True)
    normal = [s for s in want if s != 'E8']
    if normal:
        run_session(a.world, normal, a.out, None, method, log, res)
    if 'E8' in want:
        path = swapped_layout(EvalSetup(a.world, a.out), a.out)
        run_session(a.world, ['E8'], a.out, path, method, log, res)
    res['A5'] = {'status': PASS if res['scenarios'] and all(r['status'] == PASS for r in res['scenarios'].values()) else FAIL,
                 'by_scenario': {k: r['status'] for k, r in res['scenarios'].items()}}
    res['finished'] = time.strftime('%Y-%m-%d %H:%M:%S')
    log(f'[A5 {a.world}] {res["A5"]}')
    with open(os.path.join(a.out, f'a5_{a.world}.json'), 'w') as f:
        json.dump(res, f, indent=1, ensure_ascii=False, default=str)
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
