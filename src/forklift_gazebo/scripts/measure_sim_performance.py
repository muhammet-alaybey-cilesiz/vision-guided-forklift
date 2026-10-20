#!/usr/bin/env python3
"""İstasyonlu dünyaların simülasyon performansını ölçer (RTF, kamera hızı, CPU/GPU/RAM).

Her yapılandırma için station_sim.launch.py ayrı bir GZ_PARTITION'da başlatılır,
istasyonlar eklenip (gui: GUI açılıp) --settle saniye beklendikten sonra
--duration saniye boyunca ölçülür:
  RTF         : Δsim_time / Δreal_time (/world/<w>/stats) ve raporlanan RTF ortalaması
  kamera Hz   : robot kameralarının gz topic'lerine gelen mesaj sayısı / süre
  CPU, RAM    : gz sunucu (-s) ve GUI (-g) süreçleri (psutil; %100 = bir çekirdek)
  GPU         : nvidia-smi kullanım ve bellek ortalaması (varsa); ölçüm öncesi
                boşta değer ayrıca kaydedilir
Sonuçlar JSON ve Markdown tablo olarak yazılır.

Kullanım:
  measure_sim_performance.py [--worlds station_test,factory] [--gui false,true]
                             [--duration 30] [--settle 15] [--out DIR]
                             [--factory-clutter full,clean]
--factory-clutter clean için kurulu factory.yaml'ın clutter: clean kopyasından
geçici bir dünya üretilir (kurulu dosyalar değişmez).
"""
import argparse
import json
import os
import shutil
import signal
import statistics
import subprocess
import sys
import tempfile
import threading
import time

import psutil

PKG = 'forklift_gazebo'


def share():
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(PKG)


def gpu_sample():
    try:
        out = subprocess.run(['nvidia-smi', '--query-gpu=utilization.gpu,memory.used',
                              '--format=csv,noheader,nounits'], capture_output=True, text=True, timeout=5).stdout
        u, m = out.strip().splitlines()[0].split(',')
        return float(u), float(m)
    except (OSError, ValueError, IndexError, subprocess.TimeoutExpired):
        return None


def gz_procs():
    """gz sunucu ve GUI süreçleri (ruby 'gz sim -s' / 'gz sim -g')."""
    found = {}
    for p in psutil.process_iter(['cmdline']):
        cmd = ' '.join(p.info['cmdline'] or [])
        if 'gz sim' not in cmd or 'ros2' in cmd:
            continue
        if ' -s ' in cmd + ' ':
            found['server'] = p
        elif ' -g ' in cmd + ' ':
            found['gui'] = p
    return found


def clean_world(tmpdir):
    """Kurulu factory.yaml'ın clutter: clean kopyasından geçici factory.sdf üretir."""
    import yaml
    sh = share()
    F = yaml.safe_load(open(os.path.join(sh, 'config', 'factory.yaml')))
    F['hall']['clutter'] = 'clean'
    fy = os.path.join(tmpdir, 'factory_clean.yaml')
    yaml.safe_dump(F, open(fy, 'w'), sort_keys=False, allow_unicode=True)
    world = os.path.join(tmpdir, 'factory.sdf')
    gen = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'generate_factory_world.py')
    subprocess.run([sys.executable, gen, '--factory', fy,
                    '--stations', os.path.join(sh, 'config', 'stations.yaml'),
                    '--params', os.path.join(sh, 'config', 'station_params.yaml'),
                    '--world', world, '--lanes', os.path.join(tmpdir, 'lanes.yaml')], check=True,
                   capture_output=True)
    return world


def run_case(world, gui, duration, settle, label, world_file=None):
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.world_stats_pb2 import WorldStatistics
    from gz.transport13 import Node

    os.environ['GZ_PARTITION'] = f'perf_{os.getpid()}_{label}'
    log = tempfile.NamedTemporaryFile('w+', suffix='.log', delete=False)
    cmd = ['ros2', 'launch', PKG, 'station_sim.launch.py', f'station_world:={world}',
           f'gui:={str(gui).lower()}', f'headless:={str(not gui).lower()}']
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    t0 = time.time()
    res = {'label': label, 'world': world, 'gui': gui}
    try:
        need = ['model eklendi'] + (['Gazebo GUI başlatılıyor'] if gui else [])
        while time.time() - t0 < 240:
            time.sleep(1)
            text = open(log.name).read()
            if all(n in text for n in need):
                break
        else:
            raise RuntimeError(f'{label}: dünya 240 s içinde hazır olmadı (log: {log.name})')
        res['ready_s'] = round(time.time() - t0, 1)
        time.sleep(settle)

        node = Node()
        lock = threading.Lock()
        stats, counts = [], {'/front_camera/image_raw': 0, '/lane_camera/image_raw': 0}

        def on_stats(msg):
            with lock:
                stats.append((time.time(), msg.sim_time.sec + msg.sim_time.nsec * 1e-9,
                              msg.real_time.sec + msg.real_time.nsec * 1e-9, msg.real_time_factor))

        def counter(topic):
            def cb(_msg):
                with lock:
                    counts[topic] += 1
            return cb

        sim_name = 'factory' if world.startswith('factory') else world
        node.subscribe(WorldStatistics, f'/world/{sim_name}/stats', on_stats)
        for t in counts:
            node.subscribe(Image, t, counter(t))
        procs = gz_procs()
        for p in procs.values():
            p.cpu_percent(None)
        gpu, rss = [], {k: [] for k in procs}
        with lock:
            for t in counts:
                counts[t] = 0
        t_start = time.time()
        while time.time() - t_start < duration:
            time.sleep(1)
            g = gpu_sample()
            if g:
                gpu.append(g)
            for k, p in procs.items():
                try:
                    rss[k].append(p.memory_info().rss / 2**20)
                except psutil.Error:
                    pass
        elapsed = time.time() - t_start
        with lock:
            st = [s for s in stats if s[0] >= t_start]
            cam = {t: round(n / elapsed, 1) for t, n in counts.items()}
        for k, p in procs.items():
            try:
                res[f'cpu_{k}_pct'] = round(p.cpu_percent(None), 0)
            except psutil.Error:
                res[f'cpu_{k}_pct'] = None
            res[f'rss_{k}_mib'] = round(statistics.mean(rss[k]), 0) if rss[k] else None
        if len(st) >= 2:
            res['rtf'] = round((st[-1][1] - st[0][1]) / (st[-1][2] - st[0][2]), 3)
            res['rtf_reported_mean'] = round(statistics.mean(s[3] for s in st), 3)
            res['rtf_reported_min'] = round(min(s[3] for s in st), 3)
        res['camera_hz'] = cam
        if gpu:
            res['gpu_util_pct'] = round(statistics.mean(g[0] for g in gpu), 0)
            res['gpu_mem_mib'] = round(statistics.mean(g[1] for g in gpu), 0)
        res['duration_s'] = round(elapsed, 1)
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(30)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
        time.sleep(5)
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--worlds', default='station_test,factory')
    ap.add_argument('--gui', default='false,true')
    ap.add_argument('--factory-clutter', default='full')
    ap.add_argument('--duration', type=float, default=30.0)
    ap.add_argument('--settle', type=float, default=15.0)
    ap.add_argument('--out', default='.')
    a = ap.parse_args()

    idle = gpu_sample()
    cases = []
    for gui in [v.strip().lower() == 'true' for v in a.gui.split(',')]:
        for w in a.worlds.split(','):
            for clutter in (a.factory_clutter.split(',') if w == 'factory' else ['-']):
                cases.append((w, gui, clutter))
    results = []
    tmp = tempfile.mkdtemp()
    for w, gui, clutter in cases:
        label = f'{w}{"" if clutter in ("-", "full") else "_" + clutter}_{"gui" if gui else "headless"}'
        print(f'== {label}', flush=True)
        backup = None
        if clutter == 'clean':
            # station_sim.launch.py dünyayı share/worlds/factory.sdf'ten okur: ölçüm
            # süresince clean sürümü yerine koyar, sonra geri alır
            target = os.path.join(share(), 'worlds', 'factory.sdf')
            backup = target + '.perf_backup'
            shutil.copy(target, backup)
            shutil.copy(clean_world(tmp), target)
        try:
            r = run_case(w, gui, a.duration, a.settle, label)
            r['clutter'] = clutter
        finally:
            if backup:
                shutil.move(backup, target)
        print(json.dumps(r, ensure_ascii=False), flush=True)
        results.append(r)

    os.makedirs(a.out, exist_ok=True)
    meta = {'date': time.strftime('%Y-%m-%d %H:%M:%S'), 'gpu_idle': idle,
            'cpu': psutil.cpu_count(), 'duration_s': a.duration, 'settle_s': a.settle}
    json.dump({'meta': meta, 'results': results}, open(os.path.join(a.out, 'perf_results.json'), 'w'),
              indent=2, ensure_ascii=False)
    lines = ['| Yapılandırma | Hazır [s] | RTF (Δsim/Δreal) | RTF min | front / lane [Hz] | CPU sunucu / GUI [%] '
             '| RAM sunucu / GUI [MiB] | GPU kullanım [%] | GPU bellek [MiB] |',
             '|---|---|---|---|---|---|---|---|---|']
    for r in results:
        c = r.get('camera_hz', {})
        lines.append(
            f"| {r['label']} | {r.get('ready_s')} | {r.get('rtf')} | {r.get('rtf_reported_min')} "
            f"| {c.get('/front_camera/image_raw')} / {c.get('/lane_camera/image_raw')} "
            f"| {r.get('cpu_server_pct')} / {r.get('cpu_gui_pct', '—')} "
            f"| {r.get('rss_server_mib')} / {r.get('rss_gui_mib', '—')} "
            f"| {r.get('gpu_util_pct')} | {r.get('gpu_mem_mib')} |")
    md = '\n'.join(lines) + f"\n\nBoşta GPU (kullanım %, bellek MiB): {idle}; ölçüm {a.duration} s, bekleme {a.settle} s.\n"
    open(os.path.join(a.out, 'perf_results.md'), 'w').write(md)
    print(md)


if __name__ == '__main__':
    main()
