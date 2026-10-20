#!/usr/bin/env python3
"""Verilen modellerin hepsi sunucu sahnesinde görünene kadar bekler (DECISIONS D-52).

sim.launch.py Gazebo GUI istemcisini (gz sim -g) bu süreç çıkınca başlatır.
GUI açılışta sahneyi /world/<w>/scene/info'dan bir kez alır, sonra yalnızca
değişiklikleri izler; bu ikisinin arasında eklenen bir model GUI'de hiç
görünmez (yarış). Bu yüzden ölçüt create süreçlerinin bitmesi değil (async
/create varlık oluşmadan döner, D-49), GUI'nin okuyacağı sahnenin kendisidir.

Servis çağrıları gz service CLI ile yapılır (D-49).

  wait_for_models.py --models forklift,source,parcel_source_0 [--timeout 90]

Çıkış kodu: 0 → hepsi sahnede; 1 → zaman aşımı (eksikler yazdırılır).
"""

import argparse
import subprocess
import sys
import time

from google.protobuf import text_format
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.stringmsg_v_pb2 import StringMsg_V


def gz_request(service, rep_t, timeout_ms=2000):
    """gz service ile Empty istek gönderir; yanıt yoksa None döner."""
    cmd = ['gz', 'service', '-s', service, '--reqtype', 'gz.msgs.Empty',
           '--reptype', f'gz.msgs.{rep_t.__name__}', '--timeout', str(timeout_ms), '--req', '']
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_ms / 1000.0 + 10)
    except subprocess.TimeoutExpired:
        return None
    if p.returncode != 0 or not p.stdout.strip():
        return None
    try:
        return text_format.Parse(p.stdout, rep_t(), allow_unknown_field=True)
    except text_format.ParseError:
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--models', required=True, help='virgülle ayrılmış model adları')
    ap.add_argument('--timeout', type=float, default=90.0, help='[s] duvar saati')
    ap.add_argument('--period', type=float, default=0.5, help='[s] sorgu aralığı')
    args = ap.parse_args()

    expected = {m for m in args.models.split(',') if m}
    t0 = time.time()
    world, missing = None, sorted(expected)
    while time.time() - t0 < args.timeout:
        if world is None:
            worlds = gz_request('/gazebo/worlds', StringMsg_V)
            if worlds is not None and worlds.data:
                world = worlds.data[0]
        if world is not None:
            scene = gz_request(f'/world/{world}/scene/info', Scene)
            if scene is not None:
                missing = sorted(expected - {m.name for m in scene.model})
                if not missing:
                    print(f'wait_for_models: {len(expected)} model sahnede ({world}, '
                          f'{time.time() - t0:.1f} s)', flush=True)
                    return 0
        time.sleep(args.period)
    print(f'wait_for_models: {args.timeout:.0f} s zaman aşımı, eksik: {", ".join(missing)} '
          f'(dünya: {world})', file=sys.stderr, flush=True)
    return 1


if __name__ == '__main__':
    sys.exit(main())
