"""Otonomi çekirdeği: kinematik simülasyonda (tek tekerlekli araç, kusursuz ölçüm) görev akışı ve servo.

Gazebo'suz, saniyeler içinde koşar; durum makinesi ve denetleyici hatalarını kapalı döngüden önce yakalar.
"""

import math
import os

import numpy as np
import pytest
import yaml

from forklift_autonomy.core.control import BayServo, GoTo
from forklift_autonomy.core.mission import Mission, PickMission
from forklift_autonomy.core.se2 import SE2, wrap
from forklift_autonomy.core.tracker import BayTracker, LaneTracker, SignTracker

CFG = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), '..', 'config', 'autonomy.yaml')))
ROBOT = {'stop_distance': 0.13 + CFG['mission']['entry_gap'], 'fork_min': 0.0, 'fork_max': 0.15}
DT = 0.05


def drive(pose, v, w):
    return SE2(pose.x + v * math.cos(pose.yaw) * DT, pose.y + v * math.sin(pose.yaw) * DT, wrap(pose.yaw + w * DT))


def test_goto_reaches_pose():
    p, g = SE2(0, 0, 0.3), GoTo((1.0, -0.3), 0.0, 0.15, CFG['mission']['motion'])
    for _ in range(2000):
        v, w, done = g.step(p)
        if done:
            break
        p = drive(p, v, w)
    assert done
    assert np.hypot(p.x - 1.0, p.y + 0.3) < 0.015 and abs(p.yaw) < math.radians(0.5)


@pytest.mark.parametrize('lat0,yaw0', [(0.02, 0.0), (-0.02, math.radians(3)), (0.0, math.radians(-3))])
def test_servo_converges_before_entry(lat0, yaw0):
    entry, theta = np.array([1.0, 0.0]), 0.0
    p = SE2(1.0 - CFG['mission']['align_distance'], lat0, yaw0)
    s = BayServo(CFG['mission']['servo'], ROBOT['stop_distance'], CFG['mission']['v_dock'])
    for _ in range(4000):
        v, w, done = s.step(p, entry, theta)
        if done:
            break
        p = drive(p, v, w)
    dist, lat, eh = s.errors(p, entry, theta)
    assert done and abs(dist - ROBOT['stop_distance']) < 0.005
    assert abs(lat) < 0.005 and abs(eh) < math.radians(1.0)


def test_pick_mission_reaches_done_with_perfect_perception():
    tr = BayTracker(CFG['tracker'])
    m = PickMission(CFG['mission'], ROBOT, tr, SignTracker(CFG['tracker']), LaneTracker(CFG['tracker']), 'sign')
    # istasyon: ön çizgi x = 1.5, üç bölme y = −0.3, 0, 0.3; doluluk: yalnız sağdaki (y = −0.3)
    bays = [np.array([1.5, y]) for y in (-0.3, 0.0, 0.3)]
    p, q, t = SE2(0.4, 0.1, math.radians(5)), 0.0, 0.0
    m.start(t, 'pick')
    for k in range(6000):
        t = k * DT
        rel = [p.inv().apply(b) for b in bays]                          # robota göre (kusursuz algı)
        vis = [r for r in rel if 0.15 < r[0] < 2.0]
        tr.add_entries(t, [(p.apply(r), 0.0, True, 'pair', p.apply(r)) for r in vis])
        tr.add_parcels(t, [(bays[0] + [0.11, 0.0], 'red')])
        loaded = m.state in ('BACK_OUT', 'VERIFY', 'DONE')
        v, w, fork = m.step(t, p, q, loaded)
        if fork is not None:
            q += np.clip(fork - q, -0.05 * DT, 0.05 * DT)
        p = drive(p, v, w)
        if not m.active:
            break
    assert m.state == 'DONE', m.log
    picked = [e for e in m.log if e[1] == 'ALIGN'][0][2]
    assert 'bölme 3/3' in picked                                         # soldan sağa: sağdaki tek dolu


def test_label_does_not_shift_axis_while_pairs_are_seen():
    tr = BayTracker(CFG['tracker'])
    for k in range(5):
        tr.add_entries(k * 0.05, [((1.5, 0.0), 0.0, True, 'pair', (1.5, 0.0))])
    # etiket: 2° hatalı yönle kurulmuş giriş (0.30 m kol → ~10 mm yanal), merkez doğru
    th = math.radians(2.0)
    anchor = np.array([1.80025, 0.0])
    entry = anchor - 0.30025 * np.array([math.cos(th), math.sin(th)])
    for k in range(5, 20):
        tr.add_entries(k * 0.05, [(entry, th, True, 'label', anchor)])
    b = tr.bays[0]
    assert abs(b.entry[1]) < 0.001 and abs(b.entry[0] - 1.5) < 0.001


# =============================================================================
#  F4: dört istasyonlu sahte dünya (station_test / factory yerleşimi), kusurlu algı
# =============================================================================
STATIONS = {'neutral': (1.5, 0.0, 0.0), 'red': (0.0, 2.25, math.pi / 2),
            'green': (0.0, -2.25, -math.pi / 2), 'blue': (-2.25, 0.0, math.pi)}
PITCH = 0.30


class FakeWorld:
    """Robotun algısını taklit eder: görüş alanı, menzil ve tabelada uzaklıkla büyüyen hata (D-66)."""

    def __init__(self, occupancy):
        self.occ = {k: list(v) for k, v in occupancy.items()}   # renk → [mühür | None] × 3 (bölme 0 = −y)
        self.carry = None

    def bay(self, color, i):
        x, y, a = STATIONS[color]
        S = SE2(x, y, a)
        return S.apply((0.0, (i - 1) * PITCH)), a

    def sense(self, p, tr, signs, lane, t, q):
        inv = p.inv()
        ent = []
        for color in STATIONS:
            for i in range(3):
                e, a = self.bay(color, i)
                r = inv.apply(e)
                if 0.15 < r[0] < 2.0 and abs(math.atan2(r[1], r[0])) < math.radians(40):
                    ent.append((e, a, True, 'pair', e))
        tr.add_entries(t, ent)
        cam = p.apply((0.10, 0.0))
        ps = []
        for color in STATIONS:
            for i in range(3):
                if self.occ[color][i]:
                    e, a = self.bay(color, i)
                    f = e + 0.01 * np.array([math.cos(a), math.sin(a)])
                    r = inv.apply(f)
                    if 0.5 < r[0] < 3.0 and abs(math.atan2(r[1], r[0] - 0.1)) < math.radians(30):
                        ps.append((f, self.occ[color][i]))
        tr.add_parcels(t, ps, view=(cam, p.yaw, math.radians(25), 0.5, 3.0))
        for color, (x, y, a) in STATIONS.items():
            r = inv.apply((x, y))
            rng = float(np.hypot(*r))
            if 0.6 < rng < 3.5 and abs(math.atan2(r[1], r[0])) < math.radians(30):
                err = 0.05 * rng                                   # ~5 cm/m yanal, ~4°/m yaw hatası
                xy = SE2(x, y, a).apply((0.0, err))
                signs.add(t, color, xy, a + math.radians(4.0 * rng), rng)
        # şerit rengi istasyonun rengi; kaynağa giden şerit sarı (katalog lane_neutral)
        st = {'yellow': 'neutral'}.get(lane.color, lane.color)
        if lane.color and st in STATIONS and st != 'neutral' or lane.color == 'yellow':
            x, y, a = STATIONS[st]
            d = np.array([math.cos(a), math.sin(a)])
            nrm = np.array([-d[1], d[0]])
            a0, a1 = 0.12, float(np.hypot(x, y)) - 0.40
            front = float(np.hypot(x, y))

            def seen(q):
                r = inv.apply(q)
                return 0.6 < r[0] < 2.4 and abs(r[1]) < r[0] * math.tan(math.radians(38))

            def cand(points, bar=None):
                pts = [q for q in points if seen(q)]
                return (pts, [float(np.hypot(*inv.apply(q))) for q in pts], bar) if len(pts) >= 2 else None
            bar = d * a1 if seen(d * a1) else None
            cs = [cand([d * s_ for s_ in np.arange(a0, a1, 0.02)], bar)]
            # yanıltıcılar (G7): önlükteki bölme şeritleri (±0.30) ve çevre bandının kenarları (±0.42)
            for off in (-0.42, -0.30, 0.30, 0.42):
                cs.append(cand([d * s_ + nrm * off for s_ in np.arange(front - 0.30, front, 0.02)]))
            cs = [c_ for c_ in cs if c_ is not None]
            if cs:
                lane.add_candidates(t, cs, p.yaw)

    def fork_event(self, p, q_prev, q):
        """Çatal kalkınca bölmedeki koli çatala, inince çatal üstündeki koli bölmeye."""
        for color in STATIONS:
            for i in range(3):
                e, a = self.bay(color, i)
                rel = SE2(e[0], e[1], a).inv().apply(p.xy)
                if abs(rel[1]) < 0.02 and -0.2 < rel[0] < -0.1:              # taban tam girişte
                    if q_prev < 0.03 <= q and self.occ[color][i] and self.carry is None:
                        self.carry, self.occ[color][i] = self.occ[color][i], None
                    elif q_prev > 0.03 >= q and self.carry is not None:
                        self.occ[color][i], self.carry = self.carry, None


def run_mission(world, method, task='full', target='', p0=SE2(0, 0, 0), steps=40000):
    tr, sg, ln = BayTracker(CFG['tracker']), SignTracker(CFG['tracker']), LaneTracker(CFG['tracker'])
    m = Mission(CFG['mission'], ROBOT, tr, sg, ln, method)
    p, q, t = p0, (CFG['mission']['carry_q'] if world.carry else 0.0), 0.0
    m.start(t, task, target)
    for k in range(steps):
        t = k * DT
        world.sense(p, tr, sg, ln, t, q)
        v, w, fork = m.step(t, p, q, world.carry is not None)
        if fork is not None:
            q0, q = q, q + float(np.clip(fork - q, -0.05 * DT, 0.05 * DT))
            world.fork_event(p, q0, q)
        p = drive(p, v, w)
        if not m.active:
            break
    return m, p


@pytest.mark.parametrize('method', ['sign', 'lane'])
def test_full_mission_delivers_every_parcel_to_its_station(method):
    w = FakeWorld({'neutral': ['red', 'green', 'blue'], 'red': [None] * 3, 'green': [None] * 3, 'blue': [None] * 3})
    m, _ = run_mission(w, method)
    assert m.state == 'DONE' and 'kaynak istasyon boş' in m.detail, m.log[-6:]
    assert m.parcels_done == 3
    if method == 'lane':                                              # her bacak şeritle (1 + 3 × 2 transit)
        assert sum(1 for e in m.log if e[1] == 'LANE_FOLLOW') == 7, [e for e in m.log if e[1].startswith('LANE')]
    for c in ('red', 'green', 'blue'):
        assert w.occ[c] == [None, c, None], (c, w.occ[c])            # önce orta bölme (K10)


def test_drop_prefers_middle_then_left():
    w = FakeWorld({'neutral': [None] * 3, 'red': [None, 'red', None], 'green': [None] * 3, 'blue': [None] * 3})
    w.carry = 'red'
    x, y, a = STATIONS['red']
    p0 = SE2(*SE2(x, y, a).apply((-1.0, 0.1)), a + math.radians(5))
    m, _ = run_mission(w, 'sign', 'drop', p0=p0, steps=8000)
    assert m.state == 'DONE', m.log[-4:]
    assert w.occ['red'] == [None, 'red', 'red']                     # robotun solu = istasyonun +y tarafı


@pytest.mark.parametrize('method,target', [('sign', 'blue'), ('lane', 'red'), ('lane', 'green')])
def test_transit_reaches_predock(method, target):
    w = FakeWorld({'neutral': [None] * 3, 'red': [None] * 3, 'green': [None] * 3, 'blue': [None] * 3})
    w.carry = 'red'
    m, p = run_mission(w, method, 'transit', target, p0=SE2(0.75, 0.0, 0.0), steps=12000)
    assert m.state == 'DONE', m.log[-6:]
    x, y, a = STATIONS[target]
    rel = SE2(x, y, a).inv() * p                                    # robot istasyon çerçevesinde
    assert abs(rel.x + CFG['mission']['transit']['predock_distance']) < 0.10, rel
    assert abs(rel.y) < 0.10 and abs(math.degrees(rel.yaw)) < 5.0, rel


def test_settled_waits_for_all_bays_and_occupancy():
    tr = BayTracker(CFG['tracker'])
    view = ((0.0, 0.0), 0.0, math.radians(25), 0.5, 3.0)
    for k in range(10):                                              # yalnız iki bölme görülüyor
        tr.add_entries(k * 0.05, [((1.5, y), 0.0, True, 'pair', (1.5, y)) for y in (-0.3, 0.0)])
        tr.add_parcels(k * 0.05, [((1.51, -0.3), 'red')], view)
    tr.add_entries(0.5, [((1.5, 0.3), 0.0, True, 'pair', (1.5, 0.3))])   # üçüncüsü geç açılıyor
    assert not tr.settled(0.6) and not tr.settled(1.0)                    # yeni iz, henüz doğrulanmadı
    for k in range(11, 60):
        tr.add_entries(k * 0.05, [((1.5, y), 0.0, True, 'pair', (1.5, y)) for y in (-0.3, 0.0, 0.3)])
        tr.add_parcels(k * 0.05, [((1.51, -0.3), 'red')], view)
    assert tr.settled(3.0)
    b = tr.left_to_right(tr.confirmed())
    assert [tr.occupied(x, 3.0) for x in b] == [False, False, True] and tr.empty(b[0], 3.0)


def test_bay_out_of_view_is_not_empty():
    tr = BayTracker(CFG['tracker'])
    for k in range(30):
        tr.add_entries(k * 0.05, [((1.5, 0.0), 0.0, True, 'pair', (1.5, 0.0))])
        tr.add_parcels(k * 0.05, [], view=((0.0, 0.0), math.pi, math.radians(25), 0.5, 3.0))   # arkaya bakan kamera
    assert tr.settled(1.5) and not tr.empty(tr.bays[0], 1.5) and not tr.occupied(tr.bays[0], 1.5)


def test_drop_does_not_wait_for_out_of_view_side_bay():
    """Orta bölme boş görülüyorsa görüş dışındaki yan bölme kararı değiştirmez (A2 duman testi bulgusu)."""
    tr, sg, ln = BayTracker(CFG['tracker']), SignTracker(CFG['tracker']), LaneTracker(CFG['tracker'])
    m = Mission(CFG['mission'], ROBOT, tr, sg, ln, 'sign')
    m.start(0.0, 'drop')
    p = SE2(0.7, 0.0, 0.0)
    for k in range(200):
        t = k * DT
        tr.add_entries(t, [((1.5, y), 0.0, True, 'pair', (1.5, y)) for y in (-0.3, 0.0, 0.3)])
        # yalnız orta ve sol bölme görüş alanında
        tr.add_parcels(t, [], view=((0.8, 0.15), 0.0, math.radians(20), 0.5, 3.0))
        m.step(t, p, 0.06, True)
        if m.state == 'ALIGN':
            break
    assert m.state == 'ALIGN' and m.bay.entry[1] == pytest.approx(0.0, abs=1e-6), m.log
    assert t < CFG['mission']['observe_max_s']


def test_pick_repositions_when_parcel_bay_is_not_tracked():
    """Koli görülüyor ama bölmesinin hedef çifti algılanmıyor (factory A1 bulgusu): 'kaynak boş' deme,
    kolinin önüne geç ve yeniden gözle."""
    tr, sg, ln = BayTracker(CFG['tracker']), SignTracker(CFG['tracker']), LaneTracker(CFG['tracker'])
    m = Mission(CFG['mission'], ROBOT, tr, sg, ln, 'sign')
    bays = [np.array([1.5, y]) for y in (-0.3, 0.0, 0.3)]
    p, q = SE2(0.5, 0.1, math.radians(5)), 0.0
    m.start(0.0, 'pick')
    for k in range(6000):
        t = k * DT
        rel = [p.inv().apply(b) for b in bays]
        # sağdaki bölmenin hedef çifti robot ona tam bakmadıkça görülmüyor
        vis = [b for b, r in zip(bays, rel) if 0.15 < r[0] < 2.0 and (b[1] > -0.2 or abs(r[1]) < 0.05)]
        tr.add_entries(t, [(b, 0.0, True, 'pair', b) for b in vis])
        tr.add_parcels(t, [(bays[0] + [0.01, 0.0], 'red')])
        v, w, fork = m.step(t, p, q, m.state in ('BACK_OUT', 'VERIFY'))
        if fork is not None:
            q += np.clip(fork - q, -0.05 * DT, 0.05 * DT)
        p = drive(p, v, w)
        if not m.active:
            break
    states = [e[1] for e in m.log]
    assert 'REPOSITION' in states and m.state == 'DONE' and 'koli çatalda' in m.detail, m.log
    assert abs(m.bay.entry[1] + 0.3) < 0.01


def test_pick_repositions_for_untracked_bay_left_of_choice():
    """Soldaki bölmenin çifti görülmüyor, kolisi görülüyor: sağdakini alma, önce sola konumlan."""
    tr, sg, ln = BayTracker(CFG['tracker']), SignTracker(CFG['tracker']), LaneTracker(CFG['tracker'])
    m = Mission(CFG['mission'], ROBOT, tr, sg, ln, 'sign')
    bays = [np.array([1.5, y]) for y in (-0.3, 0.0, 0.3)]
    p, q = SE2(0.5, -0.1, math.radians(-5)), 0.0
    m.start(0.0, 'pick')
    for k in range(6000):
        t = k * DT
        rel = [p.inv().apply(b) for b in bays]
        vis = [b for b, r in zip(bays, rel) if 0.15 < r[0] < 2.0 and (b[1] < 0.2 or abs(r[1]) < 0.05)]
        tr.add_entries(t, [(b, 0.0, True, 'pair', b) for b in vis])
        tr.add_parcels(t, [(b + [0.01, 0.0], c) for b, c in zip(bays, ('red', 'green', 'blue'))])
        v, w, fork = m.step(t, p, q, m.state in ('BACK_OUT', 'VERIFY'))
        if fork is not None:
            q += np.clip(fork - q, -0.05 * DT, 0.05 * DT)
        p = drive(p, v, w)
        if not m.active:
            break
    assert 'REPOSITION' in [e[1] for e in m.log] and m.state == 'DONE', m.log
    assert abs(m.bay.entry[1] - 0.3) < 0.01 and m.bay.parcel == 'blue'


def test_sign_estimate_robust_to_scattered_outliers():
    sg = SignTracker(CFG['tracker'])
    for k in range(8):
        sg.add(k * 0.03, 'neutral', (2.0 + 0.01 * (k % 3), 0.0), 0.0, 2.0)
    for k, xy in enumerate([(3.5, 1.0), (0.8, -1.2), (2.9, -1.5)]):     # dağınık, uzak ölçümler
        sg.add(0.25 + k * 0.01, 'neutral', xy, 0.0, 2.0)
    est = sg.estimate('neutral', 0.3)
    assert est is not None and np.linalg.norm(est[0] - [2.01, 0.0]) < 0.02 and est[2] == 8


def test_drop_reobserves_when_target_bay_pair_is_missed():
    """A2 bulgusu: sol bölmenin çifti o pozdan algılanmıyor, orta dolu, sağ görüş dışında. FAULT yerine
    istasyon ortasından yeniden gözle ve soldakine bırak."""
    tr, sg, ln = BayTracker(CFG['tracker']), SignTracker(CFG['tracker']), LaneTracker(CFG['tracker'])
    m = Mission(CFG['mission'], ROBOT, tr, sg, ln, 'sign')
    bays = [np.array([1.5, y]) for y in (-0.3, 0.0, 0.3)]
    p = SE2(0.3, 0.15, math.radians(10))
    m.start(0.0, 'drop')
    for k in range(4000):
        t = k * DT
        rel = [p.inv().apply(b) for b in bays]
        start_pose = abs(p.yaw - math.radians(10)) < 1e-6 and abs(p.y - 0.15) < 1e-6
        vis = [b for b, r in zip(bays, rel) if 0.15 < r[0] < 2.0 and not (start_pose and b[1] > 0.2)]
        tr.add_entries(t, [(b, 0.0, True, 'pair', b) for b in vis])
        cam = p.apply((0.1, 0.0))
        tr.add_parcels(t, [(bays[1] + [0.01, 0.0], 'green')], view=(cam, p.yaw, math.radians(25), 0.5, 3.0))
        v, w, fork = m.step(t, p, 0.06 if m.state not in ('LOWER', 'BACK_OUT', 'VERIFY', 'DONE') else 0.0,
                            m.state not in ('BACK_OUT', 'VERIFY'))
        p = drive(p, v, w)
        if m.state in ('ALIGN', 'FAULT'):
            break
    assert m.state == 'ALIGN' and 'REPOSITION' in [e[1] for e in m.log], m.log
    assert abs(m.bay.entry[1] - 0.3) < 1e-6
