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
        self.blind = False                    # hata enjeksiyonu: hedef çiftleri ve etiketler görünmez (örtme)
        self.far_label = {}
        self.st = dict(STATIONS)              # istasyonların bu dünyadaki yeri; zemin şeritleri STATIONS'a göre boyalı                   # uzaktan (> 2 m) yanlış okunan tabela rengi: gerçek → okunan

    def bay(self, color, i):
        x, y, a = self.st[color]
        S = SE2(x, y, a)
        return S.apply((0.0, (i - 1) * PITCH)), a

    def sense(self, p, tr, signs, lane, t, q):
        inv = p.inv()
        ent = []
        for color in (() if self.blind else self.st):
            for i in range(3):
                e, a = self.bay(color, i)
                r = inv.apply(e)
                if 0.15 < r[0] < 2.0 and abs(math.atan2(r[1], r[0])) < math.radians(40):
                    ent.append((e, a, True, 'pair', e))
        tr.add_entries(t, ent)
        cam = p.apply((0.10, 0.0))
        ps = []
        for color in self.st:
            for i in range(3):
                if self.occ[color][i]:
                    e, a = self.bay(color, i)
                    f = e + 0.01 * np.array([math.cos(a), math.sin(a)])
                    r = inv.apply(f)
                    if 0.5 < r[0] < 3.0 and abs(math.atan2(r[1], r[0] - 0.1)) < math.radians(30):
                        ps.append((f, self.occ[color][i]))
        tr.add_parcels(t, ps, view=(cam, p.yaw, math.radians(25), 0.5, 3.0))
        for color, (x, y, a) in self.st.items():
            r = inv.apply((x, y))
            rng = float(np.hypot(*r))
            if 0.6 < rng < 3.5 and abs(math.atan2(r[1], r[0])) < math.radians(30):
                err = 0.05 * rng                                   # ~5 cm/m yanal, ~4°/m yaw hatası
                xy = SE2(x, y, a).apply((0.0, err))
                seen = self.far_label.get(color, color) if rng > 2.0 else color
                signs.add(t, seen, xy, a + math.radians(4.0 * rng), rng)
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
        for color in self.st:
            for i in range(3):
                e, a = self.bay(color, i)
                rel = SE2(e[0], e[1], a).inv().apply(p.xy)
                if abs(rel[1]) < 0.02 and -0.2 < rel[0] < -0.1:              # taban tam girişte
                    if q_prev < 0.03 <= q and self.occ[color][i] and self.carry is None:
                        self.carry, self.occ[color][i] = self.occ[color][i], None
                    elif q_prev > 0.03 >= q and self.carry is not None:
                        self.occ[color][i], self.carry = self.carry, None


def run_mission(world, method, task='full', target='', p0=SE2(0, 0, 0), steps=40000, hook=None, fork_stuck=False):
    """hook(m, world, t): her adımda hata enjeksiyonu için. fork_stuck: çatal hiç hareket etmez."""
    tr, sg, ln = BayTracker(CFG['tracker']), SignTracker(CFG['tracker']), LaneTracker(CFG['tracker'])
    m = Mission(CFG['mission'], ROBOT, tr, sg, ln, method)
    p, q, t = p0, (CFG['mission']['carry_q'] if world.carry else 0.0), 0.0
    m.start(t, task, target)
    for k in range(steps):
        t = k * DT
        if hook is not None:
            hook(m, world, t)
        world.sense(p, tr, sg, ln, t, q)
        v, w, fork = m.step(t, p, q, world.carry is not None)
        m.last_cmd = (v, w)
        if fork is not None and not fork_stuck:
            q0, q = q, q + float(np.clip(fork - q, -0.05 * DT, 0.05 * DT))
            world.fork_event(p, q0, q)
        p = drive(p, v, w)
        if not m.active:
            break
    return m, p


@pytest.mark.parametrize('method,finish_early', [('sign', True), ('lane', True), ('lane', False)])
def test_full_mission_delivers_every_parcel_to_its_station(method, finish_early):
    w = FakeWorld({'neutral': ['red', 'green', 'blue'], 'red': [None] * 3, 'green': [None] * 3, 'blue': [None] * 3})
    old = CFG['mission']['finish_when_source_seen_empty']
    CFG['mission']['finish_when_source_seen_empty'] = finish_early
    try:
        m, _ = run_mission(w, method)
    finally:
        CFG['mission']['finish_when_source_seen_empty'] = old
    why = 'kaynakta koli kalmadı' if finish_early else 'kaynak istasyon boş'      # son dönüş yok / var
    assert m.state == 'DONE' and why in m.detail, m.log[-6:]
    assert m.parcels_done == 3
    assert any('hafızadaki' in e[2] for e in m.log)                  # dönüşte görülmüş tabelaya önce dönülür
    if method == 'lane':                                              # her bacak şeritle: 1 + 3 × 2 (− son dönüş)
        n = sum(1 for e in m.log if e[1] == 'LANE_FOLLOW')
        assert n == (6 if finish_early else 7), [e for e in m.log if e[1].startswith('LANE')]
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


# =============================================================================
#  F5: hata ve kurtarma (plan §7.2)
# =============================================================================
def _source_world():
    return FakeWorld({'neutral': ['red', 'green', 'blue'], 'red': [None] * 3, 'green': [None] * 3, 'blue': [None] * 3})


P_SOURCE = SE2(0.5, 0.0, 0.0)                        # kaynağın 1 m önü


def test_target_occluded_briefly_recovers_and_picks():
    w = _source_world()
    box = {}

    def hook(m, world, t):                           # girişte 3 s örtme
        if m.state == 'ENTER' and 'on' not in box:
            box['on'] = t
        world.blind = 'on' in box and t - box['on'] < 3.0
    m, _ = run_mission(w, 'sign', 'pick', p0=P_SOURCE, steps=8000, hook=hook)
    states = [e[1] for e in m.log]
    assert 'RECOVER' in states and m.state == 'DONE' and 'koli çatalda' in m.detail, m.log[-8:]


def test_target_occluded_permanently_faults_and_stops():
    w = _source_world()

    def hook(m, world, t):
        if m.state == 'ENTER':
            world.blind = True
    m, _ = run_mission(w, 'sign', 'pick', p0=P_SOURCE, steps=8000, hook=hook)
    assert m.state == 'FAULT' and 'kurtarma denemesi tükendi' in m.fault, m.log[-6:]
    assert m.last_cmd == (0.0, 0.0)


def test_parcel_lost_while_carrying_faults():
    w = _source_world()

    def hook(m, world, t):                           # transitte koli çataldan düşer
        if m.state == 'TRANSIT' and m.loaded:
            world.carry = None
    m, _ = run_mission(w, 'sign', 'full', p0=P_SOURCE, steps=20000, hook=hook)
    assert m.state == 'FAULT' and 'yük kaybı' in m.fault, m.log[-6:]


def test_pick_failure_retries_with_next_parcel():
    w = _source_world()
    done = {}

    def hook(m, world, t):                           # kaldırmadan hemen önce koli bölmeden alınır
        if m.state == 'LIFT' and 'x' not in done:
            done['x'] = 1
            for i in range(3):
                if world.occ['neutral'][i] == 'blue':
                    world.occ['neutral'][i] = None
    m, _ = run_mission(w, 'sign', 'pick', p0=P_SOURCE, steps=12000, hook=hook)
    assert any('yeniden deneme 1' in e[2] for e in m.log), m.log
    assert m.state == 'DONE' and m.bay.parcel == 'green', m.log[-6:]


def test_fork_stuck_aborts_backs_out_and_faults():
    w = _source_world()
    m, p = run_mission(w, 'sign', 'pick', p0=P_SOURCE, steps=8000, fork_stuck=True)
    states = [e[1] for e in m.log]
    assert 'ABORT' in states and m.state == 'FAULT' and 'çatal takıldı' in m.fault, m.log[-6:]
    assert p.x < 1.5 - ROBOT['stop_distance'] - 0.4                  # bölmeden geri çıktı


def test_wrong_station_is_excluded_and_search_continues():
    """Uzaktan mavi istasyonun tabelası 'red' okunuyor; yakında mavi görülünce o nokta dışlanır, yeniden
    aranır ve gerçek kırmızı bulunur (plan §7.2)."""
    w = FakeWorld({'neutral': [None] * 3, 'red': [None] * 3, 'green': [None] * 3, 'blue': [None] * 3})
    w.far_label = {'blue': 'red'}
    m, p = run_mission(w, 'sign', 'transit', 'red', p0=SE2(0.5, 0.0, math.pi), steps=20000)   # maviye bakıyor
    assert any('yanlış istasyon: önde blue' in e[2] for e in m.log), [e for e in m.log if e[1] != 'FIND_STATION']
    x, y, a = STATIONS['red']
    rel = SE2(x, y, a).inv() * p
    assert m.state == 'DONE' and abs(rel.y) < 0.1 and abs(rel.x + 0.8) < 0.1, (m.log[-6:], rel)


def test_wrong_station_without_target_in_range_faults():
    """Gerçek hedef görüş menzilinde değilse robot dışladığı istasyona geri dönmez, arar ve FAULT verir."""
    w = FakeWorld({'neutral': [None] * 3, 'red': [None] * 3, 'green': [None] * 3, 'blue': [None] * 3})
    w.far_label = {'green': 'red'}
    m, p = run_mission(w, 'sign', 'transit', 'red', p0=SE2(0.0, -0.1, -math.pi / 2), steps=20000)
    assert any('yanlış istasyon: önde green' in e[2] for e in m.log)
    assert m.state == 'FAULT' and 'bulunamadı' in m.fault and m.last_cmd == (0.0, 0.0)


@pytest.mark.parametrize('method', ['sign', 'lane'])
def test_swapped_stations_full_mission(method):
    """A5: kırmızı ve mavi istasyonların yeri değişmiş; factory'de zemin şeritleri eski düzende boyalı.
    Koliler yine renklerine göre doğru istasyona gider (şerit tabelaya gitmiyorsa tabelayla)."""
    w = _source_world()
    w.st['red'], w.st['blue'] = STATIONS['blue'], STATIONS['red']
    m, _ = run_mission(w, method)
    assert m.state == 'DONE' and m.parcels_done == 3, m.log[-6:]
    for c in ('red', 'green', 'blue'):
        assert w.occ[c] == [None, c, None], (c, w.occ[c])


def test_lane_not_leading_to_sign_is_rejected():
    """Şerit görünüyor ama doğrusu hedef tabelaya 2 m uzaktan geçiyor: şerit kullanılmaz, tabelayla gidilir."""
    tr, sg, ln = BayTracker(CFG['tracker']), SignTracker(CFG['tracker']), LaneTracker(CFG['tracker'])
    m = Mission(CFG['mission'], ROBOT, tr, sg, ln, 'lane')
    m.start(0.0, 'transit', 'red')
    p = SE2(0.0, 0.0, math.pi / 2)
    m._go('LANE_JOIN', 0.0)
    ln.reset('red')
    for k in range(60):
        t = k * DT
        sg.add(t, 'red', (-2.0, 2.0), math.pi / 2, 2.8)                   # tabela sol önde
        ln.add(t, [(0.0, 0.6 + 0.02 * i) for i in range(40)], None, [0.6 + 0.02 * i for i in range(40)])
        m.step(t, p, 0.0, False)
        if m.state != 'LANE_JOIN':
            break
    assert m.state == 'TRANSIT' and 'tabelaya gitmiyor' in m.log[-1][2], m.log[-3:]


def test_transit_started_with_parcel_monitors_load():
    """A5 E6 bulgusu: transit görevi koli çataldayken başlatılırsa yük ilk ölçümlerden anlaşılır ve izlenir."""
    w = _source_world()
    w.carry = 'red'

    def hook(m, world, t):
        if m.state == 'TRANSIT' and t > 8.0:
            world.carry = None
    m, _ = run_mission(w, 'sign', 'transit', 'red', p0=SE2(0.75, 0.0, 0.0), steps=12000, hook=hook)
    assert m.state == 'FAULT' and 'yük kaybı' in m.fault, m.log[-5:]
