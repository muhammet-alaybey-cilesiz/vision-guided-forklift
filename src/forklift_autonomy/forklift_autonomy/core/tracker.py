"""
tracker.py — bölme girişlerini ve kolileri odometri çerçevesinde izler

Algılama her karede bölme girişlerini (hedef çifti ya da bölme etiketi) ve kolileri
robota göre verir. Robot hareket ettiği için bu ölçümler görüntünün çekildiği andaki
odometri pozuyla (TF) odometri çerçevesine taşınır; orada sabit dururlar. Aynı bölmenin
ölçümleri eşleştirilip üstel ortalamayla süzülür. Böylece:
 * bir karede kaybolan bölme hemen unutulmaz (C4: kare başına %7 kayıp),
 * hedefler görünmezken (son yanaşmanın kör yolu) odometri araya girer.
Yaw yalnız hedef çiftinden (ya da en az iki etiketten) güncellenir; tek etiket yalnız konum verir.

Transit için (F4) iki izleyici daha: tabela (istasyon kimliği ve kaba pozu, D-66) ve renkli zemin
şeridi (doğru + durma çizgisi). Hepsi aynı ilke: ölçüm görüntü anındaki poza göre odometriye taşınır.
"""

import math
from dataclasses import dataclass, field

import numpy as np

from forklift_autonomy.core.se2 import wrap


@dataclass(eq=False)                     # izler kimlikle karşılaştırılır
class BayTrack:
    entry: np.ndarray             # ön çizgi üzerinde bölme girişi (odom)
    theta: float                  # bölme ekseni (+ istasyonun içine), odom
    n_obs: int = 0
    n_pair: int = 0
    last_t: float = 0.0
    parcel: str = ''              # son görülen mühür rengi ('' = boş)
    parcel_t: float = -1e9
    empty_votes: int = 0          # koli görülmeden ön kameranın görüş alanında geçen kare sayısı
    created_t: float = 0.0
    extra: dict = field(default_factory=dict)

    @property
    def axis(self):
        return np.array([math.cos(self.theta), math.sin(self.theta)])


class BayTracker:
    def __init__(self, cfg):
        self.cfg = cfg
        self.bays = []
        self.orphans = []                 # hiçbir bölme izine bağlanmayan koli ölçümleri (t, xy, renk)

    def reset(self):
        self.bays = []
        self.orphans = []

    def add_entries(self, t, entries):
        """entries: [(entry_xy_odom, theta_odom, yaw_valid, source, anchor_xy_odom)], source: 'pair' | 'label'.

        Etiket: girişi kendi (iki etiketten, ±2°'ye kadar hatalı) yönüyle değil izlenen eksen yönüyle
        yeniden kurar: giriş = etiket merkezi − eksen · |etiket − giriş|. Hedef çifti yakın zamanda
        görüldüyse etiket yalnız eksen boyunca (mesafe) günceller; yanal konum çiftlerden kalır."""
        c = self.cfg
        for xy, th, yaw_valid, src, anchor in entries:
            xy = np.asarray(xy, float)
            best = min(self.bays, key=lambda b: np.linalg.norm(b.entry - xy), default=None)
            if best is None or np.linalg.norm(best.entry - xy) > c['assoc_dist']:
                if src != 'pair':
                    continue                         # yeni bölme yalnız hedef çiftinden açılır
                self.bays.append(BayTrack(xy.copy(), th, 1, 1, t, created_t=t, extra={'pair_t': t}))
                continue
            a = c['alpha_pair'] if src == 'pair' else c['alpha_label']
            if src == 'label' and anchor is not None:
                anchor = np.asarray(anchor, float)
                n = best.axis
                xy = anchor - n * float(np.linalg.norm(anchor - np.asarray(xy, float)))
                if t - best.extra.get('pair_t', -1e9) < c['pair_recent_s']:
                    # yalnız eksen boyunca: yanal bileşen izlenen değerde kalır
                    xy = best.entry + n * float(np.dot(xy - best.entry, n))
            best.entry = (1 - a) * best.entry + a * xy
            if src == 'pair':
                best.extra['pair_t'] = t
            if yaw_valid and src == 'pair':
                best.theta = wrap(best.theta + a * wrap(th - best.theta))
                best.n_pair += 1
            best.n_obs += 1
            best.last_t = t

    def add_parcels(self, t, parcels, view=None):
        """parcels: [(position_xy_odom, seal_color)] — koli ön yüzünün ortası.

        Bütün izlere uygulanır (doğrulanmamışlara da): bölme doğrulanmadan önce görülen koli
        unutulmasın. view = (kamera_xy, kamera_yaw, yarım_açı, en_az, en_çok) verilirse "boş" oyu
        yalnız bölme ön kameranın görüş alanındayken sayılır (görülmeyen bölme boş sayılmaz)."""
        c = self.cfg
        used = set()
        for b in self.bays:
            n = b.axis
            tv = np.array([-n[1], n[0]])
            hit = None
            for xy, color in parcels:
                d = np.asarray(xy) - b.entry
                if abs(np.dot(d, tv)) < c['parcel_lateral'] and -c['parcel_along'] < np.dot(d, n) < c['parcel_along']:
                    hit = color
                    used.add(id(xy))
            if hit:
                b.parcel, b.parcel_t, b.empty_votes = hit, t, 0
            elif view is None or self._in_view(b.entry, view):
                b.empty_votes += 1
        self.orphans = [o for o in self.orphans if t - o[0] < c['parcel_memory_s']]
        self.orphans += [(t, np.asarray(xy, float), color) for xy, color in parcels if id(xy) not in used]

    def orphan_parcels(self, now):
        """Bölmesi izlenmeyen, yakın zamanda görülmüş koliler (bölmenin hedef çifti algılanmamış olabilir)."""
        c = self.cfg
        out = []
        for t, xy, color in self.orphans:
            if now - t >= c['parcel_memory_s']:
                continue
            if any(np.linalg.norm(xy - b.entry) < c['parcel_along'] + c['parcel_lateral'] for b in self.bays):
                continue
            if all(np.linalg.norm(xy - o[1]) > 0.05 for o in out):
                out.append((t, xy, color))
        return out

    @staticmethod
    def _in_view(xy, view):
        cam, yaw, half, rmin, rmax = view
        d = np.asarray(xy) - np.asarray(cam)
        r = float(np.linalg.norm(d))
        return rmin <= r <= rmax and abs(wrap(math.atan2(d[1], d[0]) - yaw)) <= half

    def confirmed(self):
        return [b for b in self.bays if b.n_pair >= self.cfg['min_pair_obs']]

    def occupied(self, b, now):
        return b.parcel != '' and now - b.parcel_t < self.cfg['parcel_memory_s']

    def empty(self, b, now):
        return not self.occupied(b, now) and b.empty_votes >= self.cfg['min_empty_votes']

    def settled(self, now):
        """İzler kararlı mı: son stable_s içinde yeni iz açılmadı ve yakında görülen her iz doğrulandı.
        F3 A1'de seçim 1.6 s'de, bölmelerin bir kısmı henüz doğrulanmadan yapılıyordu (yanlış bölme).
        Doluluğun belirsizliği seçimde ele alınır (Mission._s_select_bay): yalnız kararı değiştirebilecek
        bölme beklenir."""
        c = self.cfg
        if not self.confirmed() or any(now - b.created_t < c['stable_s'] for b in self.bays):
            return False
        return not any(b.n_pair < c['min_pair_obs'] and now - b.last_t < c['stable_s'] for b in self.bays)

    def summary(self, now):
        """Log için: her iz (çift sayısı, boş oyu, doluluk, yaş)."""
        return '; '.join(f'[{b.entry[0]:.2f},{b.entry[1]:.2f}] çift {b.n_pair} boş {b.empty_votes} '
                         f'koli {b.parcel or "-"} yaş {now - b.created_t:.1f} s son {now - b.last_t:.1f} s' for b in self.bays)

    def left_to_right(self, bays):
        """Robotun istasyona bakışıyla soldan sağa (ortak eksen yönüne dik)."""
        if not bays:
            return []
        n = np.mean([b.axis for b in bays], axis=0)
        left = np.array([-n[1], n[0]])
        return sorted(bays, key=lambda b: -float(np.dot(b.entry, left)))

    def nearest(self, xy):
        return min(self.bays, key=lambda b: np.linalg.norm(b.entry - xy), default=None)


# =============================================================================
#  Transit (F4)
# =============================================================================
class SignTracker:
    """Renge göre istasyon tabelası: odometride ön kenar ortası ve istasyon ekseni (+x içeri).

    Tabela yaw'ı uzakta kabadır (2.5 m'de ~4°/px, D-66); konum yakında iyileşir. Kestirim son
    window_s içindeki ölçümlerin 1/r² ağırlıklı ortalamasıdır: robot yaklaştıkça yakın ölçümler baskın."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.obs = {}

    def reset(self):
        self.obs = {}

    def add(self, t, color, xy, yaw, rng):
        self.obs.setdefault(color, []).append((t, np.asarray(xy, float), float(yaw), max(float(rng), 0.3)))
        self.obs[color] = [o for o in self.obs[color] if t - o[0] < self.cfg['sign_window_s']]

    def counts(self, now):
        """Log için: son pencerede renge göre ölçüm sayısı."""
        return {k: sum(1 for x in v if now - x[0] < self.cfg['sign_window_s']) for k, v in self.obs.items() if v}

    def estimate(self, color, now, since=-1e9):
        """→ (xy, yaw, n, range_min) | None. since: yalnız bu andan sonraki ölçümler (dururken bakış)."""
        o = [x for x in self.obs.get(color, []) if now - x[0] < self.cfg['sign_window_s'] and x[0] >= since]
        if len(o) < self.cfg['sign_min_obs']:
            return None
        w = np.array([1.0 / x[3] ** 2 for x in o])
        # aykırı (başka nesne ya da ikinci tespit): ortancaya uzak olanları at. Ortalama değil ortanca:
        # eğik bakışta dağınık birkaç ölçüm ortalamayı kaydırıp bütün ölçümleri kapı dışına itiyordu.
        med = np.median(np.array([x[1] for x in o]), axis=0)
        keep = [i for i, x in enumerate(o) if np.linalg.norm(x[1] - med) < self.cfg['sign_gate']]
        if len(keep) < self.cfg['sign_min_obs']:
            return None
        o, w = [o[i] for i in keep], w[keep]
        xy = (w[:, None] * np.array([x[1] for x in o])).sum(0) / w.sum()
        yaw = math.atan2((w * np.sin([x[2] for x in o])).sum(), (w * np.cos([x[2] for x in o])).sum())
        return xy, yaw, len(o), min(x[3] for x in o)


class LaneTracker:
    """Tek renkli düz zemin şeridi: odometride doğru (PCA) ve durma çizgisi.

    Noktalar cell boyutlu ızgarada tekilleştirilir (hücre başına en yeni nokta): robot dururken aynı
    kısa parça tekrar tekrar eklense de doğru, o ana kadar görülen şeridin tamamından uydurulur.
    Plan §1: viraj kapsam dışı; mesaj çoklu çizgi olduğu için ileride doğru yerine eğri uydurulur."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.reset()

    def reset(self, color=None):
        self.color, self.cells, self.bar, self.bar_t, self.last_t = color, {}, None, -1e9, -1e9
        self._line = None
        self.hint = None                  # hedef tabelanın konumu (odom): şerit istasyona gider

    def add(self, t, pts_odom, bar_odom=None, ranges=None):
        """ranges: noktaların robota uzaklığı. Takipte lane_fit_range'den uzak noktalar (eğik bakış,
        büyük izdüşüm hatası) kullanılmaz; şeride girişte (henüz yakın nokta yokken) kullanılır."""
        c = self.cfg
        if ranges is None:
            ranges = [0.0] * len(pts_odom)
        for p, r in zip(pts_odom, ranges):
            p = np.asarray(p, float)
            key = (int(math.floor(p[0] / c['lane_cell'])), int(math.floor(p[1] / c['lane_cell'])))
            old = self.cells.get(key)
            if old is None or r <= c['lane_fit_range'] or old[2] > c['lane_fit_range']:
                self.cells[key] = (t, p, float(r))
        self.cells = {k: v for k, v in self.cells.items() if t - v[0] < c['lane_window_s']}
        if len(pts_odom):
            self.last_t = t
            self._line = None
        if bar_odom is not None:
            bar_odom = np.asarray(bar_odom, float)
            if self.bar is None or np.linalg.norm(bar_odom - self.bar) > c['bar_gate']:
                self.bar = bar_odom.copy()
            else:
                self.bar = (1 - c['alpha_bar']) * self.bar + c['alpha_bar'] * bar_odom
            self.bar_t = t

    def add_candidates(self, t, cands, heading):
        """Bir karedeki aynı renkli şerit adayları: [(noktalar_odom, uzaklıklar, durma_çizgisi | None)].

        Önlükteki bölme şeritleri ve çevre bandının kenarları da aynı renkte ve bazen "zemin" sınıfında
        gelir (G7). İlişkilendirme: izlenen doğru varsa ona lane_gate'ten yakın aday alınır (en yakını);
        yoksa doğrusu hedef tabelaya (hint) en yakın geçen aday, hint de yoksa en yakın ve en uzun."""
        c = self.cfg
        ln = self.line(heading)
        best = None
        for pts, rng, bar in cands:
            P = np.asarray(pts, float)
            if ln is not None:
                m, d = ln
                n = np.array([-d[1], d[0]])
                score = float(np.median(np.abs((P - m) @ n)))
                if score > c['lane_gate']:
                    continue
            elif self.hint is not None:
                f = self._fit(list(P)) if len(P) >= c['lane_min_pts'] else None
                if f is None:
                    continue
                m, d = f
                score = abs(float(np.dot(np.asarray(self.hint) - m, [-d[1], d[0]])))
            else:
                score = min(rng) - 0.1 * len(pts) * c['lane_cell']
            if best is None or score < best[0]:
                best = (score, pts, rng, bar)
        if best is not None:
            self.add(t, best[1], best[3], best[2])

    def line(self, heading_hint, recent_s=None):
        """→ (nokta, birim yön) | None; yön heading_hint'e yakın tarafa çevrilir.

        recent_s: yalnız son recent_s saniyede görülen noktalar (takip: odometri kayması eski noktaları
        eğer); yeterli taze nokta yoksa (şerit görüşten çıktı) bütün bellek kullanılır."""
        fr = self.cfg['lane_fit_range']
        fit = None
        if recent_s is not None:
            fit = self._fit([v[1] for v in self.cells.values() if self.last_t - v[0] <= recent_s and v[2] <= fr])
        if fit is None:
            if self._line is None:
                near = [v[1] for v in self.cells.values() if v[2] <= fr]
                self._line = self._fit(near) or self._fit([v[1] for v in self.cells.values()])
            fit = self._line
        if fit is None:
            return None
        m, d = fit
        if np.dot(d, [math.cos(heading_hint), math.sin(heading_hint)]) < 0:
            d = -d
        return m, d

    def _fit(self, pts):
        if len(pts) < self.cfg['lane_min_pts']:
            return None
        P = np.array(pts)
        m = P.mean(0)
        _, _, vt = np.linalg.svd(P - m, full_matrices=False)
        d = vt[0]
        if float(np.ptp((P - m) @ d)) < self.cfg['lane_min_len']:
            return None
        return m, d
