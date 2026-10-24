"""
mission.py — görev durum makinesi (plan §7.1): istasyon arama, transit, yanaşma, alma ve bırakma

ROS'tan bağımsız: düğüm her döngüde step(now, pose, q, load) çağırır, izleyicileri ölçümlerle besler.
Çıktı: (v, w, fork_target | None). Durum adları ve açıklamalar GUI'de gösterilir (Türkçe).

Görevler (task):
  pick      bulunduğu istasyondan bir koli al (A1)
  drop      taşınan koliyi bulunduğu istasyonun boş bir bölmesine bırak (A2)
  transit   target renkli istasyonun ön-yanaşma pozuna git (A3)
  full      kaynak boşalana kadar: kaynağa git → al → mühür rengindeki istasyona git → bırak (A4)

Akış:
  FIND_STATION  yerinde adım adım dön, istenen renkte tabela ara (dur-bak-dön)
  TRANSIT       tabela yöntemi (K4): tabelaya doğru en çok max_leg'lik ayaklar, her ayaktan sonra dur ve
                yeniden ölç; near_range'e girince istasyon ekseninde predock_distance'a dön-git-dön
  LANE_JOIN     şerit yöntemi (factory): şeridi gör, doğru uydur, şeridin üstüne çık
  LANE_FOLLOW   şerit ekseni servosu; durma çizgisinin lane_stop_offset önünde dur
  PRE_DOCK      inceltme: hedef çiftlerinden istasyon ekseni (tabela yaw'ı kaba, D-66), gerekirse
                ön-yanaşma pozuna kısa bir dön-git-dön
  OBSERVE       dur, bölmeleri ve kolileri izle; gözlem tamamlanınca (tracker.settled)
  SELECT_BAY    alma: soldan sağa ilk dolu bölme; bırakma: önce orta, sonra soldan sağa boş bölme (K10)
  REPOSITION    koli görülüyor ama bölmesinin hedef çifti algılanmamış: kolinin önüne geç, yeniden gözle
  ALIGN         ön-giriş noktasına dön-git-dön (bölme ekseninde, align_distance)
  SETTLE        dur, ölçümleri tazele; hizalama yetersizse ALIGN'a dön (en çok align_retries)
  ENTER / TRIM  bölme ekseni servosu; çatal uçları ön çizgiye gelmeden bir kez yerinde son yön
                hizalaması; durma kararı CV mesafesiyle (D-62)
  LIFT / LOWER  çatal taşıma yüksekliğine / sıfıra
  BACK_OUT      düz geri
  VERIFY        yük denetimi: alma sonrası koli çatalda, bırakma sonrası çatal boş
  DONE / FAULT

Hata ve kurtarma (plan §7.2, F5):
  RECOVER       hedef kayıp ya da zaman aşımı: dur, bekle, kısa geri, yeniden gözle / yeniden ara;
                bacak başına recover_max denemeden sonra FAULT
  ABORT         çatal takıldı: taşıma yüksekliğine çık, geri çık, FAULT
  yük kaybı     taşırken yük denetimi koli görmüyor: dur, çatalı tut, FAULT
  alma başarısız VERIFY'da koli yok: çatalı indir, pick_retries kez yeniden dene, sonra FAULT
  yanlış istasyon PRE_DOCK'ta en yakın tabela başka renk: o noktayı dışla, yeniden ara
"""

import math

import numpy as np

from forklift_autonomy.core.control import BayServo, GoTo, Straight, Turn
from forklift_autonomy.core.se2 import wrap

DETAIL = {
    'IDLE': 'Hazır', 'FIND_STATION': 'İstasyon aranıyor', 'TRANSIT': 'İstasyona gidiliyor',
    'LANE_JOIN': 'Şeride giriliyor', 'LANE_FOLLOW': 'Şerit izleniyor', 'PRE_DOCK': 'Ön-yanaşma',
    'OBSERVE': 'İstasyon gözleniyor', 'SELECT_BAY': 'Bölme seçiliyor',
    'ALIGN': 'Bölme eksenine hizalanıyor', 'SETTLE': 'Ölçüm tazeleniyor', 'ENTER': 'Bölmeye giriliyor',
    'TRIM': 'Son hizalama', 'REPOSITION': 'Yeniden konumlanma', 'RECOVER': 'Kurtarma', 'ABORT': 'Güvenli çıkış', 'LIFT': 'Koli kaldırılıyor', 'LOWER': 'Koli bırakılıyor',
    'BACK_OUT': 'Geri çıkılıyor', 'VERIFY': 'Yük denetleniyor',
    'DONE': 'Görev tamam', 'FAULT': 'Hata', 'PAUSED': 'Duraklatıldı',
}
SOURCE_COLOR = 'neutral'
# zaman aşımında kurtarma denenen durumlar (diğerlerinde doğrudan FAULT; LIFT/LOWER: çatal takıldı)
RECOVERABLE = ('TRANSIT', 'LANE_JOIN', 'LANE_FOLLOW', 'PRE_DOCK', 'OBSERVE', 'SELECT_BAY', 'REPOSITION', 'ALIGN',
               'SETTLE', 'ENTER', 'TRIM', 'BACK_OUT')
TASKS = ('pick', 'drop', 'transit', 'full')


class Mission:
    def __init__(self, cfg, robot, tracker, signs, lane, method):
        self.c, self.robot, self.tr, self.signs, self.lane = cfg, robot, tracker, signs, lane
        self.method = method                            # 'sign' | 'lane'
        self.task = 'full'
        self.reset()

    # ------------------------------------------------------------------ durum
    def reset(self):
        self.state, self.t_state, self.fault = 'IDLE', 0.0, ''
        self.bay, self.prim, self.servo = None, None, None
        self.fork_target = None
        self.align_tries = 0
        self.trimmed = False
        self.load_hist = []
        self.log = []                                   # (zaman, olay, sebep) — değerlendirme ve GUI kaydı
        self.goal = None                                # ('go', renk) | ('pick',) | ('drop',)
        self.target = ''                                # gidilen istasyonun rengi
        self.loaded = False
        self.parcels_done = 0
        self.parcels_total = 0                          # tahmin: bırakılan + kaynakta son görülen dolu bölme
        self.look_t = None
        self.reobserve = 0
        self.sign_xy = None
        self.sign_est = None
        self.sign_memory = {}
        self.recovers = 0                               # bacak içindeki kurtarma sayısı
        self.pick_retries = 0
        self.excluded = []                              # yanlış istasyon çıkan tabela konumları (odom)
        self.abort_why = ''
        self.source_left = None                         # son alımda kaynakta kalan koli (bilinmiyorsa None)                           # görev içi: görülen tabelalar (odom), ipucu
        self.hinted = False
        self.misses = 0
        self.lane_tries = 0
        self.pd_moves = 0
        self.search_steps = 0
        self.search_turns = 0
        self.legs = 0
        self.done_why = ''

    def start(self, now, task='full', target='', loaded=None):
        if task not in TASKS:
            raise ValueError(f'bilinmeyen görev: {task}')
        self.reset()
        self.task = task
        self.loaded = (task == 'drop') if loaded is None else bool(loaded)
        # transit: operatör koli çataldayken de başlatabilir; "taşıyor muyum" ilk yük ölçümlerinden (A5 E6 bulgusu)
        self.load_known = task != 'transit' or loaded is not None
        self.fork_target = self.c['carry_q'] if self.loaded else None
        self.parcels_total = 1 if task in ('pick', 'drop') else 0
        if task == 'pick':
            self._begin(now, ('pick',))
        elif task == 'drop':
            self._begin(now, ('drop',))
        elif task == 'transit':
            self._begin(now, ('go', target))
        else:
            self._begin(now, ('go', SOURCE_COLOR))

    def _begin(self, now, goal, why=''):
        self.goal = goal
        if goal[0] == 'go':
            self.target, self.search_steps, self.legs, self.look_t, self.prim = goal[1], 0, 0, None, None
            self.recovers = 0
            self.search_turns = 0
            self.sign_est, self.misses, self.lane_tries, self.hinted = None, 0, 0, False
            self.signs.reset()
            self._go('FIND_STATION', now, why or f'hedef: {goal[1]}')
        else:
            self.tr.reset()
            self.bay, self.align_tries, self.trimmed, self.prim = None, 0, False, None
            self.recovers = 0
            self._go('OBSERVE', now, why)

    def _leg_done(self, now, why):
        """Bir bacak bitti: görev türüne göre sıradaki bacak ya da DONE."""
        g = self.goal[0]
        if self.task != 'full':
            self._finish(now, why)
        elif g == 'go':
            self._begin(now, ('drop',) if self.loaded else ('pick',), why)
        elif g == 'pick':
            self._begin(now, ('go', self.bay.parcel), why)
        else:
            self.parcels_done += 1
            if self.c['finish_when_source_seen_empty'] and self.source_left == 0:
                # son alımda kaynakta başka dolu ya da belirsiz bölme, sahipsiz koli görülmedi (plan §7.1: … | DONE)
                self._finish(now, f'{why}; kaynakta koli kalmadı (son alımda görüldü)')
            else:
                self._begin(now, ('go', SOURCE_COLOR), why)

    def _finish(self, now, why):
        self.done_why = why
        self._go('DONE', now, why)

    def _go(self, state, now, why=''):
        self.state, self.t_state = state, now
        self.log.append((now, state, why))

    def _fail(self, now, why):
        self.fault = why
        self._go('FAULT', now, why)

    @property
    def detail(self):
        d = DETAIL.get(self.state, self.state)
        if self.state == 'FAULT':
            return f'{d}: {self.fault}'
        if self.state == 'DONE' and self.done_why:
            return f'{d} ({self.done_why})'
        return d

    @property
    def active(self):
        return self.state not in ('IDLE', 'DONE', 'FAULT')

    @property
    def target_color(self):
        """GUI: gidilen istasyonun rengi; istasyondayken alınacak / taşınan kolinin mühür rengi."""
        if self.goal is not None and self.goal[0] == 'go':
            return self.target
        return self.bay.parcel if self.bay is not None and self.bay.parcel else self.target

    def resume_state(self, paused_from):
        """Duraklatmadan dönüş (plan §7.1): yanaşmanın ortasından ise gözlemden yeniden başla."""
        if paused_from in ('ALIGN', 'SETTLE', 'ENTER', 'TRIM', 'REPOSITION', 'RECOVER'):
            self.tr.reset()
            self.bay, self.align_tries, self.trimmed, self.prim = None, 0, False, None
            return 'OBSERVE'
        if paused_from in ('FIND_STATION', 'TRANSIT', 'LANE_JOIN', 'LANE_FOLLOW', 'PRE_DOCK'):
            self.prim, self.look_t = None, None
        return paused_from

    # ------------------------------------------------------------------ döngü
    def step(self, now, pose, q, load):
        """→ (v, w, fork_target | None). load: son yük ölçümü (True/False/None)."""
        c = self.c
        if load is not None:
            self.load_hist = (self.load_hist + [bool(load)])[-c['verify_window']:]
        if not self.load_known and len(self.load_hist) >= c['verify_window']:
            self.load_known = True
            self.loaded = float(np.mean(self.load_hist)) >= c['verify_ratio']
            if self.loaded:
                self.log.append((now, self.state, 'çatalda koli var (yük denetimi); taşıma kuralları geçerli'))
        el = now - self.t_state
        timeout = c['timeouts'].get(self.state)
        if timeout is not None and el > timeout:
            why = f'zaman aşımı ({self.state}, {timeout:.0f} s)'
            if self.state in ('LIFT', 'LOWER'):
                self._abort(now, f'çatal takıldı (q {q if q is None else round(q * 1000, 1)} mm, hedef '
                                 f'{self.fork_target * 1000:.0f} mm)')
            elif self.state in RECOVERABLE:
                self._recover(now, why)
            else:
                self._fail(now, why)
        elif self._load_lost():
            self.fork_target = q if q is not None else self.fork_target      # çatalı olduğu yerde tut
            self._fail(now, 'yük kaybı: taşınan koli çatalda görülmüyor')
        fn = getattr(self, f'_s_{self.state.lower()}', None)
        if fn is None:
            return 0.0, 0.0, self.fork_target
        v, w = fn(now, pose, q, el)
        return v, w, self.fork_target

    def _load_lost(self):
        """Taşırken (bırakma için indirmeye başlamadan önce) yük denetimi koliyi görmüyor mu."""
        c = self.c
        if not self.loaded or self.state in ('LOWER', 'BACK_OUT', 'VERIFY', 'DONE', 'FAULT', 'ABORT', 'IDLE'):
            return False
        h = self.load_hist
        return len(h) >= c['verify_window'] and 1.0 - float(np.mean(h)) >= c['load_loss_ratio']

    def _recover(self, now, why):
        """Plan §7.2: dur → bekle → kısa geri → yeniden gözle (istasyonda) ya da yeniden ara (transitte)."""
        if self.recovers >= self.c['recover_max']:
            self._fail(now, f'{why}; {self.recovers} kurtarma denemesi tükendi')
            return
        self.recovers += 1
        self.prim, self.recover_why = None, why
        self._go('RECOVER', now, f'{why} → kurtarma {self.recovers}/{self.c["recover_max"]}')

    def _s_recover(self, now, pose, q, el):
        c = self.c
        if el < c['recover_wait_s']:
            return 0.0, 0.0
        if self.prim is None:
            self.prim = Straight(pose, -c['recover_back'], c['v_back'], self.c['motion'])
        v, w, done = self._run_prim(pose)
        if not done:
            return v, w
        if self.goal[0] == 'go':
            self.search_steps, self.look_t, self.legs, self.misses = 0, None, 0, 0
            self._go('FIND_STATION', now, 'kurtarma: yeniden aranıyor')
        else:
            self.tr.reset()
            self.bay, self.align_tries, self.trimmed = None, 0, False
            self._go('OBSERVE', now, 'kurtarma: yeniden gözleniyor')
        return 0.0, 0.0

    def _abort(self, now, why):
        """Çatal takıldı: taşıma yüksekliğine çık (yük varsa kolisini kurtar), geri çık, FAULT."""
        self.abort_why, self.prim = why, None
        self.fork_target = self.c['carry_q']
        self._go('ABORT', now, why)

    def _s_abort(self, now, pose, q, el):
        c = self.c
        if self.prim is None:
            if el < c['abort_fork_wait_s'] and (q is None or abs(q - c['carry_q']) > c['fork_tol']):
                return 0.0, 0.0                             # çatal taşıma yüksekliğine (ya da süre dolana kadar)
            self.prim = Straight(pose, -c['back_out'], c['v_back'], self.c['motion'])
        v, w, done = self._run_prim(pose)
        if done:
            self._fail(now, self.abort_why)
            return 0.0, 0.0
        return v, w

    def _run_prim(self, pose):
        v, w, done = self.prim.step(pose)
        if done:
            self.prim = None
        return v, w, done

    # ---------------------------------------------------------------- transit
    def _look(self, now, any_time=False):
        """Dur ve bak: look_s dolunca tabela kestirimini döndürür (yoksa None); bakış sürerken 'wait'.
        any_time: pencere içindeki bütün ölçümler (aramada yavaş dönüş anları da; düğüm hızlı dönüşte
        ölçüm almaz). Gürültüsüz render'da nötr tabela bazı açılarda hiç algılanmıyor, aradakilerde
        algılanıyor; duruştaki tek açıya bağlı kalınmaz.
        Kestirim bulunursa odometride saklanır (istasyon sabit): sonraki boş bakış onu kullanır."""
        t = self.c['transit']
        if self.look_t is None:
            self.look_t = now
        if now - self.look_t < t['look_s']:
            return 'wait'
        since = -1e9 if any_time else self.look_t + t['look_skip_s']
        est = self.signs.estimate(self.target, now, since=since)
        if est is None:
            o = [x for x in self.signs.obs.get(self.target, [])
                 if x[0] >= since and now - x[0] < self.signs.cfg['sign_window_s']]
            if o:
                P = np.array([x[1] for x in o])
                self.log.append((now, self.state, f'{self.target} tabelası: {len(o)} ölçüm, kestirim yok '
                                                  f'(yayılım {np.ptp(P, axis=0).round(2).tolist()} m)'))
        self.look_t = None
        if est is not None and self._excluded(est[0]):
            est = None                                      # yanlış istasyon çıkmıştı
        if est is not None:
            self.sign_est = est
        # bakışta görülen bütün tabelalar hatırlanır: istasyonlar sabit, sonraki aramada ipucu olur
        for color in list(self.signs.obs):
            e = self.signs.estimate(color, now)
            if e is not None:
                self.sign_memory[color] = e[0]
        return est

    def _s_find_station(self, now, pose, q, el):
        t = self.c['transit']
        if self.prim is not None:
            v, w, _ = self._run_prim(pose)
            return v, w
        if not self.hinted and self.target in self.sign_memory:
            # görev içinde daha önce görüldü: önce o yöne dön (yalnız ipucu; görülmezse normal arama)
            self.hinted = True
            d = self.sign_memory[self.target] - pose.xy
            b = math.atan2(d[1], d[0])
            if abs(wrap(b - pose.yaw)) > math.radians(t['hint_tol_deg']):
                self.prim = Turn(b, self.c['motion'])
                self.log.append((now, 'FIND_STATION', f'hafızadaki {self.target} tabelasına dön ({math.degrees(wrap(b - pose.yaw)):.0f}°)'))
                return 0.0, 0.0
        est = self._look(now, any_time=True)
        if est == 'wait':
            return 0.0, 0.0
        if est is not None:
            xy, yaw, n, rmin = est
            self.sign_xy = xy
            r = float(np.linalg.norm(xy - pose.xy))
            nxt = 'LANE_JOIN' if self.method == 'lane' else 'TRANSIT'
            if nxt == 'LANE_JOIN':
                self.lane.reset(self._lane_color())
                self.lane.hint = xy
            self._go(nxt, now, f'tabela {self.target}: {r:.2f} m, yön {math.degrees(wrap(math.atan2(*(xy - pose.xy)[::-1]) - pose.yaw)):.0f}°')
            return 0.0, 0.0
        self.search_steps += 1
        self.log.append((now, 'FIND_STATION', f'bakış {self.search_steps}: yön {math.degrees(pose.yaw):.0f}°, '
                                               f'görülen {self.signs.counts(now)}'))
        if self.search_steps > t['search_steps']:
            if self.search_turns < t['search_turns']:
                # ikinci tur yarım adım kaydırılmış: tek bir duruş açısında görülmeyen tabela arada görülür
                self.search_turns += 1
                self.search_steps = 0
                self.prim = Turn(wrap(pose.yaw + math.radians(t['search_step_deg'] / 2.0)), self.c['motion'])
                self.log.append((now, 'FIND_STATION', f'tam tur boş, {self.search_turns + 1}. tur yarım adım kaydırılmış'))
                return 0.0, 0.0
            self._fail(now, f'{self.target} tabelası bulunamadı ({self.search_turns + 1} tur)')
            return 0.0, 0.0
        self.prim = Turn(wrap(pose.yaw + math.radians(t['search_step_deg'])), self.c['motion'])
        return 0.0, 0.0

    def _s_transit(self, now, pose, q, el):
        t = self.c['transit']
        if self.prim is not None:
            v, w, done = self._run_prim(pose)
            if done and self.legs < 0:                      # son ayak (ön-yanaşma) bitti
                self._arrived(now, pose)
            elif done and self.method == 'lane' and self.lane_tries < t['lane_retries'] and self.sign_est is not None:
                self.lane_tries += 1                        # bir ayak yaklaştı: şeride yeniden dene
                self.lane.reset(self._lane_color())
                self.lane.hint = self.sign_est[0]
                self._go('LANE_JOIN', now, f'şeride yeniden giriş ({self.lane_tries})')
            return v, w
        est = self._look(now)
        if est == 'wait':
            return 0.0, 0.0
        if est is None and self.sign_est is not None and self.misses < t['max_misses']:
            self.misses += 1
            est = self.sign_est                                 # boş bakış: saklı kestirim (odometride)
            self.log.append((now, 'TRANSIT', f'tabela görülmedi, saklı kestirimle ({self.misses})'))
        elif est is not None:
            self.misses = 0
        if est is None:
            self.search_steps = 0
            self._go('FIND_STATION', now, 'tabela kayboldu, yeniden aranıyor')
            return 0.0, 0.0
        xy, yaw, n, rmin = est
        self.sign_xy = xy
        d = xy - pose.xy
        r = float(np.linalg.norm(d))
        u = d / r
        self.legs += 1
        if self.legs > t['max_legs']:
            self._fail(now, 'ön-yanaşmaya ulaşılamadı (ayak sınırı)')
            return 0.0, 0.0
        if r > t['near_range']:
            # uzak: tabela yaw'ı güvenilmez (D-66), tabelaya doğru en çok max_leg git ve yeniden bak
            p = pose.xy + u * min(r - t['approach_range'], t['max_leg'])
            self.prim = GoTo(p, math.atan2(u[1], u[0]), t['v_travel'], self.c['motion'])
            self.log.append((now, 'TRANSIT', f'ayak {self.legs}: {r:.2f} m → {min(r - t["approach_range"], t["max_leg"]):.2f} m ileri'))
            return 0.0, 0.0
        nv = np.array([math.cos(yaw), math.sin(yaw)])
        if np.dot(nv, u) < math.cos(math.radians(t['max_axis_dev_deg'])):
            nv, yaw = u, math.atan2(u[1], u[0])             # tutarsız yaw: bakış doğrultusunu kullan
        p = xy - nv * t['predock_distance']
        self.prim = GoTo(p, yaw, t['v_travel'], self.c['motion'])
        self.legs = -1
        self.log.append((now, 'TRANSIT', f'ön-yanaşma: {r:.2f} m, eksen {math.degrees(yaw):.1f}°'))
        return 0.0, 0.0

    def _wrong_station(self, now, pose):
        """Önümüzdeki (en yakın, yakın mesafedeki) tabela beklenen renk değilse (renk, konum)."""
        t = self.c['transit']
        best = None
        for color in list(self.signs.obs):
            e = self.signs.estimate(color, now)
            if e is None:
                continue
            r = float(np.linalg.norm(e[0] - pose.xy))
            if r <= t['wrong_station_range'] and (best is None or r < best[2]):
                best = (color, e[0], r)
        if best is None or best[0] == self.target:
            return None
        return best

    def _excluded(self, xy):
        return any(np.linalg.norm(np.asarray(xy) - x) < self.c['transit']['exclude_radius'] for x in self.excluded)

    def _lane_color(self):
        """İstasyona giden şeridin rengi: istasyon rengi; kaynak (nötr) istasyonun şeridi sarı (katalog
        lane_neutral). F4 ara koşusunda eşleme yoktu: dönüşte robot 'neutral' şerit arıyor, hiç bulamıyordu."""
        return self.c['transit']['lane_color'].get(self.target, self.target)

    def _arrived(self, now, pose):
        self.tr.reset()
        self.prim, self.pd_moves = None, 0
        self._go('PRE_DOCK', now, 'istasyon ekseni ölçülüyor')

    def _s_pre_dock(self, now, pose, q, el):
        t = self.c['transit']
        if self.prim is not None:
            v, w, done = self._run_prim(pose)
            if done and self.pd_moves >= t['predock_moves']:
                # düzeltmeden sonra yeniden ölçülmez: OBSERVE hemen ölçüyor, ALIGN bölme eksenine götürüyor
                self._leg_done(now, f'{self.target} ön-yanaşmada (düzeltildi)')
            elif done:
                self.tr.reset()
                self.t_state = now                       # yeniden ölç
            return v, w
        if el < self.c['observe_s']:
            return 0.0, 0.0
        wrong = self._wrong_station(now, pose)
        if wrong is not None:
            # plan §7.2: önümüzdeki istasyon başka renk; o noktayı dışla, geri çık, yeniden ara
            self.excluded.append(self.sign_xy if self.sign_xy is not None else wrong[1])
            self.sign_memory.pop(self.target, None)
            self.sign_est, self.sign_xy = None, None
            self._recover(now, f'yanlış istasyon: önde {wrong[0]} tabelası, beklenen {self.target}')
            return 0.0, 0.0
        conf = self.tr.confirmed()
        if not conf:
            if el > t['predock_wait_s']:
                self._leg_done(now, f'{self.target} ön-yanaşmada (hedef çifti görülmedi, tabelayla)')
            return 0.0, 0.0
        th = math.atan2(np.mean([math.sin(b.theta) for b in conf]), np.mean([math.cos(b.theta) for b in conf]))
        n = np.array([math.cos(th), math.sin(th)])
        tv = np.array([-n[1], n[0]])
        E = np.array([b.entry for b in conf])
        # istasyon ortası: üç bölme görülüyorsa ortadaki; değilse tabela konumunun ön çizgiye izdüşümü
        if len(conf) >= 3:
            ys = sorted(float(np.dot(e, tv)) for e in E)
            c_lat = ys[len(ys) // 2]
        else:
            sxy = self.sign_xy if self.sign_xy is not None else E.mean(0)
            c_lat = float(np.dot(sxy, tv))
        c_ax = float(np.mean(E @ n))
        center = n * c_ax + tv * c_lat
        goal = center - n * t['predock_distance']
        dl = float(np.dot(pose.xy - goal, tv))
        dd = float(np.dot(pose.xy - goal, n))
        dy = wrap(th - pose.yaw)
        if (abs(dl) <= t['predock_lat_tol'] and abs(dd) <= t['predock_dist_tol']
                and abs(dy) <= math.radians(t['predock_yaw_tol_deg'])) or self.pd_moves >= t['predock_moves']:
            self._leg_done(now, f'{self.target} ön-yanaşmada (yanal {dl * 1000:.0f} mm, yön {math.degrees(dy):.1f}°)')
            return 0.0, 0.0
        self.pd_moves += 1
        self.log.append((now, 'PRE_DOCK', f'düzeltme: yanal {dl * 1000:.0f} mm, mesafe {dd * 1000:.0f} mm, yön {math.degrees(dy):.1f}°'))
        self.prim = GoTo(goal, th, self.c['v_transit'], self.c['motion'])
        return 0.0, 0.0

    def _s_lane_join(self, now, pose, q, el):
        t = self.c['transit']
        if self.prim is not None:
            v, w, done = self._run_prim(pose)
            if done:
                self._go('LANE_FOLLOW', now, 'şeritte')
            return v, w
        if el < t['lane_look_s']:
            return 0.0, 0.0
        est = self.signs.estimate(self.target, now)
        hint = pose.yaw if est is None else math.atan2(*(est[0] - pose.xy)[::-1])
        ln = self.lane.line(hint)
        if ln is None:
            # şerit yok: tabela yöntemiyle devam (aynı hedef, yalnız yöntem farklı)
            self._go('TRANSIT', now, f'{self.target} şeridi görülmedi, tabelayla')
            return 0.0, 0.0
        m, d = ln
        if est is not None:
            off = abs(float(np.dot(est[0] - m, [-d[1], d[0]])))
            if off > t['lane_sign_max_offset']:
                # şerit tabelaya gitmiyor (istasyonların yeri değişmiş, zemin eski düzende): tabelayla git
                self._go('TRANSIT', now, f'{self._lane_color()} şeridi tabelaya gitmiyor ({off:.2f} m), tabelayla')
                return 0.0, 0.0
        s0 = float(np.dot(pose.xy - m, d))
        p = m + d * (s0 + t['join_ahead'])
        self.prim = GoTo(p, math.atan2(d[1], d[0]), t['v_travel'], self.c['motion'])
        return 0.0, 0.0

    def _s_lane_follow(self, now, pose, q, el):
        t = self.c['transit']
        if now - self.lane.last_t > t['lane_lost_s'] and self.lane.bar is None:
            self._fail(now, f'{self.target} şeridi kayboldu')
            return 0.0, 0.0
        ln = self.lane.line(pose.yaw, recent_s=t['lane_recent_s'])
        if ln is None:
            return 0.0, 0.0
        m, d = ln
        theta = math.atan2(d[1], d[0])
        if self.lane.bar is not None:
            s_stop = float(np.dot(self.lane.bar - m, d)) - t['lane_stop_offset']
        else:
            s_stop = float(np.dot(pose.xy - m, d)) + t['lane_free_ahead']
        entry = m + d * s_stop
        v, w, done = BayServo(t['lane_servo'], 0.0, t['v_travel']).step(pose, entry, theta)
        if done:
            self._arrived(now, pose)
            return 0.0, 0.0
        return v, w

    # ---------------------------------------------------------------- istasyonda
    def _s_observe(self, now, pose, q, el):
        c = self.c
        if el >= c['observe_s'] and self.tr.settled(now):
            self._go('SELECT_BAY', now)
        elif el >= c['observe_max_s'] and self.tr.confirmed():
            self._go('SELECT_BAY', now, 'gözlem eksik, görülenlerle: ' + self.tr.summary(now))
        return 0.0, 0.0

    def _s_select_bay(self, now, pose, q, el):
        """Politika sırasıyla bölmelere bakılır; sırada doluluğu belirsiz (görüş dışında kalmış) bir bölme
        kararı değiştirebiliyorsa select_wait_s beklenir, sonra belirsizler atlanır."""
        c, tr = self.c, self.tr
        bays = tr.left_to_right(tr.confirmed())
        pick = self.goal[0] == 'pick'
        if pick:
            order = list(range(len(bays)))                                  # soldan sağa ilk dolu (K10)
            want, skip = tr.occupied, (lambda b, t: tr.empty(b, t))
        else:
            mid = (len(bays) - 1) / 2.0
            order = sorted(range(len(bays)), key=lambda i: (abs(i - mid), i))  # önce orta, sonra sol
            want, skip = tr.empty, tr.occupied
        chosen, unknown = None, []
        for i in order:
            b = bays[i]
            if want(b, now):
                chosen = b
                break
            if not skip(b, now):
                unknown.append(i + 1)
                if el <= c['select_wait_s']:
                    return 0.0, 0.0                                         # belirsiz: bekle
        if pick:
            orph = tr.orphan_parcels(now)
            if chosen is not None and bays:
                # yalnız seçilenin solundaki sahipsiz koli kararı değiştirir (soldan sağa ilk dolu)
                nm = np.mean([x.axis for x in bays], axis=0)
                left = np.array([-nm[1], nm[0]])
                orph = [o for o in orph if float(np.dot(o[1] - chosen.entry, left)) > c['orphan_left_min']]
            if orph and self.reobserve < c['reobserve_max']:
                # koli görülüyor ama bölmesi izlenmiyor: kolinin önüne geç, yeniden gözle (plan §7.2)
                if bays:
                    nm = np.mean([x.axis for x in bays], axis=0)
                    orph.sort(key=lambda o: -float(np.dot(o[1], [-nm[1], nm[0]])))   # en soldaki
                xy = orph[0][1]
                conf = tr.confirmed()
                th = (math.atan2(np.mean([math.sin(b.theta) for b in conf]), np.mean([math.cos(b.theta) for b in conf]))
                      if conf else pose.yaw)
                n = np.array([math.cos(th), math.sin(th)])
                self.reobserve += 1
                self.prim = GoTo(xy - n * (c['align_distance'] + c['reobserve_back']), th, c['v_transit'], self.c['motion'])
                self._go('REPOSITION', now, f'bölmesi görülmeyen koli ({orph[0][2]}), yeniden konumlanma {self.reobserve}')
                return 0.0, 0.0
        if chosen is None and (unknown or not pick) and self.reobserve < c['reobserve_max'] and bays:
            # karar yok (bırakmada boş bölme yok ya da belirsiz bölme var): bir bölmenin çifti algılanmamış
            # olabilir. İstasyon ortasının daha gerisinden, eksene dik bakarak yeniden gözle.
            th = math.atan2(np.mean([math.sin(b.theta) for b in bays]), np.mean([math.cos(b.theta) for b in bays]))
            n, tv = np.array([math.cos(th), math.sin(th)]), np.array([-math.sin(th), math.cos(th)])
            E = np.array([b.entry for b in bays])
            ctr = E.mean(0) if self.sign_xy is None else n * float(np.mean(E @ n)) + tv * float(np.dot(self.sign_xy, tv))
            self.reobserve += 1
            self.prim = GoTo(ctr - n * c['reobserve_wide'], th, c['v_transit'], self.c['motion'])
            self._go('REPOSITION', now, f'karar yok (belirsiz {unknown}), istasyon ortasından yeniden gözlem {self.reobserve}')
            return 0.0, 0.0
        if chosen is None:
            if pick and not unknown:
                self._finish(now, 'kaynak istasyon boş')
            elif pick:
                self._fail(now, f'doluluk belirlenemedi (bölme {unknown})')
            else:
                self._fail(now, 'boş bölme yok' + (f' (belirsiz: {unknown})' if unknown else ''))
                self.log.append((now, 'FAULT', tr.summary(now)))
            return 0.0, 0.0
        if pick and self.task == 'full':
            occ = sum(1 for b in bays if tr.occupied(b, now))
            self.parcels_total = self.parcels_done + occ
            # alınacak koliden sonra kaynakta kalan (belirsiz bölme ve sahipsiz koli de "kalan" sayılır)
            self.source_left = occ - 1 + len(unknown) + len(tr.orphan_parcels(now))
        self.bay = chosen
        self.fork_target = c['pick_q'] if pick else c['carry_q']
        what = f'mühür {self.bay.parcel}' if pick else 'boş'
        note = f', belirsiz atlandı: {unknown}' if unknown else ''
        self._go('ALIGN', now, f'bölme {bays.index(self.bay) + 1}/{len(bays)} (soldan), {what}{note}')
        self.prim = None
        return 0.0, 0.0

    def _s_reposition(self, now, pose, q, el):
        v, w, done = self._run_prim(pose)
        if done:
            self.tr.reset()
            self._go('OBSERVE', now, 'yeniden gözlem')
            return 0.0, 0.0
        return v, w

    def _s_align(self, now, pose, q, el):
        c = self.c
        if self.prim is None:
            p = self.bay.entry - self.bay.axis * c['align_distance']
            self.prim = GoTo(p, self.bay.theta, c['v_transit'], self.c['motion'])
        v, w, done = self._run_prim(pose)
        if done:
            self.align_tries += 1
            self._go('SETTLE', now)
            return 0.0, 0.0
        return v, w

    def _s_settle(self, now, pose, q, el):
        c = self.c
        if el < c['settle_s']:
            return 0.0, 0.0
        dist, lat, eh = BayServo(c['servo'], 0, 0).errors(pose, self.bay.entry, self.bay.theta)
        ok = abs(lat) <= c['align_lat_tol'] and abs(eh) <= math.radians(c['align_yaw_tol_deg'])
        if not ok and self.align_tries <= c['align_retries']:
            self._go('ALIGN', now, f'yeniden hizalama (yanal {lat * 1000:.0f} mm, yön {math.degrees(eh):.1f}°)')
            return 0.0, 0.0
        self.servo = BayServo(c['servo'], self.robot['stop_distance'], c['v_dock'])
        self._go('ENTER', now, f'giriş (yanal {lat * 1000:.0f} mm, yön {math.degrees(eh):.2f}°, mesafe {dist:.2f} m)')
        return 0.0, 0.0

    def _s_enter(self, now, pose, q, el):
        c = self.c
        dist, lat, eh = self.servo.errors(pose, self.bay.entry, self.bay.theta)
        seen = now - self.bay.last_t
        if dist > c['blind_distance'] and seen > c['lost_hold_s']:
            if seen > c['lost_fail_s']:
                self._recover(now, f'hedef kayıp ({seen:.1f} s görülmedi)')
            return 0.0, 0.0                                     # görüş dönene kadar bekle
        if not self.trimmed and dist <= c['trim_distance']:
            self.trimmed = True
            self.prim = None
            self._go('TRIM', now, f'yanal {lat * 1000:.1f} mm, yön {math.degrees(eh):.2f}°')
            return 0.0, 0.0
        v, w, done = self.servo.step(pose, self.bay.entry, self.bay.theta)
        if done:
            if self.goal[0] == 'pick':
                self.fork_target = c['carry_q']
                self._go('LIFT', now, f'tam giriş (CV mesafesi {dist:.3f} m)')
            else:
                self.fork_target = c['pick_q']
                self._go('LOWER', now, f'tam giriş (CV mesafesi {dist:.3f} m)')
            return 0.0, 0.0
        return v, w

    def _s_trim(self, now, pose, q, el):
        # dururken ölçüm tazelenir (settle_s), sonra yerinde dönüşle yön düzeltilir
        c = self.c
        if el < c['settle_s']:
            return 0.0, 0.0
        if self.prim is None:
            self.prim = Turn(self.bay.theta, dict(self.c['motion'], turn_tol=math.radians(c['trim_yaw_tol_deg'])))
        v, w, done = self._run_prim(pose)
        if done:
            self._go('ENTER', now, 'son hizalama tamam')
            return 0.0, 0.0
        return v, w

    def _s_lift(self, now, pose, q, el):
        if q is not None and abs(q - self.c['carry_q']) < self.c['fork_tol']:
            self.prim = None
            self._go('BACK_OUT', now)
        return 0.0, 0.0

    def _s_lower(self, now, pose, q, el):
        if q is not None and abs(q - self.c['pick_q']) < self.c['fork_tol']:
            self.prim = None
            self._go('BACK_OUT', now)
        return 0.0, 0.0

    def _s_back_out(self, now, pose, q, el):
        c = self.c
        if self.prim is None:
            self.prim = Straight(pose, -c['back_out'], c['v_back'], self.c['motion'])
        v, w, done = self._run_prim(pose)
        if done:
            self.load_hist = []
            self._go('VERIFY', now)
            return 0.0, 0.0
        return v, w

    def _s_verify(self, now, pose, q, el):
        c = self.c
        h = self.load_hist
        if len(h) < c['verify_window']:
            return 0.0, 0.0
        r = float(np.mean(h))
        if self.goal[0] == 'pick':
            if r >= c['verify_ratio']:
                self.loaded = True
                self._leg_done(now, f'koli çatalda (mühür {self.bay.parcel})')
            elif self.pick_retries < c['pick_retries']:
                # plan §7.2: indir, (geri çıkıldı), bir kez yeniden dene; seçim yeniden yapılır
                self.pick_retries += 1
                self.fork_target = c['pick_q']
                self.tr.reset()
                self.bay, self.align_tries, self.trimmed, self.prim = None, 0, False, None
                self._go('OBSERVE', now, f'alma başarısız: çatalda koli yok → yeniden deneme {self.pick_retries}')
            else:
                self._fail(now, 'alma başarısız: çatalda koli yok')
        else:
            if r <= 1.0 - c['verify_ratio']:
                self.loaded = False
                self._leg_done(now, 'koli bırakıldı')
            else:
                self._fail(now, 'bırakma başarısız: koli çatalda')
        return 0.0, 0.0


PickMission = Mission                                   # F3 adı (geriye uyum)
