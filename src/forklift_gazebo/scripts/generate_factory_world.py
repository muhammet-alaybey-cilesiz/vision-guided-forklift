#!/usr/bin/env python3
"""factory.yaml + stations.yaml + station_params.yaml → worlds/factory.sdf ve factory_lanes.yaml.

Build sırasında CMake çalıştırır (kaynak ağaca yazmaz). Dünya:
  - fizik, sistemler ve zemin collision'ı station_test ile aynı (DART 1 ms,
    μ = 1.0, D-15); zeminin visual'ı yoktur, Depot'un zemini görünür
  - Depot holü (Fuel, sürüm sabit), experimental:params ile collision'ı ve
    clutter: clean ise raf/kasa setleri kaldırılmış
  - ışıklar, zemin işaretleri (hücre bandı, kılavuz şeritler, yaya yolu),
    kolon koruyucuları, levhalar ve Fuel aksesuarları
İstasyonlar ve koliler dünyada YOKTUR: stations.launch.py spawn eder (D-43).

factory_lanes.yaml: her kılavuz şeridin dünya çerçevesinde başlangıç/bitiş
noktası, genişliği ve hedef istasyonu (ileride şerit tespiti için ground truth).

Kullanım:
  generate_factory_world.py --factory F --stations S --params P --world OUT.sdf --lanes OUT.yaml
"""
import argparse
import math
import os
import sys
from urllib.parse import quote

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import station_layout as sl  # noqa: E402

SIGN_ROOT = 'model://factory_signs/materials/textures'


def f(v):
    return ' '.join(f'{round(x, 6):g}' for x in v)


def matte(rgba, albedo='', roughness=0.85, extra=''):
    rgba = list(rgba) + [1.0] * (4 - len(rgba))
    amb = [c * 0.8 for c in rgba[:3]] + [rgba[3]]
    amap = f'<albedo_map>{albedo}</albedo_map>' if albedo else ''
    return (f'<material><ambient>{f(amb)}</ambient><diffuse>{f(rgba)}</diffuse>'
            f'<specular>0.05 0.05 0.05 1</specular><pbr><metal>{amap}{extra}'
            f'<metalness>0</metalness><roughness>{roughness}</roughness></metal></pbr></material>')



def box(name, size, xyz, yaw, material, shadows=False):
    cs = '' if shadows else '<cast_shadows>false</cast_shadows>'
    return (f'<visual name="{name}"><pose>{f(xyz)} 0 0 {round(yaw, 9):g}</pose>{cs}'
            f'<geometry><box><size>{f(size)}</size></box></geometry>{material}</visual>')


def static_model(name, visuals, pose=(0, 0, 0, 0)):
    return (f'<model name="{name}"><static>true</static><pose>{f(pose[:3])} 0 0 {pose[3]:g}</pose>'
            f'<link name="link">{"".join(visuals)}</link></model>')


# --------------------------------------------------------------- bölümler ---
def hall(F):
    H = F['hall']
    edits = [f'<collision element_id="{c}" action="remove"/>' for c in H.get('remove_collisions', [])]
    edits += [f'<visual element_id="{v}" action="remove"/>' for v in H.get('remove_visuals', [])]
    edits += [f'<light element_id="{v}" action="remove"/>' for v in H.get('remove_lights', [])]
    if H.get('clutter', 'full') == 'clean':
        edits += [f'<visual element_id="{v}" action="remove"/>' for v in H['clutter_visuals']]
    elif H.get('clutter') != 'full':
        raise SystemExit(f'hall.clutter full | clean olmalı: {H.get("clutter")!r}')
    x, y, z, yaw = (H['pose'] + [0, 0])[:4]
    return (f'<include><uri>{F["fuel"]}/{quote(H["model"])}</uri><name>hall</name><static>true</static>'
            f'<pose>{f([x, y, z])} 0 0 {yaw:g}</pose>'
            f'<experimental:params>{"".join(edits)}</experimental:params></include>')


def lights(F):
    out = []
    for L in F['lights']:
        if L['type'] == 'directional':
            out.append(f'<light type="directional" name="{L["name"]}"><pose>0 0 10 0 0 0</pose>'
                       f'<cast_shadows>{str(L["shadows"]).lower()}</cast_shadows>'
                       f'<diffuse>{f(L["diffuse"])} 1</diffuse><specular>{f([L["specular"]] * 3)} 1</specular>'
                       f'<direction>{f(L["direction"])}</direction></light>')
            continue
        pts = [(x, y) for x in L['grid']['x'] for y in L['grid']['y']]
        c, l, q = L['attenuation']
        for i, (x, y) in enumerate(pts):
            out.append(
                f'<light type="spot" name="{L["name"]}_{i}"><pose>{f([x, y, L["z"]])} 0 0 0</pose>'
                f'<cast_shadows>{"true" if i < L["shadows_first"] else "false"}</cast_shadows>'
                f'<diffuse>{f(L["diffuse"])} 1</diffuse><specular>{f([L["specular"]] * 3)} 1</specular>'
                f'<attenuation><range>{L["range"]}</range><constant>{c}</constant><linear>{l}</linear>'
                f'<quadratic>{q}</quadratic></attenuation><direction>0 0 -1</direction>'
                f'<spot><inner_angle>{L["inner"]}</inner_angle><outer_angle>{L["outer"]}</outer_angle>'
                f'<falloff>1.0</falloff></spot></light>')
    return out


def cell_band(M):
    C = M['cell']
    x0, x1, y0, y1 = C['bounds']
    w, seg, h = C['width'], C['segment'], M['height']
    mat = matte([1, 1, 1], f'{SIGN_ROOT}/hazard_tape.png', 0.7)
    vis = []

    # Kutunun üst yüzünde doku u ekseni yerel -y boyuncadır (UV ölçümü, D-53):
    # segmentin uzun kenarı yerel y'ye konur, x yönündeki kenarlar 90° döndürülür.
    def run(tag, a, b, fixed, along_x):
        n = max(1, round((b - a) / seg))
        L = (b - a) / n
        for i in range(n):
            c = a + (i + 0.5) * L
            xyz = [c, fixed, h / 2] if along_x else [fixed, c, h / 2]
            vis.append(box(f'cell_{tag}_{i}', [w, L, h], xyz, math.pi / 2 if along_x else 0, mat))

    run('s', x0, x1, y0 + w / 2, True)
    run('n', x0, x1, y1 - w / 2, True)
    run('w', y0 + w, y1 - w, x0 + w / 2, False)
    run('e', y0 + w, y1 - w, x1 - w / 2, False)
    return vis


def lanes(M, P, layout):
    Ln = M['lanes']
    S = P['station']
    by_name = {st['name']: st for st in layout['stations']}
    h, w = M['height'], Ln['width']
    vis, truth = [], []
    cx, cy = Ln['from']
    for name in Ln['to_stations']:
        st = by_name[name]
        ex, ey, _ = sl.compose(st['pose'], -(S['apron_depth'] + Ln['end_gap']), 0.0)
        yaw = math.atan2(ey - cy, ex - cx)
        x0, y0 = cx + math.cos(yaw) * Ln['start_gap'], cy + math.sin(yaw) * Ln['start_gap']
        L = math.hypot(ex - x0, ey - y0)
        color_name = 'yellow' if st['color'] == 'neutral' else st['color']
        rgba = P['colors']['lane_neutral'] if st['color'] == 'neutral' else P['colors'][st['color']]
        vis.append(box(f'lane_{name}', [L, w, h], [(x0 + ex) / 2, (y0 + ey) / 2, h / 2], yaw,
                       matte(rgba, roughness=0.6)))
        sb = Ln['stop_bar']
        bx, by = ex - math.cos(yaw) * sb['width'] / 2, ey - math.sin(yaw) * sb['width'] / 2
        vis.append(box(f'stop_{name}', [sb['width'], sb['length'], h * 1.5], [bx, by, h * 0.75], yaw,
                       matte(sb['color'], roughness=0.6)))
        truth.append({'station': name, 'start': [round(x0, 6) + 0.0, round(y0, 6) + 0.0],
                      'end': [round(ex, 6) + 0.0, round(ey, 6) + 0.0],
                      'width': w, 'yaw': round(yaw, 6), 'color': color_name})
    return vis, truth


def walkway(M):
    W = M['walkway']
    (x0, x1), (y0, y1) = W['x'], W['y']
    h = M['height']
    e = W['edge']
    vis = [box('walk_floor', [x1 - x0, y1 - y0 - 2 * e['width'], h], [(x0 + x1) / 2, (y0 + y1) / 2, h / 2], 0,
               matte(W['color'], roughness=0.7))]
    for tag, y in (('s', y0 + e['width'] / 2), ('n', y1 - e['width'] / 2)):
        vis.append(box(f'walk_edge_{tag}', [x1 - x0, e['width'], h], [(x0 + x1) / 2, y, h / 2], 0,
                       matte(e['color'], roughness=0.6)))
    st = W['stencil']
    sw = (y1 - y0 - 2 * e['width']) * 0.8          # yazı yaya yolu genişliğinin %80'i
    x = x0 + st['every'] / 2
    i = 0
    while x < x1:
        # yazının taban çizgisi yaya yolunu enine keser (doku u ekseni yerel -y);
        # +x yönünde yürüyen kişi soldan sağa okur
        vis.append(box(f'walk_text_{i}', [sw / 4, sw, h * 1.5], [x, (y0 + y1) / 2, h * 0.75], 0,
                       matte([1, 1, 1], f'{SIGN_ROOT}/floor_walkway.png', 0.7)))
        x += st['every']
        i += 1
    return vis


def guards_and_signs(F):
    vis = []
    G = F['pillar_guards']
    sx, sy, sz = G['size']
    for i, (x, y) in enumerate(G['centers']):
        vis.append(box(f'guard_{i}', [sx, sy, sz], [x, y, sz / 2], 0,
                       matte([1, 1, 1], f'{SIGN_ROOT}/hazard_panel.png', 0.5), shadows=True))
    for i, s in enumerate(F['wall_signs']):
        w, hh = s['size']
        # levhanın okunan yüzü kutunun -x yüzü (D-47); normal yaw → kutu yaw + π
        vis.append(box(f'sign_{i}', [0.004, w, hh], s['center'], s['yaw'] + math.pi,
                       matte([1, 1, 1], f'{SIGN_ROOT}/{s["texture"]}', 0.6), shadows=True))
    return vis


def props(F):
    out = []
    for i, p in enumerate(F['props']):
        x, y, z, yaw = (p['pose'] + [0, 0])[:4]
        out.append(f'<include><uri>{F["fuel"]}/{quote(p["uri"])}</uri><name>prop_{i}</name><static>true</static>'
                   f'<pose>{f([x, y, z])} 0 0 {yaw:g}</pose></include>')
    return out


WORLD = '''<?xml version="1.0"?>
<!--
  factory.sdf — ÜRETİLMİŞ DOSYA, elle düzenlemeyin.
  Kaynak: config/factory.yaml (+ stations.yaml, station_params.yaml),
  üreten: scripts/generate_factory_world.py (build sırasında).
  İstasyonlar ve koliler launch sırasında spawn edilir (stations.launch.py).
-->
<sdf version="1.11" xmlns:experimental="http://sdformat.org/schemas/experimental">
  <world name="factory">
    <physics name="dart_1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <scene>
      <ambient>{ambient} 1</ambient>
      <background>{background} 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
    </scene>
{lights}
    <!-- zemin: yalnız collision (μ = 1.0, D-15); görünen zemin Depot'un -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
      </link>
    </model>
    {hall}
    {markings}
    {fixtures}
{props}
  </world>
</sdf>
'''


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    for k in ('factory', 'stations', 'params', 'world', 'lanes'):
        ap.add_argument('--' + k, required=True)
    a = ap.parse_args()
    F = sl.load_yaml(a.factory)
    layout = sl.load_yaml(a.stations)
    P = sl.load_yaml(a.params)
    sl.validate_stations(P, layout)
    M = F['markings']
    lane_vis, truth = lanes(M, P, layout)
    world = WORLD.format(
        ambient=f(F['scene']['ambient']), background=f(F['scene']['background']),
        lights='\n'.join('    ' + s for s in lights(F)),
        hall=hall(F),
        markings=static_model('floor_markings', cell_band(M) + lane_vis + walkway(M)),
        fixtures=static_model('fixtures', guards_and_signs(F)),
        props='\n'.join('    ' + s for s in props(F)))
    for path in (a.world, a.lanes):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    open(a.world, 'w').write(world)
    with open(a.lanes, 'w') as fh:
        fh.write('# ÜRETİLMİŞ (generate_factory_world.py): factory dünyasındaki kılavuz şeritler,\n'
                 '# dünya çerçevesinde [m, rad]; şerit merkez çizgisi start → end.\n')
        yaml.safe_dump({'world': 'factory', 'lanes': truth}, fh, sort_keys=False, allow_unicode=True)
    print(f'{a.world}: {len(truth)} şerit, {len(F["props"])} aksesuar')


if __name__ == '__main__':
    main()
