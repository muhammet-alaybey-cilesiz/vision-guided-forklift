"""stations.yaml'daki istasyonları ve kolileri xacro'dan üretip çalışan dünyaya spawn eder.

Her istasyon station.sdf.xacro'dan (statik), her dolu bölme için bir koli
parcel.sdf.xacro'dan (dinamik) üretilir ve ros_gz_sim create -string ile
eklenir. Koliler istasyonlardan SONRA eklenir: create süreçleri zincirlenir
(her biri bir öncekinin çıkışında başlar). Aksi halde bir koli, istasyonu
gelmeden yerçekimiyle zemine düşebilirdi.

Koli nominal pozu (bölme çerçevesi): önü x = parcel_front_gap, tabanı
z = ledge_height; hesap station_layout.parcels() içinde.

Argümanlar:
  world         : hedef dünya adı (SDF <world name>), varsayılan station_test
  stations_file : yerleşim (varsayılan share/forklift_gazebo/config/stations.yaml)
  params_file   : ölçüler (varsayılan share/forklift_gazebo/config/station_params.yaml)
  parcels       : true → occupancy'deki kolileri de ekle (varsayılan true)

Not: ros_gz_sim create modelin kendi <pose>'unu ezer (D-47); poz -x -y -z -Y ile verilir.
"""

import os
import sys

import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'forklift_gazebo'
# Ortak yerleşim modülü lib/forklift_gazebo altında kurulu (CMakeLists.txt)
sys.path.insert(0, os.path.join(get_package_prefix(PKG), 'lib', PKG))
import station_layout as sl  # noqa: E402


def _create(world, name, sdf, x, y, z, yaw):
    return Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_{name}',
        arguments=['-world', world, '-name', name, '-string', sdf,
                   '-x', f'{x:.6f}', '-y', f'{y:.6f}', '-z', f'{z:.6f}', '-Y', f'{yaw:.9f}'],
        output='screen',
    )


def _launch_setup(context):
    share = get_package_share_directory(PKG)
    world = LaunchConfiguration('world').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    layout = sl.load_yaml(LaunchConfiguration('stations_file').perform(context))
    params = sl.load_yaml(params_file)
    sl.validate_stations(params, layout)

    def expand(xacro_name, args):
        mappings = dict(a.split(':=', 1) for a in args)
        return xacro.process_file(os.path.join(share, 'xacro', xacro_name), mappings=mappings).toxml()

    nodes = []
    for st in layout['stations']:
        sdf = expand('station.sdf.xacro', sl.station_xacro_args(st, params_file))
        x, y, yaw = st['pose']
        nodes.append(_create(world, st['name'], sdf, x, y, 0.0, yaw))
    if LaunchConfiguration('parcels').perform(context).lower() == 'true':
        for name, color, (x, y, z, yaw) in sl.parcels(params, layout):
            sdf = expand('parcel.sdf.xacro', sl.parcel_xacro_args(name, color, params_file))
            nodes.append(_create(world, name, sdf, x, y, z, yaw))

    # Zincir: nodes[0] hemen, nodes[i] nodes[i-1] bitince
    actions = [nodes[0]] if nodes else []
    for prev, nxt in zip(nodes, nodes[1:]):
        actions.append(RegisterEventHandler(OnProcessExit(target_action=prev, on_exit=[nxt])))
    if nodes:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=nodes[-1],
            on_exit=[LogInfo(msg=f'stations.launch: {len(nodes)} model eklendi ({world})')])))
    return actions


def generate_launch_description():
    share = get_package_share_directory(PKG)
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='station_test'),
        DeclareLaunchArgument('stations_file', default_value=os.path.join(share, 'config', 'stations.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'station_params.yaml')),
        DeclareLaunchArgument('parcels', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
