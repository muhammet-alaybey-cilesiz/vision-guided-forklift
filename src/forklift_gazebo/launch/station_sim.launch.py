"""İstasyonlu dünya: sim.launch.py (robot, köprüler, komut zinciri) + stations.launch.py.

sim.launch.py'nin tüm argümanları (gui, headless, rviz, control, x, y, z, yaw,
params_file, verbosity) komut satırından aynen geçer; dünya dosyası
station_world argümanından seçilir:
  station_test : sade test dünyası (worlds/station_test.sdf, varsayılan)
  factory      : fabrika holü (build'de config/factory.yaml'dan üretilen
                 worlds/factory.sdf); Gazebo GUI açılış kamerası
                 config/factory_gui.config'ten (D-53)
İstasyon/koli yerleşimi iki dünyada da config/stations.yaml'dan gelir
(stations_file argümanıyla değiştirilebilir).
Gazebo GUI'si bu istasyonlar ve koliler sunucu sahnesine girdikten sonra açılır
(gui_wait_models, DECISIONS D-52).

  ros2 launch forklift_gazebo station_sim.launch.py
  ros2 launch forklift_gazebo station_sim.launch.py station_world:=factory
  ros2 launch forklift_bringup bringup.launch.py world:=station_test   (aynı dünya + RViz + GUI)
  ros2 launch forklift_bringup bringup.launch.py world:=factory
"""

import os
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
    SetLaunchConfiguration)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

PKG = 'forklift_gazebo'
sys.path.insert(0, os.path.join(get_package_prefix(PKG), 'lib', PKG))
import station_layout as sl  # noqa: E402

WORLDS = ('station_test', 'factory')


def _gui_wait_models(context):
    """stations.launch.py'nin ekleyeceği model adları (istasyonlar + koliler)."""
    layout = sl.load_yaml(LaunchConfiguration('stations_file').perform(context))
    params = sl.load_yaml(os.path.join(get_package_share_directory(PKG), 'config', 'station_params.yaml'))
    names = [st['name'] for st in layout['stations']] + [n for n, _, _ in sl.parcels(params, layout)]
    return [SetLaunchConfiguration('gui_wait_models', ','.join(names))]


def _setup(context):
    """Dünya adına bağlı adımlar: sim (dünya dosyası, GUI yapılandırması) ve istasyonlar."""
    share = get_package_share_directory(PKG)
    world = LaunchConfiguration('station_world').perform(context)
    gui_config = os.path.join(share, 'config', 'factory_gui.config') if world == 'factory' else ''
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'sim.launch.py')),
        launch_arguments={'world': os.path.join(share, 'worlds', world + '.sdf'),
                          'gui_wait_models': LaunchConfiguration('gui_wait_models'),
                          'gui_config': gui_config}.items(),
    )
    # Kapsamlı: stations.launch.py'nin params_file argümanı robotunkini ezmesin
    # (ikisi de 'params_file' adını kullanır).
    stations = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'stations.launch.py')),
        launch_arguments={
            'world': world,
            'stations_file': LaunchConfiguration('stations_file'),
            'params_file': os.path.join(share, 'config', 'station_params.yaml'),
        }.items(),
    )], scoped=True)
    return [sim, stations]


def generate_launch_description():
    share = FindPackageShare('forklift_gazebo')
    return LaunchDescription([
        DeclareLaunchArgument('station_world', default_value='station_test', choices=list(WORLDS)),
        DeclareLaunchArgument(
            'stations_file', default_value=PathJoinSubstitution([share, 'config', 'stations.yaml'])),
        OpaqueFunction(function=_gui_wait_models),
        OpaqueFunction(function=_setup),
    ])
