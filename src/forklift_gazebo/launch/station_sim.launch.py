"""station_test dünyası: sim.launch.py (robot, köprüler, komut zinciri) + stations.launch.py.

sim.launch.py'nin tüm argümanları (gui, headless, rviz, control, x, y, z, yaw,
params_file, verbosity) komut satırından aynen geçer; burada yalnızca dünya
dosyası station_test.sdf'e sabitlenir. İstasyon/koli yerleşimi
config/stations.yaml'dan gelir (stations_file argümanıyla değiştirilebilir).
Gazebo GUI'si bu istasyonlar ve koliler sunucu sahnesine girdikten sonra açılır
(gui_wait_models, DECISIONS D-52).

  ros2 launch forklift_gazebo station_sim.launch.py
  ros2 launch forklift_bringup bringup.launch.py world:=station_test   (aynı dünya + RViz + GUI)
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

WORLD = 'station_test'


def _gui_wait_models(context):
    """stations.launch.py'nin ekleyeceği model adları (istasyonlar + koliler)."""
    layout = sl.load_yaml(LaunchConfiguration('stations_file').perform(context))
    params = sl.load_yaml(os.path.join(get_package_share_directory(PKG), 'config', 'station_params.yaml'))
    names = [st['name'] for st in layout['stations']] + [n for n, _, _ in sl.parcels(params, layout)]
    return [SetLaunchConfiguration('gui_wait_models', ','.join(names))]


def generate_launch_description():
    share = FindPackageShare('forklift_gazebo')
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([share, 'launch', 'sim.launch.py'])),
        launch_arguments={'world': PathJoinSubstitution([share, 'worlds', WORLD + '.sdf']),
                          'gui_wait_models': LaunchConfiguration('gui_wait_models')}.items(),
    )
    # Kapsamlı: stations.launch.py'nin params_file argümanı robotunkini ezmesin
    # (ikisi de 'params_file' adını kullanır).
    stations = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([share, 'launch', 'stations.launch.py'])),
        launch_arguments={
            'world': WORLD,
            'stations_file': LaunchConfiguration('stations_file'),
            'params_file': PathJoinSubstitution([share, 'config', 'station_params.yaml']),
        }.items(),
    )], scoped=True)
    return LaunchDescription([
        DeclareLaunchArgument(
            'stations_file', default_value=PathJoinSubstitution([share, 'config', 'stations.yaml'])),
        OpaqueFunction(function=_gui_wait_models),
        sim,
        stations,
    ])
