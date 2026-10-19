"""station_test dünyası: sim.launch.py (robot, köprüler, komut zinciri) + stations.launch.py.

sim.launch.py'nin tüm argümanları (gui, headless, rviz, control, x, y, z, yaw,
params_file, verbosity) komut satırından aynen geçer; burada yalnızca dünya
dosyası station_test.sdf'e sabitlenir. İstasyon/koli yerleşimi
config/stations.yaml'dan gelir (stations_file argümanıyla değiştirilebilir).

  ros2 launch forklift_gazebo station_sim.launch.py
  ros2 launch forklift_bringup bringup.launch.py world:=station_test   (aynı dünya + RViz + GUI)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

WORLD = 'station_test'


def generate_launch_description():
    share = FindPackageShare('forklift_gazebo')
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([share, 'launch', 'sim.launch.py'])),
        launch_arguments={'world': PathJoinSubstitution([share, 'worlds', WORLD + '.sdf'])}.items(),
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
        sim,
        stations,
    ])
