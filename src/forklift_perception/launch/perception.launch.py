"""Algılama düğümleri: front_perception + lane_perception (config/perception.yaml).

Katalog (station_params.yaml), robot ölçüleri (forklift_params.yaml), algılayıcı eşikleri ve
kalibrasyon dizini paket share dizinlerinden verilir; düğümlerin kodda varsayılanı yok (D-29).
  ros2 launch forklift_perception perception.launch.py [debug:=true] [use_sim_time:=true]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('forklift_perception')
    files = {
        'catalog_file': os.path.join(get_package_share_directory('forklift_gazebo'), 'config', 'station_params.yaml'),
        'robot_params_file': os.path.join(get_package_share_directory('forklift_description'), 'config',
                                          'forklift_params.yaml'),
        'detectors_file': os.path.join(share, 'config', 'detectors.yaml'),
        'calibration_dir': os.path.join(share, 'config', 'calibration'),
    }
    params = os.path.join(share, 'config', 'perception.yaml')
    debug = ParameterValue(LaunchConfiguration('debug'), value_type=bool)
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    nodes = [Node(package='forklift_perception', executable=exe, name=exe, output='screen',
                  parameters=[params, files, {'publish_debug_images': debug, 'use_sim_time': sim_time}])
             for exe in ('front_perception', 'lane_perception')]
    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        *nodes,
    ])
