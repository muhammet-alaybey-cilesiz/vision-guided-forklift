"""Görev yöneticisi (config/autonomy.yaml; robot ölçüleri forklift_params.yaml).

  ros2 launch forklift_autonomy autonomy.launch.py [use_sim_time:=true] [transit_method:=sign|lane]
transit_method: sign (station_test, K4) | lane (factory, şerit takibi)
Algılama ayrıca: ros2 launch forklift_perception perception.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    files = {
        'config_file': os.path.join(get_package_share_directory('forklift_autonomy'), 'config', 'autonomy.yaml'),
        'robot_params_file': os.path.join(get_package_share_directory('forklift_description'), 'config',
                                          'forklift_params.yaml'),
    }
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('transit_method', default_value='sign', choices=['sign', 'lane']),
        Node(package='forklift_autonomy', executable='mission_manager', name='mission_manager', output='screen',
             parameters=[files, {'use_sim_time': sim_time, 'transit_method': LaunchConfiguration('transit_method')}]),
    ])
