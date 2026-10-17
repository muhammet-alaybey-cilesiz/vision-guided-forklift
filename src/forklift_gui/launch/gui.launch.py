"""forklift_gui operatör arayüzünü başlatır.

Argümanlar:
  mock        : true -> ROS grafiği olmadan sahte verilerle açılır (--mock)
  params_file : gui_params.yaml yolu (varsayılan: paketin config/ dosyası)

GUI düğümü duvar saatiyle çalışır (use_sim_time: false, DECISIONS D-25);
sim zamanını /clock'tan okur.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mock = LaunchConfiguration('mock')
    params_file = LaunchConfiguration('params_file')

    declared_args = [
        DeclareLaunchArgument('mock', default_value='false'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('forklift_gui'), 'config', 'gui_params.yaml'])),
    ]

    gui = Node(
        package='forklift_gui',
        executable='forklift_gui',
        name='forklift_gui',
        parameters=[params_file],
        output='screen',
        condition=UnlessCondition(mock),
    )

    gui_mock = Node(
        package='forklift_gui',
        executable='forklift_gui',
        name='forklift_gui',
        arguments=['--mock'],
        parameters=[params_file],
        output='screen',
        condition=IfCondition(mock),
    )

    return LaunchDescription(declared_args + [gui, gui_mock])
