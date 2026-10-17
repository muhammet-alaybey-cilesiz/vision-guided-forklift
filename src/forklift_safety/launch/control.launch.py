"""Komut zinciri: twist_mux ve cmd_vel_gate (DECISIONS D-23, D-24, D-42).

Robot tarafında yaşar: forklift_gazebo/sim.launch.py bunu varsayılan olarak
başlatır (control:=true), böylece sim ile GUI ayrı terminallerde açılsa da
GUI'nin sürüş komutları DiffDrive'a ulaşır ve GUI kapanınca kapı ayakta kalır.

  /cmd_vel/teleop, /cmd_vel/auto --> twist_mux (+ /e_stop kilidi) --> input_topic
  input_topic --> cmd_vel_gate --> /cmd_vel --> ros_gz_bridge --> DiffDrive

twist_mux'ın çıkış topic'i cmd_vel_gate.yaml'daki input_topic'ten okunur; ad
tek yerde tanımlıdır.

Argümanlar:
  twist_mux_params : twist_mux.yaml yolu
  gate_params      : cmd_vel_gate.yaml yolu
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def _launch_setup(context):
    gate_params = LaunchConfiguration('gate_params').perform(context)
    with open(gate_params) as f:
        gate = yaml.safe_load(f)['cmd_vel_gate']['ros__parameters']

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[LaunchConfiguration('twist_mux_params')],
        remappings=[('cmd_vel_out', gate['input_topic'])],
        output='screen',
    )
    cmd_vel_gate = Node(
        package='forklift_safety',
        executable='cmd_vel_gate',
        name='cmd_vel_gate',
        parameters=[gate_params],
        output='screen',
    )
    return [twist_mux, cmd_vel_gate]


def generate_launch_description():
    share = FindPackageShare('forklift_safety')
    return LaunchDescription([
        DeclareLaunchArgument(
            'twist_mux_params',
            default_value=PathJoinSubstitution([share, 'config', 'twist_mux.yaml'])),
        DeclareLaunchArgument(
            'gate_params',
            default_value=PathJoinSubstitution([share, 'config', 'cmd_vel_gate.yaml'])),
        OpaqueFunction(function=_launch_setup),
    ])
