"""Gazebo Sim'de lift_test dünyasını başlatır, forklift'i spawn eder ve ROS köprülerini kurar.

Başlatılanlar:
  gz sim (ros_gz_sim/gz_sim.launch.py), robot_state_publisher (use_sim:=true),
  ros_gz_sim create (spawn), ros_gz_bridge parameter_bridge (config/ros_gz_bridge.yaml),
  ros_gz_image image_bridge (iki kamera görüntüsü), isteğe bağlı rviz2 ve
  komut zinciri (forklift_safety/control.launch.py: twist_mux + cmd_vel_gate).
Simülasyon düğümleri use_sim_time: true ile çalışır; komut zinciri duvar
saatiyle (D-25).

Komut zinciri robot tarafındadır (DECISIONS D-42): GUI ayrı terminalde açılsa da
/cmd_vel/teleop → /cmd_vel yolu hazırdır, GUI kapanınca kapı sıfır komutu sürdürür.

Argümanlar:
  world        : dünya SDF dosyası (varsayılan worlds/lift_test.sdf)
  gui          : true → Gazebo GUI; false → yalnızca sunucu (-s)
  headless     : true → sunucuda ekransız render (--headless-rendering, EGL)
  rviz         : RViz'i aç (sabit frame odom)
  control      : twist_mux + cmd_vel_gate komut zinciri (varsayılan true; zinciri
                 ayrıca başlatan testler false verir)
  x, y, z, yaw : spawn pozu [m, rad]; varsayılan orijin, +x'e bakış
  params_file  : forklift parametre dosyası (görüntü topic adları da buradan okunur)
  verbosity    : gz günlük seviyesi (0-4)

Model yolu: package.xml'deki <gazebo_ros gazebo_model_path="${prefix}/models"/>
dışa aktarımını ros_gz_sim GZ_SIM_RESOURCE_PATH'e ekler (model://test_parcel).
"""

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

MODEL_NAME = 'forklift'


def _launch_setup(context):
    """Metin değerine ihtiyaç duyan adımlar (gz argümanları, YAML'dan topic adları)."""
    world = LaunchConfiguration('world').perform(context)
    gz_args = ['-r', '-v', LaunchConfiguration('verbosity').perform(context), world]
    if LaunchConfiguration('gui').perform(context).lower() != 'true':
        gz_args.append('-s')
    if LaunchConfiguration('headless').perform(context).lower() == 'true':
        gz_args.append('--headless-rendering')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': ' '.join(gz_args), 'on_exit_shutdown': 'true'}.items(),
    )

    # Görüntü topic'leri tek kaynaktan: forklift_params.yaml → cameras.*.topic
    with open(LaunchConfiguration('params_file').perform(context)) as f:
        cameras = yaml.safe_load(f)['cameras']
    image_topics = [cam['topic'] for cam in cameras.values()]

    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='image_bridge',
        arguments=image_topics,
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    return [gz_sim, image_bridge]


def generate_launch_description():
    desc_share = FindPackageShare('forklift_description')
    sim_share = FindPackageShare('forklift_gazebo')

    declared_args = [
        DeclareLaunchArgument('world', default_value=PathJoinSubstitution([sim_share, 'worlds', 'lift_test.sdf'])),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('control', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([desc_share, 'config', 'forklift_params.yaml'])),
        DeclareLaunchArgument('verbosity', default_value='3'),
    ]

    robot_description = ParameterValue(
        Command([
            'xacro ', PathJoinSubstitution([desc_share, 'urdf', 'forklift.urdf.xacro']),
            ' use_sim:=true',
            ' params_file:=', LaunchConfiguration('params_file'),
        ]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', MODEL_NAME,
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([sim_share, 'config', 'ros_gz_bridge.yaml']),
            'use_sim_time': True,
        }],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([desc_share, 'rviz', 'forklift.rviz']), '-f', 'odom'],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    # Kapsamlı grup: control.launch.py'nin argümanları bu dosyaya sızmasın.
    control = GroupAction(
        [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('forklift_safety'), 'launch', 'control.launch.py'])))],
        scoped=True,
        condition=IfCondition(LaunchConfiguration('control')),
    )

    return LaunchDescription(declared_args + [
        OpaqueFunction(function=_launch_setup),
        robot_state_publisher,
        spawn,
        bridge,
        rviz,
        control,
    ])
