"""Robot tanımını RViz'de gösterir.

Başlatılanlar: robot_state_publisher + joint_state_publisher(_gui) + rviz2.

Argümanlar:
  gui         : true → joint_state_publisher_gui (kaydırıcılar), false → joint_state_publisher
  rviz        : RViz'i başlat
  fork_height : başlangıç çatal yüksekliği [m] (fork_lift_joint; yalnız prefix boşken etkili)
  prefix      : frame/eklem ön eki
  params_file : parametre YAML dosyası
  rviz_config : RViz yapılandırması
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('forklift_description')

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    fork_height = LaunchConfiguration('fork_height')
    prefix = LaunchConfiguration('prefix')
    params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')

    declared_args = [
        DeclareLaunchArgument('gui', default_value='true',
                              description='joint_state_publisher_gui kullan'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz başlat'),
        DeclareLaunchArgument('fork_height', default_value='0.0',
                              description='Başlangıç çatal yüksekliği [m]'),
        DeclareLaunchArgument('prefix', default_value='', description='Frame/eklem ön eki'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'forklift_params.yaml']),
            description='Parametre YAML dosyası'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([pkg_share, 'rviz', 'forklift.rviz']),
            description='RViz yapılandırması'),
    ]

    # Simülatörden bağımsız tanım (use_sim:=false)
    robot_description = ParameterValue(
        Command([
            'xacro ', PathJoinSubstitution([pkg_share, 'urdf', 'forklift.urdf.xacro']),
            ' use_sim:=false',
            ' prefix:=', prefix,
            ' params_file:=', params_file,
        ]),
        value_type=str)

    # Başlangıç eklem değeri; diğer eklemler 0
    joint_zeros = {
        'zeros.fork_lift_joint': ParameterValue(fork_height, value_type=float),
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[joint_zeros],
        condition=UnlessCondition(gui),
        output='screen',
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[joint_zeros],
        condition=IfCondition(gui),
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription(declared_args + [
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        rviz_node,
    ])
