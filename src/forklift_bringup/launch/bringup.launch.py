"""Tüm sistemi başlatır: Gazebo (pencereli), RViz, komut zinciri ve operatör GUI'si.

Komut zinciri (twist_mux + cmd_vel_gate) forklift_gazebo/sim.launch.py içinden
başlar (DECISIONS D-42); bu dosya yalnız 'control' argümanını iletir.

Argümanlar:
  sim        : forklift_gazebo/sim.launch.py (varsayılan true)
  sim_gui    : Gazebo istemci penceresi (varsayılan true; false → kameralar
               EGL ile ekransız render edilir, görüntüler operatör GUI'sinde)
  rviz       : RViz (varsayılan true)
  control    : twist_mux + cmd_vel_gate, sim ile birlikte (varsayılan true)
  gui        : forklift_gui operatör arayüzü (varsayılan true)
  gui_params : forklift_gui parametre dosyası
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, NotSubstitution, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(package, launch_file, arguments, condition):
    # Kapsamlı grup ŞART: IncludeLaunchDescription argümanları üst bağlama
    # SetLaunchConfiguration ile yazar. Kapsamsız hâlde sim'e verilen
    # gui:=<sim_gui> bu dosyanın 'gui' argümanını ezer ve sim_gui:=false iken
    # operatör GUI'si hiç başlamazdı.
    return GroupAction(
        [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])),
            launch_arguments=arguments.items(),
        )],
        scoped=True,
        condition=condition,
    )


def generate_launch_description():
    sim_gui = LaunchConfiguration('sim_gui')
    declared_args = [
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('sim_gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('control', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument(
            'gui_params',
            default_value=PathJoinSubstitution(
                [FindPackageShare('forklift_gui'), 'config', 'gui_params.yaml'])),
    ]

    sim = _include(
        'forklift_gazebo', 'sim.launch.py',
        {'gui': sim_gui, 'headless': NotSubstitution(sim_gui),
         'rviz': LaunchConfiguration('rviz'),
         'control': LaunchConfiguration('control')},
        IfCondition(LaunchConfiguration('sim')))
    gui = _include(
        'forklift_gui', 'gui.launch.py',
        {'params_file': LaunchConfiguration('gui_params')},
        IfCondition(LaunchConfiguration('gui')))

    return LaunchDescription(declared_args + [sim, gui])
