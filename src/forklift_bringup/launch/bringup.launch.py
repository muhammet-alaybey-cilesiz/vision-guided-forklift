"""Tüm sistemi başlatır: Gazebo (pencereli), RViz, komut zinciri ve operatör GUI'si.

Komut zinciri (twist_mux + cmd_vel_gate) forklift_gazebo/sim.launch.py içinden
başlar (DECISIONS D-42); bu dosya yalnız 'control' argümanını iletir.

Argümanlar:
  sim        : simülasyonu başlat (varsayılan true)
  world      : lift_test (varsayılan; forklift_gazebo/sim.launch.py) |
               station_test | factory (forklift_gazebo/station_sim.launch.py:
               istasyonlar ve koliler config/stations.yaml'dan; factory fabrika
               holüdür, config/factory.yaml)
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
from launch.substitutions import (
    AndSubstitution, EqualsSubstitution, LaunchConfiguration, NotSubstitution, PathJoinSubstitution)
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
        DeclareLaunchArgument('world', default_value='lift_test', choices=['lift_test', 'station_test', 'factory']),
        DeclareLaunchArgument('sim_gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('control', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument(
            'gui_params',
            default_value=PathJoinSubstitution(
                [FindPackageShare('forklift_gui'), 'config', 'gui_params.yaml'])),
    ]

    # world açıkça dosya yolu olarak verilir: kapsamlı grup üst bağlamdaki
    # world (çıplak ad, ör. 'lift_test') değerini miras alır ve sim.launch.py'nin
    # varsayılan yolunu ezerdi. lift_test için yol, sim.launch.py varsayılanıyla aynı.
    sim_args = {'gui': sim_gui, 'headless': NotSubstitution(sim_gui),
                'rviz': LaunchConfiguration('rviz'),
                'control': LaunchConfiguration('control'),
                'world': PathJoinSubstitution([FindPackageShare('forklift_gazebo'), 'worlds',
                                               [LaunchConfiguration('world'), '.sdf']])}

    def _sim_if(world):
        return IfCondition(AndSubstitution(
            LaunchConfiguration('sim'), EqualsSubstitution(LaunchConfiguration('world'), world)))

    # Varsayılan dünya değişmedi: lift_test → sim.launch.py, kendi varsayılan dünyasıyla
    sim = _include('forklift_gazebo', 'sim.launch.py', sim_args, _sim_if('lift_test'))
    station_sim = _include('forklift_gazebo', 'station_sim.launch.py',
                           {**sim_args, 'station_world': 'station_test'}, _sim_if('station_test'))
    factory_sim = _include('forklift_gazebo', 'station_sim.launch.py',
                           {**sim_args, 'station_world': 'factory'}, _sim_if('factory'))
    gui = _include(
        'forklift_gui', 'gui.launch.py',
        {'params_file': LaunchConfiguration('gui_params')},
        IfCondition(LaunchConfiguration('gui')))

    return LaunchDescription(declared_args + [sim, station_sim, factory_sim, gui])
