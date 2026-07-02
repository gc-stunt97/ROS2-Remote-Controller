#!/usr/bin/env python3
"""
Visualizza il modello Genghis in RViz.

Argomento `gui` (default true):
  - gui:=true  -> avvia joint_state_publisher_gui (slider manuali sui giunti).
  - gui:=false -> NON avvia gli slider: /joint_states arriva da un comando ESTERNO
                  (es. il nodo `teleop` sul robot). Usare questo per la teleop.

Avvia inoltre robot_state_publisher (URDF -> TF) e rviz2 (config genghis.rviz).

Uso (macchina Linux con ROS2 Humble + desktop):
    ros2 launch ./display.launch.py              # slider manuali
    ros2 launch ./display.launch.py gui:=false   # comando esterno (teleop)

NB: non e' un pacchetto ROS installato, quindi si lancia per PERCORSO diretto.
Dipendenze: ros-humble-robot-state-publisher, ros-humble-joint-state-publisher-gui,
ros-humble-rviz2 (tutte incluse in ros-humble-desktop).
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(here, "genghis.urdf")
    rviz_path = os.path.join(here, "genghis.rviz")

    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    rviz_args = ["-d", rviz_path] if os.path.exists(rviz_path) else []
    gui = LaunchConfiguration("gui")

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="true=slider manuali; false=comando esterno (teleop) su /joint_states",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
            condition=IfCondition(gui),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=rviz_args,
        ),
    ])
