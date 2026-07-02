#!/usr/bin/env python3
"""
Visualizza il modello Genghis in RViz con gli slider dei giunti.

Avvia 3 nodi:
  - robot_state_publisher : legge l'URDF e pubblica le TF dei link
  - joint_state_publisher_gui : finestra con uno slider per ogni giunto mobile
  - rviz2 : visualizzatore 3D (config genghis.rviz)

Uso (sulla macchina Linux con ROS2 Humble + desktop):
    ros2 launch <percorso>/display.launch.py
oppure, dalla cartella description:
    ros2 launch ./display.launch.py

NB: non e' un pacchetto ROS installato, quindi si lancia per PERCORSO diretto.
Dipendenze: ros-humble-robot-state-publisher, ros-humble-joint-state-publisher-gui,
ros-humble-rviz2 (tutte incluse in ros-humble-desktop).
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(here, "genghis.urdf")
    rviz_path = os.path.join(here, "genghis.rviz")

    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    rviz_args = ["-d", rviz_path] if os.path.exists(rviz_path) else []

    return LaunchDescription([
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
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=rviz_args,
        ),
    ])
