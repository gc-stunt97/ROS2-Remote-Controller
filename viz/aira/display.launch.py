#!/usr/bin/env python3
"""RViz della base mobile AIRA sul CONTROLLER (modello + odometria).

Carica l'URDF PIATTO di AIRA (viz/aira/aira_base.urdf, copia generata dallo xacro del
repo AIRA_Robot) e avvia robot_state_publisher + rviz2. Il modello si muove perche' il
fixed frame e' `odom` e il robot pubblica TF odom->base_link via /odom.

    ros2 launch ./display.launch.py            # slider per girare le ruote a mano
    ros2 launch ./display.launch.py gui:=false # senza slider (usa /joint_states del robot)

Dipendenze: ros-humble-robot-state-publisher, ros-humble-joint-state-publisher-gui,
ros-humble-rviz2 (in ros-humble-desktop).
NB: viz/ non e' un pacchetto ROS: si lancia per percorso diretto.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(here, "aira_base.urdf")
    rviz_path = os.path.join(here, "aira_base.rviz")

    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    rviz_args = ["-d", rviz_path] if os.path.exists(rviz_path) else []
    gui = LaunchConfiguration("gui")

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
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
