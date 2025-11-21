#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Chemin vers le package mobile_robot
    pkg_share = FindPackageShare("mobile_robot").find("mobile_robot")

    # Chemin vers le fichier XACRO du robot
    xacro_file = os.path.join(pkg_share, "urdf", "robot.xacro")
    
    # Générer la description du robot à partir du XACRO
    robot_description = {"robot_description": Command(["xacro", xacro_file])}

    # Chemin par défaut vers le monde Gazebo
    default_world = os.path.join(pkg_share, "worlds", "empty_world.sdf")

    return LaunchDescription([
        # Déclarer l'argument pour choisir le monde
        DeclareLaunchArgument(
            "world",
            default_value=default_world,
            description="Chemin vers le monde Gazebo à utiliser"
        ),

        # Lancer Gazebo via gz_sim
        ExecuteProcess(
            cmd=["gz", "sim", "-r", LaunchConfiguration("world")],
            output="screen"
        ),

        # Publier l'état du robot
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description]
        ),

        # Publier les états des joints via GUI
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
            parameters=[robot_description]
        ),
    ])

