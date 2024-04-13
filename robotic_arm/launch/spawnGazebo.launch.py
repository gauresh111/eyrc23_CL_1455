#!/usr/bin/python3
# -*- coding: utf-8 -*-

""" 
*****************************************************************************************
*
*        =============================================
*                  TBD Theme (eYRC 2023-24)
*        =============================================
*
*
*  Filename:			ebot_display_launch.py
*  Description:         Use this file to spawn ebot inside e-yantra warehouse world in the gazebo simulator and publish robot states.
*  Created:				16/07/2023
*  Last Modified:	    16/07/2023
*  Modified by:         Archit, Jaison
*  Author:				e-Yantra Team
*  
*****************************************************************************************
"""

import launch
import launch_ros
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_prefix


def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path


pkg_name = "robotic_arm"


def generate_launch_description():

    spawn_arm = launch_ros.actions.Node(
        package="gazebo_ros",
        name="spawn_entity",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "robotic_arm",
            "-topic",
            "/robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
            "-Y",
            "0.0",
        ],
        output="screen",
    )

    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_models_dir = get_package_share_directory(pkg_name)

    install_dir = get_package_prefix(pkg_name)

    if "GAZEBO_MODEL_PATH" in os.environ:
        gazebo_models_path = os.path.join(pkg_models_dir, "models")
        os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    else:
        os.environ["GAZEBO_MODEL_PATH"] = install_dir + "/share"

    if "GAZEBO_PLUGIN_PATH" in os.environ:
        os.environ["GAZEBO_PLUGIN_PATH"] = (
            os.environ["GAZEBO_PLUGIN_PATH"] + ":" + install_dir + "/lib"
        )
    else:
        os.environ["GAZEBO_PLUGIN_PATH"] = install_dir + "/lib"

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py"),
        )
    )

    # return launch.LaunchDescription(
    #     [
    #         DeclareLaunchArgument(
    #             "world",
    #             default_value=[
    #                 os.path.join(
    #                     get_package_share_directory("robotic_arm"),
    #                     "worlds",
    #                     "arm_world.world",
    #                 ),
    #                 "",
    #             ],  # Change name of world file if required.
    #             description="SDF world file",
    #         ),
    #         gazebo,
    #         # ExecuteProcess(cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen'),
    #         # spawn_arm
    #     ]
    # )
    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_models_dir, 'worlds', 'arm_world.world'), ''], # Change name of world file if required.
          description='SDF world file'),
        gazebo,
        spawn_arm
        # ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'], output='screen'),
    ])
