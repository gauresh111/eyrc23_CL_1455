#!/usr/bin/python3
# -*- coding: utf-8 -*-

''' 
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
'''

import launch
import launch_ros
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def generate_launch_description():
    

    spawn_arm = launch_ros.actions.Node(
    	package='gazebo_ros', 
        name='spawn_entity',
    	executable='spawn_entity.py',
        arguments=['-entity', 'robotic_arm', '-topic', '/robot_description', '-x', '0.0', '-y', '0.0', '-z', '0.0', '-Y', '3.14'],
        output='screen')

                                                 
    return launch.LaunchDescription([
        ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'], output='screen'),
        
        spawn_arm
    ])
