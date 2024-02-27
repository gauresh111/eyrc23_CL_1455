#!/usr/bin/env python3
'''
# Team ID:          < 1455 >
# Theme:            < Cosmo Logistic >
# Author List:      < Joel Devasia , Gauresh Wadekar >
# Filename:         < sensor_filter.launch.py >
# Functions:        < generate_launch_description >
# Global variables: < none >
'''

from launch.actions import ExecuteProcess
from launch import LaunchDescription
def generate_launch_description():
    '''
    Purpose: to launch the perception.py, duplicateImu.py and ultraFilter.py
    args: None
    return: LaunchDescription
    '''
    
    
    Start_perceiption = ExecuteProcess( 
                                       cmd=[[
            'ros2 run mani_stack perception.py',
        ]],
        shell=True
    )
    Start_Imu = ExecuteProcess(
        cmd=[[
            'ros2 run mani_stack duplicateImu.py',
        ]],
        shell=True
    )
    Start_ultraFilter = ExecuteProcess(
        cmd=[[
            'ros2 run mani_stack ultraFilter.py',
        ]],
        shell=True
    )
    
    return LaunchDescription([
    Start_perceiption,
    Start_Imu,
    Start_ultraFilter
    ])