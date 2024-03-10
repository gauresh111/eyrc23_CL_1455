#!/usr/bin/python3
# -*- coding: utf-8 -*-



import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_prefix
import yaml


pkg_name='isro_robotic_arm_new'
def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def load_file(file_path):
    """Load the contents of a file into a string"""
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(file_path):
    """Load a yaml file into a dictionary"""
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def run_xacro(xacro_file):
    """Run xacro and output a file in the same directory with the same name, w/o a .xacro suffix"""
    urdf_file, ext = os.path.splitext(xacro_file)
    # if ext != '.xacro':
    #     raise RuntimeError(f'Input file to xacro must have a .xacro extension, got {xacro_file}')
    os.system(f'xacro {xacro_file} -o {urdf_file}')
    return urdf_file

def generate_launch_description():
    pkgPAth = get_package_file(pkg_name,'urdf/isro_robotic_arm_new.urdf')
    print("path:",pkgPAth)
    urdf_file = run_xacro(pkgPAth)
    robot_description_arm = load_file(urdf_file)
    params = {'robot_description' : robot_description_arm}
    robot_state_publisher_arm = Node(
        name='robot_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sime_time':True,'robot_description': robot_description_arm}]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='both',
        parameters=[params]
    )
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition = launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    # spawn_arm = Node(
    # 	package='gazebo_ros', 
    #     name='spawn_entity',
    # 	executable='spawn_entity.py',
    #     arguments=['-entity', 'ISRO_ARM', '-topic', '/robot_description', '-x', '0.0', '-y', '0.0', '-z', '0.58', '-Y', '3.14'],
    #     output='screen')
    rviz_node = Node( 
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True},{'robot_description': robot_description_arm}]
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='True',
            description='Flag to enable joint_state_publisher_gui'
        ),
        
        robot_state_publisher_arm,
        joint_state_publisher_node,
        # spawn_arm,
        joint_state_publisher_gui_node,
        rviz_node
        ]
    )
# def main():
#     pkgPAth = get_package_share_directory(pkg_name,'urdf/joel_gauresh_rdf.urdf')
#     print(pkgPAth)
