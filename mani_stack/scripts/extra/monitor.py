#!/usr/bin/env python3

'''
# Team ID:          < 1455 >
# Theme:            < Cosmo Logistic >
# Author List:      < Joel Devasia , Gauresh Wadekar >
# Filename:         < docking_reset.py >
# Functions:        < main ,imu_callback ,odometry_callback,ultrasonic_callback>
# Global variables: < robot_pose , ultrasonic_value  >
'''


# Import necessary ROS2 packages and message types
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ebot_docking.srv import DockSw  # Import custom service message
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
import math
from threading import Thread
from rclpy.time import Time
from std_msgs.msg import Bool,Float32MultiArray,String
import yaml

rclpy.init()
global robot_pose
robot_pose = [0.0,0.0,0.0]
global ultrasonic_value
ultrasonic_value = [0.0, 0.0]
from tf_transformations import euler_from_quaternion,quaternion_from_euler


def main():
    MonitorNode = Node('monitor_node')
    callback_group = ReentrantCallbackGroup()
    executor = rclpy.executors.MultiThreadedExecutor(3)
    executor.add_node(MonitorNode)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    def imu_callback(msg):
        '''
        Purpose:read the IMU data and update the robot pose
        args: msg
        return: None
        
        
        '''
        global robot_pose
        quaternion_array = msg.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw_new = euler_from_quaternion(orientation_list)
        
        yaw = math.degrees(yaw_new)
        robot_pose[2] = round(yaw,2)  
    def odometry_callback(msg):
        # Extract and update robot pose information from odometry message
        '''
        Purpose:read the Odometry data and update the robot pose
        args: msg
        return: None
        
        
        '''
        global robot_pose
        robot_pose[0] = round(msg.pose.pose.position.x,2)
        robot_pose[1] = round(msg.pose.pose.position.y,2)
        
    def ultrasonic_callback(msg):
        '''
        Purpose:read the ultraSonic Value data and update the ultrasonic_value
        args: msg
        return: None
        '''
        global ultrasonic_value
        ultrasonic_value[0] = round(msg.data[4],4)
        ultrasonic_value[1] = round(msg.data[5],4)  
    MonitorNode.odom = MonitorNode.create_subscription(Odometry, '/odom', odometry_callback, 10,callback_group=callback_group)
    MonitorNode.ultra_sub = MonitorNode.create_subscription(Float32MultiArray, '/ultrasonic_filter', ultrasonic_callback, 10,callback_group=callback_group)
    MonitorNode.imu_sub = MonitorNode.create_subscription(Imu, 'sensors/imu1', imu_callback, 10,callback_group=callback_group)
    while True:
        print(f"robot_pose: X = {robot_pose[0]}, Y = {robot_pose[1]}, filter: lu = {ultrasonic_value[0]}, ru = {ultrasonic_value[1]}, IMU = {robot_pose[2]}")

        # rclpy.spin_(duplicateNode)
    MonitorNode.destroy_node()
    rclpy.shutdown()
    exit(0)

if __name__ == '__main__':
    main()

