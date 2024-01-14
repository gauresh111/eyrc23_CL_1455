#!/usr/bin/env python3

## Overview

# ###
# This ROS2 script is designed to control a robot's docking behavior with a rack. 
# It utilizes odometry data, ultrasonic sensor readings, and provides docking control through a custom service. 
# The script handles both linear and angular motion to achieve docking alignment and execution.
# ###

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
from usb_relay.srv import RelaySw
from std_srvs.srv import Trigger
import math
from threading import Thread
from rclpy.time import Time
from std_msgs.msg import Bool,Float32MultiArray,Float32
from sensor_msgs.msg import Imu
from linkattacher_msgs.srv import AttachLink , DetachLink
from std_msgs.msg import Bool
rclpy.init()
global robot_pose
global ultrasonic_value
ultrasonic_value = [0.0, 0.0]
robot_pose = [0.0, 0.0, 0.0,0.0]
def main():
    duplicateNode = Node('duplicate_node')
    executor = rclpy.executors.MultiThreadedExecutor(3)
    executor.add_node(duplicateNode)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    
    def sendUltaData(msg):
        ultraData = msg.data
    
    
    
    
    
    rclpy.spin(duplicateNode)
    duplicateNode.destroy_node()
    rclpy.shutdown()
    exit(0)

if __name__ == '__main__':
    main()

