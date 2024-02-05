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
from mani_stack.srv import DockSw  # Import custom service message

from usb_relay.srv import RelaySw # type: ignore
from std_srvs.srv import Trigger
import math
from threading import Thread
from rclpy.time import Time
from std_msgs.msg import Bool,Float32MultiArray,Float32
global ultrasonic_value
ultrasonic_value = [0.0,0.0]
rclpy.init()



class FilterUltraSonic(Node):

    def __init__(self):
        super().__init__('FilterUltraSonic')

        # self.copy_imu_sub = self.create_subscription(Imu, 'sensors/imu', self.imu_cb, 10)
        self.ultraSoincSub = self.create_subscription(Float32MultiArray, 'ultrasonic_sensor_std_float', self.ultraSonicCb, 10)
        self.ultraSoincPub = self.create_publisher(Float32MultiArray, 'ultrasonic_filtered', 10)
        self.controller_timer = self.create_timer(0.1, self.controller_loop)
    def ultraSonicCb(self, msg):
        global ultrasonic_value
        ultrasonic_value[0] = round(msg.data[4],4)
        ultrasonic_value[1] = round(msg.data[5],4)  
        print(f'Ultrasonic = {ultrasonic_value}')
    def ultraSonicFilter(self):
        msg = Float32MultiArray()
        msg.data = [0.0,0.0,0.0,0.0,ultrasonic_value[0],ultrasonic_value[1]]
        self.ultrasonic_pub.publish(msg)
    def controller_loop(self):
        # global ultrasonic_value
        # for i  in range (10):
        #     print(f'Ultrasonic = {ultrasonic_value}')
        1+1

def main(args=None):
    rclpy.init(args=args)
    try:
        node = FilterUltraSonic()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("###### Keyboard interrupt detected. Closing script. ######")


if __name__ == '__main__':
    main()