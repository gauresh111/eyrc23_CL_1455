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
from scipy.signal import savgol_filter
import math
from threading import Thread
from rclpy.time import Time
from std_msgs.msg import Bool,Float32MultiArray,Float32
import time
global ultrasonic_value
ultrasonic_value = [0.0,0.0]
global sonicLeft,sonicRight
sonicLeft =[i for i in range(10)]

sonicRight =[i for i in range(10)]


class FilterUltraSonic(Node):

    def __init__(self):
        super().__init__('FilterUltraSonic')

        # self.copy_imu_sub = self.create_subscription(Imu, 'sensors/imu', self.imu_cb, 10)
        self.ultraSoincSub = self.create_subscription(Float32MultiArray, 'ultrasonic_sensor_std_float', self.ultraSonicCb, 1, callback_group=ReentrantCallbackGroup())
        self.ultraSoincPub = self.create_publisher(Float32MultiArray, 'ultrasonic_filtered', 10)
        self.controller_timer = self.create_timer(0.1, self.controller_loop)
        self.counter = 0
        self.averagel = 0.0
        self.averager = 0.0
    def ultraSonicCb(self, msg):
        global ultrasonic_value
        global sonicLeft,sonicRight
        alpha=0.05
        sonicLeft.pop(0)
        sonicRight.pop(0)
        sonicLeft.append(round(msg.data[4],4))
        sonicRight.append(round(msg.data[5],4))  
        ultrasonic_value[0] = round(msg.data[4],4)
        ultrasonic_value[1] = round(msg.data[5],4)
        def moving_average(data, n):
            return savgol_filter(data, window_length=n, polyorder=1)  # Use savgol_filter for efficiency

        filtered_data_left = moving_average(sonicLeft, 10)  # Window size of 5
        filtered_data_right = moving_average(sonicRight, 10)
        self.averagel = self.averagel*(1.0 - alpha) + alpha*ultrasonic_value[0]
        self.averager = self.averager*(1.0 - alpha) + alpha*ultrasonic_value[1]
        print(f' {self.counter} l {ultrasonic_value[0]} lf {filtered_data_left} :::: R {ultrasonic_value[1]} rf {filtered_data_right}')
        self.counter += 1
    def ultraSonicFilter(self):
        msg = Float32MultiArray()
        msg.data = [0.0,0.0,0.0,0.0,ultrasonic_value[0],ultrasonic_value[1]]
        self.ultrasonic_pub.publish(msg)
    def controller_loop(self):
        global ultrasonic_value
        global sonicLeft,sonicRight
        sl = sonicLeft.copy()
        sr = sonicRight.copy()
        sl.sort()
        sr.sort()
       
        # print(f' left UltraSonic {sl[0]} Right UltraSonic{sr[0]}   l{sonicLeft} R{sonicRight}')
        

def main(args=None):
    rclpy.init(args=args)
    try:
        node = FilterUltraSonic()
        executor = rclpy.executors.MultiThreadedExecutor(3)
        executor.add_node(node)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
        
    
    except KeyboardInterrupt:
        print("###### Keyboard interrupt detected. Closing script. ######")


if __name__ == '__main__':
    main()