#!/usr/bin/env python3

## Overview

# ###
# This ROS2 script is designed to control a robot's docking behavior with a rack. 
# It utilizes odometry data, ultrasonic sensor readings, and provides docking control through a custom service. 
# The script handles both linear and angular motion to achieve docking alignment and execution.
# ###

# Import necessary ROS2 packages and message types

import rclpy
from rclpy.node import Node
from threading import Thread
from std_msgs.msg import Float32MultiArray

rclpy.init()
global heading
global ultrasonic_value , readings , Leftreadings , Rightreadings , leftTotal , rightTotal , readIndex
Leftreadings = [0 for i in range(20)]
Rightreadings = [0 for i in range(20)]
leftTotal = 0
rightTotal = 0
ultrasonic_value = [0.0, 0.0]
heading = 0.0
readIndex = 0

def main():
    duplicateNode = Node('duplicate_node')
    executor = rclpy.executors.MultiThreadedExecutor(3)
    executor.add_node(duplicateNode)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    
    def ultrasonic_callback(msg):
            global ultrasonic_value
            ultrasonic_value[0] = round(msg.data[4],4)
            ultrasonic_value[1] = round(msg.data[5],4)  
    duplicateNode.ultra_sub = duplicateNode.create_subscription(Float32MultiArray, 'ultrasonic_sensor_std_float', ultrasonic_callback, 10)
    duplicateNode.ultrasonic_pub = duplicateNode.create_publisher(Float32MultiArray, '/ultrasonic_filter', 1)
    
    def movingAverage():
        global readings, ultrasonic_value, Leftreadings, Rightreadings, leftTotal, rightTotal ,readIndex
        average =0
        
        leftTotal = leftTotal - Leftreadings[readIndex]
        rightTotal -= Rightreadings[readIndex]
        
        
        Leftreadings[readIndex] = ultrasonic_value[0]
        Rightreadings[readIndex] = ultrasonic_value[1]
        
        
        leftTotal = leftTotal + Leftreadings[readIndex]
        rightTotal = rightTotal + Rightreadings[readIndex]
        
        readIndex = readIndex + 1
        if (readIndex >= 20): 
            readIndex = 0
        leftaverage = leftTotal / 20
        rightaverage = rightTotal / 20
        return leftaverage, rightaverage
    def ultrasonic_publisher():
        
        left,right = movingAverage()
        msg = Float32MultiArray()
        minValue = min(left,right)
        msg.data = [0.0,0.0,0.0,0.0,minValue,minValue]
        duplicateNode.ultrasonic_pub.publish(msg)
        
        
        
    rate = duplicateNode.create_rate(5, duplicateNode.get_clock())
    while True:
        ultrasonic_publisher()
        rate.sleep()
        # rclpy.spin_(duplicateNode)
    duplicateNode.destroy_node()
    rclpy.shutdown()
    exit(0)

if __name__ == '__main__':
    main()

