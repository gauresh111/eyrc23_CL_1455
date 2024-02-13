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
from ebot_docking.srv import RelaySw
from std_srvs.srv import Trigger
import math
from threading import Thread
from rclpy.time import Time
from std_msgs.msg import Bool,Float32MultiArray,Float32
from sensor_msgs.msg import Imu
from linkattacher_msgs.srv import AttachLink , DetachLink
from std_msgs.msg import Bool
from sensor_msgs.msg import Range
from tf_transformations import euler_from_quaternion
import random
rclpy.init()
global heading
global ultrasonic_value
ultrasonic_value = [0.0, 0.0]
heading = 0.0
counter = random.randint(0, 100)
def main():
    duplicateNode = Node('duplicate_node')
    executor = rclpy.executors.MultiThreadedExecutor(3)
    executor.add_node(duplicateNode)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    
    def ultrasonic_rl_callback(msg):
            global ultrasonic_value
            ultrasonic_value[0] = round(msg.range,3)
            ultrasonic_value[0] = ultrasonic_value[0]*100

    def ultrasonic_rr_callback(msg):
        global ultrasonic_value
        ultrasonic_value[1] = round(msg.range,3)
        ultrasonic_value[1] = ultrasonic_value[1]*100
    def imu_callback(msg):
        global heading
        quaternion_array = msg.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        yaw = math.degrees(yaw)
        heading = round(yaw,2)
    def usb_relay_sw_callback(request, response):
        print("usb_relay_sw_callback")
        result = False
        if request.relaystate == True:
            req = AttachLink.Request()
            req.model1_name =  'ebot'     
            req.link1_name  = 'ebot_base_link'       
            req.model2_name =  request.rackname       
            req.link2_name  = 'link'
            duplicateNode.future = duplicateNode.link_attach_cli.call_async(req) 
            result = True
            print("Rack attached")
        else:
        
            req = DetachLink.Request()
            req.model1_name =  'ebot'     
            req.link1_name  = 'ebot_base_link'       
            req.model2_name =  request.rackname       
            req.link2_name  = 'link'  
            duplicateNode.future = duplicateNode.lind_detached_cli.call_async(req)
            print("Rack detached")
        if not result:
            result="Unable to connect to board of the relay"
            response.success = False
            response.message = result
        else:
            result = "success"        
            response.success = True
            response.message = str(result)
            #self.get_logger().info('Incoming request',request.data)
        return response
    
    duplicateNode.ultrasonic_rl_sub = duplicateNode.create_subscription(Range, '/ultrasonic_rl/scan', ultrasonic_rl_callback, 10)
    duplicateNode.ultrasonic_rr_sub = duplicateNode.create_subscription(Range, '/ultrasonic_rr/scan', ultrasonic_rr_callback, 10)
    duplicateNode.ultrasonic_pub = duplicateNode.create_publisher(Float32MultiArray, '/ultrasonic_sensor_std_float', 1)
    duplicateNode.imu_sub = duplicateNode.create_subscription(Imu, '/imu', imu_callback, 10)
    duplicateNode.imu_pub = duplicateNode.create_publisher(Float32, '/orientation', 10)
    duplicateNode.srv = duplicateNode.create_service(RelaySw, 'usb_relay_sw', usb_relay_sw_callback)
    duplicateNode.link_attach_cli = duplicateNode.create_client(AttachLink, '/ATTACH_LINK')
    duplicateNode.lind_detached_cli = duplicateNode.create_client(DetachLink, '/DETACH_LINK')
    def ultrasonic_publisher():
        global ultrasonic_value
        global heading
        global counter
        msg = Float32MultiArray()
        if counter%(10)== 0:
            valueleft = random.randint(0, 100)
            valueright = random.randint(0, 100)
            ultrasonic_value[0] = valueleft.__float__()
            ultrasonic_value[1] = valueright.__float__()
         
        msg.data = [0.0,0.0,0.0,0.0,ultrasonic_value[0],ultrasonic_value[1]]
        duplicateNode.ultrasonic_pub.publish(msg)
        counter=counter+1
        
        msg = Float32()
        msg.data = heading
        duplicateNode.imu_pub.publish(msg)
    rate = duplicateNode.create_rate(2, duplicateNode.get_clock())
    while True:
        ultrasonic_publisher()
        rate.sleep()
        # rclpy.spin_(duplicateNode)
    duplicateNode.destroy_node()
    rclpy.shutdown()
    exit(0)

if __name__ == '__main__':
    main()

