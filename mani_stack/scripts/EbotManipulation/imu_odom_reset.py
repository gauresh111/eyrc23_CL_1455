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

from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from mani_stack.srv import DockSw  # Import custom service message
from std_srvs.srv import Trigger

from threading import Thread
from rclpy.time import Time
from std_msgs.msg import Bool
rclpy.init()
global robot_pose
global ultrasonic_value
from tf_transformations import euler_from_quaternion,quaternion_from_euler
ultrasonic_value = [0.0, 0.0]
robot_pose = [0.0, 0.0, 0.0,0.0]

class MyRobotDockingController(Node):

    def __init__(self):
        # Initialize the ROS2 node with a unique name
        super().__init__('my_robot_docking_reset')
        global robot_pose
        # Create a callback group for managing callbacks
        self.callback_group = ReentrantCallbackGroup()   
        self.reset_imu()                                    # Reset IMU data
        self.reset_odom()                                   # Reset Odom
    # Initialize a timer for the main control loop
        

    def reset_odom(self):
        self.get_logger().info('Resetting Odometry. Please wait...')
        self.reset_odom_ebot = self.create_client(Trigger, 'reset_odom')
        while not self.reset_odom_ebot.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/reset_odom service not available. Waiting for /reset_odom to become available.')
            
        self.request_odom_reset = Trigger.Request()
        self.odom_service_resp=self.reset_odom_ebot.call_async(self.request_odom_reset)
        # while self.odom_service_resp is None:
        #     self.GlobalStopTime(0.1)
        rclpy.spin_until_future_complete(self, self.odom_service_resp)
        if(self.odom_service_resp.result().success == True):
            self.get_logger().info(self.odom_service_resp.result().message)
        else:
            self.get_logger().warn(self.odom_service_resp.result().message)
    def GlobalStopTime(self,StopSeconds):
        future_time = Time(seconds=self.globalnodeClock.now().nanoseconds / 1e9 + StopSeconds, clock_type=self.globalnodeClock.clock_type)
        self.globalnodeClock.sleep_until(future_time)
    def reset_imu(self):
        self.get_logger().info('Resetting IMU. Please wait...')
        self.reset_imu_ebot = self.create_client(Trigger, 'reset_imu')
        while not self.reset_imu_ebot.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/reset_imu service not available. Waiting for /reset_imu to become available.')

        request_imu_reset = Trigger.Request()
        self.imu_service_resp=self.reset_imu_ebot.call_async(request_imu_reset)
        # while self.imu_service_resp is None:
        #     self.GlobalStopTime(0.1)
        rclpy.spin_until_future_complete(self, self.imu_service_resp)
        if(self.imu_service_resp.result().success== True):
            self.get_logger().info(self.imu_service_resp.result().message)
        else:
            self.get_logger().warn(self.imu_service_resp.result().message)

    

# Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    
    my_robot_docking_controller = MyRobotDockingController()
    executor = MultiThreadedExecutor(2)
    executor.add_node(my_robot_docking_controller)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    def ExitCallBack(msg):
        if msg.data == True:
            raise SystemExit
    exitDocking=my_robot_docking_controller.create_subscription(Bool, '/ExitNav',ExitCallBack, 10)
    try:
        rclpy.spin(my_robot_docking_controller)
    except SystemExit:
        print("SystemExit")
        my_robot_docking_controller.destroy_node()
        rclpy.shutdown()
        exit(0)
    my_robot_docking_controller.destroy_node()
    rclpy.shutdown()
    exit(0)
if __name__ == '__main__':
    main()