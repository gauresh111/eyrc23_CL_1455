#!/usr/bin/env python3
import os
from nav_msgs.msg import Odometry
import rclpy
from threading import Thread
import time
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
import tf2_ros
from rclpy.duration import Duration # Handles time for ROS 2
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool
from ebot_docking.srv import RackSw  # Import custom service message
from tf_transformations import euler_from_quaternion
import math
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import Polygon,Point32
import yaml
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from std_msgs.msg import Bool
from rclpy.duration import Duration
from rclpy.time import Time
def main():
    rclpy.init()
    node = Node("moveBotYaml")
    def convert_nanoseconds_to_seconds(nanoseconds):
        return nanoseconds / 1e9
    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()
    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(3)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    nodeStopClock = node.get_clock()
    
    while True:
        future_time = Time(seconds=nodeStopClock.now().nanoseconds / 1e9 + 1, clock_type=nodeStopClock.clock_type)
        nodeStopClock.sleep_until(future_time)
        print("Time: ",nodeStopClock.now().nanoseconds / 1e9)
        time.sleep(1)
if __name__ == '__main__':
    main()