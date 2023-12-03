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
    navigator = BasicNavigator()
    node = Node("moveBot")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()
    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(1)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    global positionToGO
    positionToGO = {
        'initalPose':{'xyz': [0.0, 0.0, 0.0], 'quaternions': [0.0, 0.0, 0.0, 1.0], 'XYoffsets': [0.0, 0.0]},
        'rack1':{'xyz': [0.0, 4.4, 0.0], 'quaternions': [0.0, 0.0, 1.0, 0.0], 'XYoffsets': [1.26, 0.0]}, 
        'rack2':{'xyz': [2.03, 2.06, 0.0], 'quaternions': [0.0, 0.0, -0.7068252, 0.7073883], 'XYoffsets': [0.0, 1.24]},
        'rack3':{'xyz': [2.13, -7.09, 0.0], 'quaternions': [0.0, 0.0, 0.7068252, 0.7073883], 'XYoffsets': [0.0, -1.0]}, 
        'ap1':{'xyz': [0.0, -2.45, 0.0], 'quaternions': [0.0, 0.0, 1.0, 0.0], 'XYoffsets': [0.7, 0.0]}, 
        'ap2':{'xyz': [1.37, -4.15, 0.0], 'quaternions': [0.0, 0.0, -0.7068252, 0.7073883], 'XYoffsets': [0.0, 0.8]}, 
        'ap3':{'xyz': [1.37, -1.04, 0.0], 'quaternions': [0.0, 0.0, 0.7068252, 0.7073883], 'XYoffsets': [0.0, -0.72]}
            }
    def getGoalPoseStamped(goal):
        global positionToGO
        Goal = positionToGO[goal]
        goalPose = PoseStamped()
        goalPose.header.frame_id = 'map'
        goalPose.header.stamp = navigator.get_clock().now().to_msg()
        goalPose.pose.position.x = Goal['xyz'][0]
        goalPose.pose.position.y = Goal['xyz'][1]
        goalPose.pose.position.z = Goal['xyz'][2]
        goalPose.pose.orientation.x = Goal['quaternions'][0]
        goalPose.pose.orientation.y = Goal['quaternions'][1]
        goalPose.pose.orientation.z = Goal['quaternions'][2]
        goalPose.pose.orientation.w = Goal['quaternions'][3]
        print(goalPose)
        return goalPose  
    navigator.setInitialPose(getGoalPoseStamped("initalPose"))
    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()
    navigator.goToPose(getGoalPoseStamped("ap1"),"navigate_through_poses_w_replanning_and_recovery")
    # navigator.lifecycleShutdown()
    # time.sleep(15)
    # navigator.lifecycleStartup()
    # navigator.setInitialPose(getGoalPoseStamped("initalPose"))
    # # Wait for navigation to fully activate
    # navigator.waitUntilNav2Active()
if __name__ == '__main__':
    main()