#!/usr/bin/env python3

from os import path
from threading import Thread

import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from std_msgs.msg import String

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
import tf2_ros

from sensor_msgs.msg import JointState

aruco_name_list = []
joint_states = [0,0,0,0,0,0]

class PredefinedJointStates:
    def __init__(self):
        self.joint_states = None
        self.name = None

class ArucoNameCoordinate:
    def __init__(self):
        self.name = None
        self.position = None
        self.quaternions = None

class ArucoBoxPose:
    def __init__(self):
        self.position = None
        self.quaternions = None

def aruco_name_list_updater(msg):
    global aruco_name_list
    aruco_name_list = msg.data.split()

def joint_states_updater(msg):
    global joint_states
    joint_states = list([states for states in msg.position])
    print(joint_states)


def main():
    rclpy.init()

    Initial_Pose = ArucoBoxPose()
    Initial_Pose.position = [0.18, 0.10, 0.46]
    Initial_Pose.quaternions = [0.50479, 0.495985, 0.499407, 0.499795]

    P1 = ArucoBoxPose()
    P1.position = [0.35, 0.1, 0.68]
    P1.quaternions = [0.50479, 0.495985, 0.499407, 0.499795]

    P2 = ArucoBoxPose()
    P2.position = [0.194, -0.43, 0.701]
    P2.quaternions = [ 0.7657689, 0.0, 0.0, 0.6431158 ]

    Drop = ArucoBoxPose()
    Drop.position = [-0.37, 0.12, 0.397]
    Drop.quaternions = [ 0.5414804, -0.4547516, -0.5414804, 0.4547516 ]


    Initial_Joints = PredefinedJointStates()
    Initial_Joints.joint_states = [0.0, -2.39, 2.4, -3.15, -1.58, 3.15]
    Initial_Joints.name = "Initial_Joints"

    Drop_Joints = PredefinedJointStates()
    Drop_Joints.joint_states = [-3.147, -2.094, 2.09, -3.135, -1.566, 6.127]
    Drop_Joints.name = "Drop_Joints"

    global aruco_name_list
    global joint_states

    # Create node for this example
    node = Node("pick_aruco")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    tf_buffer = tf2_ros.buffer.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)
    aruco_name_subscriber = node.create_subscription(
            String, "/aruco_list", aruco_name_list_updater, 10
        )
    joint_states_subscriber = node.create_subscription(
        JointState, "/joint_states", joint_states_updater, 10
    )
    i = 0
    # while i < 5:
    #     print("Moving to Drop position")
    #     moveit2.move_to_configuration(Drop_Joints.joint_states)
    #     moveit2.wait_until_executed()
    #     time.sleep(2)
    #     print("Moving to Initial position")
    #     moveit2.move_to_configuration(Initial_Joints.joint_states)
    #     moveit2.wait_until_executed()
    #     time.sleep(2)   
    #     i += 1 
    
    while True:
        a = 1+3
        # print("Running Infinite Loop")
    print("Done")
    rclpy.spin(node)
    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
