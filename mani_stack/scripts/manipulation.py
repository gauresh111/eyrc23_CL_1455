#!/usr/bin/env python3

is_sim = True

from os import path
from threading import Thread
import time
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from pymoveit2 import MoveIt2, MoveIt2Servo
from pymoveit2.robots import robotic_arm
import tf2_ros
import math
import re
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
import transforms3d as tf3d
import numpy as np
from std_msgs.msg import Bool
import yaml

from tf_transformations import quaternion_from_euler, euler_from_quaternion

aruco_name_list = []
aruco_angle_list = []
aruco_ap_list = []
global servo_status
servo_status = 5
current_joint_states = [0, 0, 0, 0, 0, 0]


class ArucoData:
    def __init__(self):
        self.name = None
        self.id = None
        self.position = None
        self.quaternions = None
        self.yaw = None
        self.yaw_error = None
        self.ap = None
        self.rotation_name = None


class ArucoBoxPose:
    def __init__(self):
        self.position = None
        self.quaternions = None


class PredefinedJointStates:
    def __init__(self):
        self.joint_states = None
        self.name = None


def aruco_name_list_updater(msg):
    global aruco_name_list
    # aruco_name_list = msg.data.split()


def aruco_data_updater(msg):
    global aruco_name_list
    global aruco_angle_list
    global aruco_ap_list
    data = yaml.safe_load(msg.data)
    aruco_name_list = data.get("id")
    aruco_angle_list = data.get("angle")
    aruco_ap_list = data.get("ap")


def main():
    rclpy.init()

    if is_sim == True:
        print("Starting Simulation Manipulation Script")
    else:
        print("Starting Real Robot Manipulation Script")

    Initial_Pose = ArucoBoxPose()
    Initial_Pose.position = [0.18, 0.10, 0.46]
    Initial_Pose.quaternions = [0.50479, 0.495985, 0.499407, 0.499795]

    Initial_Joints = PredefinedJointStates()
    Initial_Joints.joint_states = [0.0, -2.39, 2.4, -3.15, -1.58, 3.15]
    Initial_Joints.name = "Initial_Joints"

    Pre_Drop_Joints = PredefinedJointStates()
    Pre_Drop_Joints.joint_states = [0.00, -2.94, 1.291, -1.491, -1.570, -3.14]
    Pre_Drop_Joints.name = "Pre_Drop_Joints"

    Drop_Joints_Left = PredefinedJointStates()
    Drop_Joints_Left.joint_states = [-0.403, -1.970, -1.145, -3.156, -1.978, 3.152]
    Drop_Joints_Left.name = "Drop_Joints_Left"

    Drop_Joints_Right = PredefinedJointStates()
    Drop_Joints_Right.joint_states = [0.568, -1.792, -1.374, -3.105, -1.006, 3.142]
    Drop_Joints_Right.name = "Drop_Joints_Right"

    Drop_Joints_Back = PredefinedJointStates()
    Drop_Joints_Back.joint_states = [-0.013, -2.428, -0.375, -3.500, -1.587, 3.148]
    Drop_Joints_Back.name = "Drop_Joints_Back"

    Pickup_Joints_Front = PredefinedJointStates()
    Pickup_Joints_Front.joint_states = [-0.00, -2.43, 2.10, -2.81, -1.56, 3.15]
    Pickup_Joints_Front.name = "Pickup_Joints_Front"

    Pickup_Joints_Left = PredefinedJointStates()
    Pickup_Joints_Left.joint_states = [1.57, -2.43, 2.10, -2.81, -1.56, 3.15]
    Pickup_Joints_Left.name = "Pickup_Joints_Left"

    Pickup_Joints_Right = PredefinedJointStates()
    Pickup_Joints_Right.joint_states = [-1.57, -2.43, 2.10, -2.81, -1.56, 3.15]
    Pickup_Joints_Right.name = "Pickup_Joints_Right"

    Drop_Joints_List = [Drop_Joints_Left, Drop_Joints_Right, Drop_Joints_Back]


    cylinder = path.join(
        path.dirname(path.realpath(__file__)), "..", "assets", "cylinder.stl"
    )

    tolerance = 0.02

    global aruco_name_list
    global aruco_angle_list
    global aruco_ap_list

    # Create node for this example
    node = Node("pick_aruco")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=robotic_arm.joint_names(),
        base_link_name=robotic_arm.base_link_name(),
        end_effector_name=robotic_arm.end_effector_name(),
        group_name=robotic_arm.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )
    moveit2Servo = MoveIt2Servo(
        node=node, frame_id=robotic_arm.base_link_name(), callback_group=callback_group
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    tf_buffer = tf2_ros.buffer.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)
    aruco_data_subscriber = node.create_subscription(
        String, "/aruco_data", aruco_data_updater, 10, callback_group=callback_group
    )

   

    moveit2.add_collision_mesh(
        filepath=cylinder,
        id="Floor",
        position=[0.0, 0.4, 0.0],
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        frame_id="base_link",
    )
    print("Done")
    rclpy.spin(node)
    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()