#!/usr/bin/env python3
from os import path
from threading import Thread
import time
import sys
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from pymoveit2 import MoveIt2, MoveIt2Servo
from pymoveit2.robots import ur5
import tf2_ros
import math
from ur_msgs.srv import SetIO
from controller_manager_msgs.srv import SwitchController

# from linkattacher_msgs.srv import AttachLink, DetachLink
import re
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
import transforms3d as tf3d
import numpy as np
from std_msgs.msg import Bool

global servo_status
servo_status = 5
# StartBox = False
current_joint_states = [0, 0, 0, 0, 0, 0]


class ArucoNameCoordinate:
    def __init__(self):
        self.name = None
        self.position = None
        self.quaternions = None
        self.eulerAngles = None
        self.rotationName = None


class PredefinedJointStates:
    def __init__(self):
        self.joint_states = None
        self.name = None


class Manipulation(Node):
    def __init__(self):
        super().__init__("manipulation_process")
        print("Manipulation Process Node Registered")

        self.Initial_Joints = PredefinedJointStates()
        self.Initial_Joints.joint_states = [0.0, -2.39, 2.4, -3.15, -1.58, 3.15]
        # Initial_Joints.joint_states = [-0.02, -2.28, 1.85, -2.71, -1.56, 3.15] # Higher
        self.Initial_Joints.name = "Initial_Joints"

        self.Pre_Drop_Joints = PredefinedJointStates()
        # Pre_Drop_Joints.joint_states = [0.0, -2.79, 1.95, -2.30, -1.57, 3.14159]
        self.Pre_Drop_Joints.joint_states = [0.00, -2.94, 1.291, -1.491, -1.570, -3.14]
        self.Pre_Drop_Joints.name = "Pre_Drop_Joints"

        self.Drop_Joints = PredefinedJointStates()
        self.Drop_Joints.joint_states = [0.0, -1.918, -1.213, -3.143, -1.574, 3.149]
        self.Drop_Joints.name = "Drop_Joints"

        self.Drop_Joints_Left = PredefinedJointStates()
        self.Drop_Joints_Left.joint_states = [
            -0.403,
            -1.970,
            -1.145,
            -3.156,
            -1.978,
            3.152,
        ]
        self.Drop_Joints_Left.name = "Drop_Joints_Left"

        self.Drop_Joints_Right = PredefinedJointStates()
        self.Drop_Joints_Right.joint_states = [
            0.568,
            -1.792,
            -1.374,
            -3.105,
            -1.006,
            3.142,
        ]
        self.Drop_Joints_Right.name = "Drop_Joints_Right"

        self.Drop_Joints_Back = PredefinedJointStates()
        self.Drop_Joints_Back.joint_states = [
            -0.013,
            -2.428,
            -0.375,
            -3.500,
            -1.587,
            3.148,
        ]
        self.Drop_Joints_Back.name = "Drop_Joints_Back"

        self.Pickup_Joints_Front = PredefinedJointStates()
        self.Pickup_Joints_Front.joint_states = [-0.00, -2.43, 2.10, -2.81, -1.56, 3.15]
        self.Pickup_Joints_Front.name = "Pickup_Joints_Front"

        self.Pickup_Joints_Left = PredefinedJointStates()
        self.Pickup_Joints_Left.joint_states = [1.57, -2.43, 2.10, -2.81, -1.56, 3.15]
        self.Pickup_Joints_Left.name = "Pickup_Joints_Left"

        self.Pickup_Joints_Right = PredefinedJointStates()
        self.Pickup_Joints_Right.joint_states = [-1.57, -2.43, 2.10, -2.81, -1.56, 3.15]
        self.Pickup_Joints_Right.name = "Pickup_Joints_Right"

        self.Drop_Joints_List = [
            self.Drop_Joints_Left,
            self.Drop_Joints_Right,
            self.Drop_Joints_Back,
        ]

        self.box_file_path = path.join(
            path.dirname(path.realpath(__file__)), "..", "assets", "box.stl"
        )
        # floor_file_path = path.join(
        #     path.dirname(path.realpath(__file__)),"..", "assets", "floor.stl"
        # )
        self.floor_file_path = path.join(
            path.dirname(path.realpath(__file__)), "..", "assets", "simpleRack.stl"
        )
        self.floor = path.join(
            path.dirname(path.realpath(__file__)), "..", "assets", "floor.stl"
        )

        self.tolerance = 0.02

        self.aruco_name_list = []
        self.servo_status = 0

        self.callback_group = ReentrantCallbackGroup()

        self.moveit2 = MoveIt2(
            node=Node,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

        moveit2Servo = MoveIt2Servo(
            node=Node, frame_id=ur5.base_link_name(), callback_group=self.callback_group
        )

        self.executor = rclpy.executors.MultiThreadedExecutor(4)
        self.executor.add_node(Node)
        self.executor_thread = Thread(target=self.executor.spin, daemon=True, args=())
        self.executor_thread.start()

        self.tf_buffer = tf2_ros.buffer.Buffer()

        aruco_name_subscriber = self.create_subscription(
            String,
            "/aruco_list",
            aruco_name_list_updater,
            10,
            callback_group=self.callback_group,
        )

        servo_status_subscriber = self.create_subscription(
            String,
            "/servo_node/status",
            servo_status_updater,
            10,
            callback_group=self.callback_group,
        )

        joint_states_subscriber = self.create_subscription(
            JointState,
            "/joint_states",
            joint_states_updater,
            10,
            callback_group=self.callback_group,
        )

        ManipulationStart = self.create_subscription(
            Bool,
            "/StartArnManipulation",
            getBox_id,
            10,
            callback_group=self.callback_group,
        )


def main():
    """
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    """
    rclpy.init(args=sys.argv)  # initialisation
    node = rclpy.create_node("manipulation_process")  # creating ROS node
    node.get_logger().info("Node created: Manipulation Process")  # logging information
    manipulation_class = Manipulation()  # creating a new object for class 'aruco_tf'
    rclpy.spin(
        manipulation_class
    )  # spining on the object to make it alive in ROS 2 DDS
    manipulation_class.destroy_node()  # destroy node after spin ends
    rclpy.shutdown()  # shutdown process


if __name__ == "__main__":
    """
    Description:    If the python interpreter is running that module (the source file) as the main program,
                    it sets the special __name__ variable to have a value “__main__”.
                    If this file is being imported from another module, __name__ will be set to the module's name.
                    You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/
    """

    main()
