#!/usr/bin/env python3

from os import path
from threading import Thread
import time
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from pymoveit2 import MoveIt2, MoveIt2Servo
from pymoveit2.robots import ur5
import tf2_ros
import math
from linkattacher_msgs.srv import AttachLink, DetachLink
import re
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8


aruco_name_list = []
global servo_status
servo_status = 0

class ArucoNameCoordinate:
    def __init__(self):
        self.name = None
        self.position = None
        self.quaternions = None


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
    aruco_name_list = msg.data.split()


def main():
    rclpy.init()
    global servo_status

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
    moveit2Servo = MoveIt2Servo(
        node=node, frame_id=ur5.base_link_name(), callback_group=callback_group
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    tf_buffer = tf2_ros.buffer.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)

    aruco_name_subscriber = node.create_subscription(
        String, "/aruco_list", aruco_name_list_updater, 10, callback_group=callback_group
    )
    # servo_status_subscriber = node.create_subscription(
    #     String, "/servo_node/status", servo_status_updater, 10, callback_group=callback_group
    # )
    # joint_states_subscriber = node.create_subscription(
    #     JointState, "/joint_states", joint_states_updater, 10, callback_group=callback_group
    # )

    twist_pub = node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
    time.sleep(5)

    arucoData = []
    while len(arucoData) <2:
        flag = True
        for aruco in aruco_name_list:
            tempFlag = False
            if(tf_buffer.can_transform("base_link", aruco, rclpy.time.Time())):
                tempFlag = True
            flag = tempFlag and flag

        if(flag == True):  
            arucoData = []
            
            for i in range(len(aruco_name_list)):
                arucoData.append(ArucoNameCoordinate())
                arucoData[i].name = aruco_name_list[i]
                transform = tf_buffer.lookup_transform("base_link", aruco_name_list[i], rclpy.time.Time()).transform
                arucoData[i].position = [transform.translation.x,
                                        transform.translation.y,
                                        transform.translation.z
                                        ]
                arucoData[i].quaternions = [transform.rotation.x,
                                            transform.rotation.y,
                                            transform.rotation.z,
                                            transform.rotation.w
                                            ]
                
    def moveToPose(position, quaternions, position_name):
        def servo_status_updater(msg):
            global servo_status
            servo_status = msg.data
            print("Servo Status: ", servo_status)

        def joint_states_updater(msg):
            global joint_states
            joint_states = list([states for states in msg.position])
        
        global servo_status
        servoNode = Node("ServoNode")
        callback_group = ReentrantCallbackGroup()
        servo_executor = rclpy.executors.MultiThreadedExecutor(2)
        servo_executor.add_node(servoNode)
        servo_executor_thread = Thread(target=servo_executor.spin, daemon=True, args=())
        servo_executor_thread.start()
        servoNode.odom_sub = servoNode.create_subscription(Int8, '/servo_node/status', servo_status_updater, 10)
        servoNode.odom_sub
    
        while True:
            a = 1+3

            while True:
                a = 1+3
                print("I am running servo status", servo_status)

        servoNode.destroy_node()
        jointStatesNode.destroy_node()


    for aruco in arucoData:
        moveToPose(aruco.position, aruco.quaternions, aruco.name)
        # print("Reached ", aruco.name)
        # moveToPose(Drop.position, Drop.quaternions, "Drop")
        # print("Reached Drop")

    print("Done")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
