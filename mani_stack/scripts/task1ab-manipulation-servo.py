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

aruco_name_list = []


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


def main():
    rclpy.init()

    Initial_Pose  = ArucoBoxPose()
    Initial_Pose.position = [0.18, 0.10, 0.46]
    Initial_Pose.quaternions = [0.50479, 0.495985, 0.499407, 0.499795]

    P1 = ArucoBoxPose()
    P1.position = [0.35, 0.1, 0.68]
    P1.quaternions = [0.50479, 0.495985, 0.499407, 0.499795]

    P2 = ArucoBoxPose()
    P2.position = [0.194, -0.43, 0.701]
    P2.quaternions = [0.7657689, 0.0, 0.0, 0.6431158]

    Drop = ArucoBoxPose()
    Drop.position = [-0.37, 0.12, 0.397]
    Drop.quaternions = [0.5414804, -0.4547516, -0.5414804, 0.4547516]

    global aruco_name_list

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
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    tf_buffer = tf2_ros.buffer.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)
    aruco_name_subscriber = node.create_subscription(
        String, "/aruco_list", aruco_name_list_updater, 10
    )
    twist_pub = node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
    time.sleep(5)

    arucoData = []
    while len(arucoData) < 2:
        if (
            tf_buffer.can_transform("base_link", aruco_name_list[0], rclpy.time.Time())
            and tf_buffer.can_transform(
                "base_link", aruco_name_list[1], rclpy.time.Time()
            )
            and tf_buffer.can_transform(
                "base_link", aruco_name_list[2], rclpy.time.Time()
            )
        ):
            arucoData = []

            for i in range(len(aruco_name_list)):
                arucoData.append(ArucoNameCoordinate())
                arucoData[i].name = aruco_name_list[i]
                arucoData[i].position = [
                    tf_buffer.lookup_transform(
                        "base_link", aruco_name_list[i], rclpy.time.Time()
                    ).transform.translation.x,
                    tf_buffer.lookup_transform(
                        "base_link", aruco_name_list[i], rclpy.time.Time()
                    ).transform.translation.y,
                    tf_buffer.lookup_transform(
                        "base_link", aruco_name_list[i], rclpy.time.Time()
                    ).transform.translation.z,
                ]
                arucoData[i].quaternions = [
                    tf_buffer.lookup_transform(
                        "base_link", aruco_name_list[i], rclpy.time.Time()
                    ).transform.rotation.x,
                    tf_buffer.lookup_transform(
                        "base_link", aruco_name_list[i], rclpy.time.Time()
                    ).transform.rotation.y,
                    tf_buffer.lookup_transform(
                        "base_link", aruco_name_list[i], rclpy.time.Time()
                    ).transform.rotation.z,
                    tf_buffer.lookup_transform(
                        "base_link", aruco_name_list[i], rclpy.time.Time()
                    ).transform.rotation.w,
                ]

    def moveWithServo(linear_speed, angular_speed):
        twist_msg = TwistStamped()
        twist_msg.header.frame_id = ur5.base_link_name()
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.twist.linear.x = linear_speed[0]
        twist_msg.twist.linear.y = linear_speed[1]
        twist_msg.twist.linear.z = linear_speed[2]
        twist_msg.twist.angular.x = angular_speed[0]
        twist_msg.twist.angular.y = angular_speed[1]
        twist_msg.twist.angular.z = angular_speed[2]
        twist_pub.publish(twist_msg)

    def getCurrentPose(TargetPose, TargetQuats):
        tempPose = [0, 0, 0]
        tempQuats = [0, 0, 0, 0]
        transform = tf_buffer.lookup_transform("base_link", "tool0", rclpy.time.Time())
        tempPose[0] = round(transform.transform.translation.x, 2)
        tempPose[1] = round(transform.transform.translation.y, 2)
        tempPose[2] = round(transform.transform.translation.z, 2)
        time = transform.header.stamp.sec
        tempQuats[0] = round(transform.transform.rotation.x, 4)
        tempQuats[1] = round(transform.transform.rotation.y, 4)
        tempQuats[2] = round(transform.transform.rotation.z, 4)
        tempQuats[3] = round(transform.transform.rotation.w, 4)
        print(
            "Current Pose:",
            tempPose, 
            "Current Quats:",
            tempQuats,
            "Time:",
            time,
            "\nTarget Pose:",
            TargetPose,
            "Target Quats:",
            TargetQuats,
        )
        return tempPose

    def checkSphericalTolerance(currentPose, targetPose, tolerance):
        currentTolerance = math.sqrt(
            (currentPose[0] - targetPose[0]) ** 2
            + (currentPose[1] - targetPose[1]) ** 2
            + (currentPose[2] - targetPose[2]) ** 2
        )
        print("Distance left: ", currentTolerance, "\n")
        return True if currentTolerance <= tolerance else False

    def moveToPose(position, quaternions, position_name):
        counter = 0
        position = [round(position[0], 2), round(position[1], 2), round(position[2], 2)]
        quaternions = [round(quaternions[0], 4), round(quaternions[1], 4), round(quaternions[2], 4), round(quaternions[3], 4)]
        # quaternions = P2.quaternions

        x, y, z = False, False, False
        currentPose = [0, 0, 0, 0]
        while x == False and y == False and z == False:
            counter += 1
            print("Moving to ", position_name, "    [Attempt: ", counter, "]")
            if position_name != "Drop":
                moveit2.move_to_pose(
                    position=[position[0], 2*position[1]/3, position[2]],
                    quat_xyzw=quaternions,
                    cartesian=True,
                )
            else:
                moveit2.move_to_pose(
                    position=position, quat_xyzw=quaternions, cartesian=False
                )
            moveit2.wait_until_executed()
            try:
                currentPose = getCurrentPose(TargetPose=position, TargetQuats=quaternions)
                time.sleep(0.05)
            except Exception as e:
                print(e)
            x = True if (currentPose[0] - position[0]) == 0.00 else False
            y = True if (currentPose[1] - position[1]) == 0.00 else False
            z = True if (currentPose[2] - position[2]) == 0.00 else False
            print("x:", x, "y:", y, "z:", z)

        if position_name != "Drop":
            moveit2Servo.enable()
            sphericalToleranceAchieved = False
            tolerance = 0.01
            vx, vy, vz = (position[0]-currentPose[0])/2, (position[1]-currentPose[1])/2, (position[2]-currentPose[2])/2

            while sphericalToleranceAchieved == False:
                moveWithServo([vx, vy, vz], [0.0, 0.0, 0.0])
                print("Vx:", vx, "Vy:", vy, "Vz:", vz)
                currentPose = getCurrentPose(TargetPose=position, TargetQuats=quaternions)
                sphericalToleranceAchieved = checkSphericalTolerance(
                    currentPose, position, tolerance
                )
                time.sleep(0.05)
            print("Tolerance Achieved")

    for aruco in arucoData:
        moveToPose(aruco.position, aruco.quaternions, aruco.name)
        print("Reached ", aruco.name)
        moveToPose(Drop.position, Drop.quaternions, "Drop")
        print("Reached Drop")

    print("Done")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
