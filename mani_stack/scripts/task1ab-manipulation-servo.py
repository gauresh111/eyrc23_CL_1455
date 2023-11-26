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
servo_status = 5
joint_states = [0, 0, 0, 0, 0, 0]


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

    print("Starting Task 1AB Manipulation Servo")

    Initial_Pose = ArucoBoxPose()
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

    Pre_Drop_Joints = PredefinedJointStates()
    # Pre_Drop_Joints.joint_states = [0.0, -2.79, 1.95, -2.30, -1.57, 3.14159]
    Pre_Drop_Joints.joint_states = [0.00, -2.94, 1.291, -1.491, -1.570, -3.14]
    Pre_Drop_Joints.name = "Pre_Drop_Joints"

    Drop_Joints = PredefinedJointStates()
    Drop_Joints.joint_states = [0.0, -1.918, -1.213, -3.143, -1.574, 3.149]
    Drop_Joints.name = "Drop_Joints"

    Drop_Joints_Left = PredefinedJointStates()
    Drop_Joints_Left.joint_states = [-0.403, -1.970, -1.145, -3.156, -1.978, 3.152]
    Drop_Joints_Left.name = "Drop_Joints_Left"

    Drop_Joints_Right = PredefinedJointStates()
    Drop_Joints_Right.joint_states = [0.568, -1.792, -1.374, -3.105, -1.006, 3.142]
    Drop_Joints_Right.name = "Drop_Joints_Right"

    Drop_Joints_Back = PredefinedJointStates()
    Drop_Joints_Back.joint_states = [-0.013, -2.428, -0.375, -3.500, -1.587, 3.148]
    Drop_Joints_Back.name = "Drop_Joints_Back"

    Drop_Joints_List = [Drop_Joints_Left, Drop_Joints_Right, Drop_Joints_Back]

    box_file_path = path.join(
        path.dirname(path.realpath(__file__)),"..", "assets", "box.stl"
    )

    tolerance = 0.02

    global aruco_name_list
    # global servo_status

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
        String,
        "/aruco_list",
        aruco_name_list_updater,
        10,
        callback_group=callback_group,
    )
    # servo_status_subscriber = node.create_subscription(
    #     String, "/servo_node/status", servo_status_updater, 10, callback_group=callback_group
    # )
    # joint_states_subscriber = node.create_subscription(
    #     JointState, "/joint_states", joint_states_updater, 10, callback_group=callback_group
    # )

    twist_pub = node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
    time.sleep(5)

    while not node.create_client(AttachLink, "/GripperMagnetON").wait_for_service(timeout_sec=1.0):
        node.get_logger().info("EEF service not available, waiting again...")

    arucoData = []
    while len(arucoData) < len(aruco_name_list):
        flag = True
        for aruco in aruco_name_list:
            tempFlag = False
            if tf_buffer.can_transform("base_link", aruco, rclpy.time.Time()):
                tempFlag = True
            flag = tempFlag and flag

        if flag == True:
            arucoData = []

            for i in range(len(aruco_name_list)):
                arucoData.append(ArucoNameCoordinate())
                arucoData[i].name = aruco_name_list[i]
                transform = tf_buffer.lookup_transform(
                    "base_link", aruco_name_list[i], rclpy.time.Time()
                ).transform
                arucoData[i].position = [
                    transform.translation.x,
                    transform.translation.y,
                    transform.translation.z,
                ]
                arucoData[i].quaternions = [
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w,
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

    def getCurrentPose(TargetPose=[0.0, 0.0, 0.0], TargetQuats=[0.0, 0.0, 0.0, 1.0]):
        tempPose = [0, 0, 0]
        tempQuats = [0, 0, 0, 0]
        transform = tf_buffer.lookup_transform("base_link", "tool0", rclpy.time.Time())
        tempPose[0] = round(transform.transform.translation.x, 7)
        tempPose[1] = round(transform.transform.translation.y, 7)
        tempPose[2] = round(transform.transform.translation.z, 7)
        time = transform.header.stamp.sec
        tempQuats[0] = round(transform.transform.rotation.x, 2)
        tempQuats[1] = round(transform.transform.rotation.y, 2)
        tempQuats[2] = round(transform.transform.rotation.z, 2)
        tempQuats[3] = round(transform.transform.rotation.w, 2)
        # print(
        #     "Current Pose:",
        #     tempPose,
        #     "Current Quats:",
        #     tempQuats,
        #     "Time:",
        #     time,
        #     "\nTarget Pose:",
        #     TargetPose,
        #     "Target Quats:",
        #     TargetQuats,
        # )
        return tempPose, tempQuats

    def controlGripper(status, box_name):
        if status == "ON":
            gripper_control = node.create_client(AttachLink, "/GripperMagnetON")
            req = AttachLink.Request()
        else:
            gripper_control = node.create_client(DetachLink, "/GripperMagnetOFF")
            req = DetachLink.Request()

        while not gripper_control.wait_for_service(timeout_sec=1.0):
            node.get_logger().info("EEF service not available, waiting again...")

        req.model1_name = box_name
        req.link1_name = "link"
        req.model2_name = "ur5"
        req.link2_name = "wrist_3_link"
        print(gripper_control.call_async(req))
        time.sleep(1)

        print("Gripper Status: ", status, "has been requested")

    def checkSphericalTolerance(currentPose, targetPose, tolerance):
        currentTolerance = math.sqrt(
            (currentPose[0] - targetPose[0]) ** 2
            + (currentPose[1] - targetPose[1]) ** 2
            + (currentPose[2] - targetPose[2]) ** 2
        )
        print("Distance left: ", currentTolerance, "\n")
        return True if currentTolerance <= tolerance else False, currentTolerance

    def moveToJointStates(joint_states):
        moveit2.move_to_configuration(joint_states)
        moveit2.wait_until_executed()

    def moveToPose(position, quaternions, position_name, dropData):
        def servo_status_updater(msg):
            global servo_status
            servo_status = msg.data
            # print("Servo Status: ", servo_status)

        def joint_states_updater(msg):
            global joint_states
            joint_states = list([states for states in msg.position])

        servoNode = Node("ServoNode")
        callback_group = ReentrantCallbackGroup()
        servo_executor = rclpy.executors.MultiThreadedExecutor(2)
        servo_executor.add_node(servoNode)
        servo_executor_thread = Thread(target=servo_executor.spin, daemon=True, args=())
        servo_executor_thread.start()
        servoNode.odom_sub = servoNode.create_subscription(
            Int8, "/servo_node/status", servo_status_updater, 10
        )
        servoNode.odom_sub

        jointStatesNode = Node("JointStatesNode")
        callback_group = ReentrantCallbackGroup()
        jointStates_executor = rclpy.executors.MultiThreadedExecutor(2)
        jointStates_executor.add_node(jointStatesNode)
        jointStates_executor_thread = Thread(
            target=jointStates_executor.spin, daemon=True, args=()
        )
        jointStates_executor_thread.start()
        jointStatesNode.odom_sub = jointStatesNode.create_subscription(
            JointState, "/joint_states", joint_states_updater, 10
        )
        jointStatesNode.odom_sub

        time.sleep(3)

        while True:
            global servo_status

            counter = 0
            position = [
                round(position[0], 2),
                round(position[1], 2),
                round(position[2], 2),
            ]
            midPosition = [3 * position[0] / 4, 3 * position[1] / 4, position[2]]
            quaternions = [
                round(quaternions[0], 4),
                round(quaternions[1], 4),
                round(quaternions[2], 4),
                round(quaternions[3], 4),
            ]
            box_name = "box" + str(int(re.search(r"\d+", position_name).group()))
            # quaternions = P2.quaternions

            x, y, z = False, False, False
            currentPose = [0, 0, 0, 0]
            while x == False and y == False and z == False:
                counter += 1
                print("Moving to ", position_name, "    [Attempt: ", counter, "]")
                # if position_name != "Drop":
                moveit2.move_to_pose(
                    position=midPosition,
                    quat_xyzw=quaternions,
                    cartesian=True,
                )
                # else:
                #     moveit2.move_to_pose(
                #         position=position, quat_xyzw=quaternions, cartesian=False
                #     )
                moveit2.wait_until_executed()
                try:
                    currentPose = getCurrentPose(
                        TargetPose=position, TargetQuats=quaternions
                    )[0]
                    time.sleep(0.05)
                except Exception as e:
                    print(e)
                x = (
                    True
                    if (round(currentPose[0], 2) - midPosition[0]) == 0.00
                    else False
                )
                y = (
                    True
                    if (round(currentPose[1], 2) - midPosition[1]) == 0.00
                    else False
                )
                z = (
                    True
                    if (round(currentPose[2], 2) - midPosition[2]) == 0.00
                    else False
                )
                print("x:", x, "y:", y, "z:", z)

            # if position_name != "Drop":
            moveit2Servo.enable()
            sphericalToleranceAchieved = False
            currentPose = getCurrentPose(TargetPose=position, TargetQuats=quaternions)[0]
            _, magnitude = checkSphericalTolerance(currentPose, position, tolerance)
            magnitude *= 1.5
            vx, vy, vz = (
                (position[0] - currentPose[0]) / magnitude,
                (position[1] - currentPose[1]) / magnitude,
                (position[2] - currentPose[2]) / magnitude,
            )

            while sphericalToleranceAchieved == False and servo_status == 0:
                moveWithServo([vx, vy, vz], [0.0, 0.0, 0.0])
                # print("Vx:", vx, "Vy:", vy, "Vz:", vz)
                currentPose = getCurrentPose(
                    TargetPose=position, TargetQuats=quaternions
                )[0]
                sphericalToleranceAchieved, _ = checkSphericalTolerance(
                    currentPose, position, tolerance
                )
                time.sleep(0.01)
                # print("Servo Status in While Loop: ", servo_status)
                if servo_status > 0:
                    print("Exited While Loop due to Servo Error", servo_status)
                    break
            if servo_status > 0:
                print(
                    "Servo Status ERROR:",
                    servo_status,
                    "     Continuing next iteration",
                )
                continue
            print("Tolerance Achieved: Reached Box")
            time.sleep(0.1)

            controlGripper("ON", box_name)
            time.sleep(1)
            # return

            newMidPose = [position[0] / 3, position[1] / 3, midPosition[2]]
            moveit2Servo.enable()
            sphericalToleranceAchieved = False
            currentPose = getCurrentPose(TargetPose=newMidPose, TargetQuats=quaternions)[0]
            _, magnitude = checkSphericalTolerance(currentPose, newMidPose, tolerance)
            magnitude *= 3
            vx, vy, vz = (
                (newMidPose[0] - currentPose[0]) / magnitude,
                (newMidPose[1] - currentPose[1]) / magnitude,
                (newMidPose[2] - currentPose[2]) / magnitude,
            )
            while sphericalToleranceAchieved == False and servo_status < 1:
                moveWithServo([vx, vy, vz], [0.0, 0.0, 0.0])
                # print("Vx:", vx, "Vy:", vy, "Vz:", vz)
                currentPose = getCurrentPose(
                    TargetPose=newMidPose, TargetQuats=quaternions
                )[0]
                sphericalToleranceAchieved, _ = checkSphericalTolerance(
                    currentPose, newMidPose, tolerance
                )
                time.sleep(0.01)
                # print("Servo Status in While Loop: ", servo_status)
                if servo_status > 0:
                    print("Exited next While Loop due to Servo Error", servo_status)
                    break
            # if servo_status > 0:
            #         print("Exited next While Loop due to Servo Error", servo_status)
            #         continue
            print("Tolerance Achieved: Came out")
            time.sleep(0.1)

            for i in range(5):
                moveit2.add_collision_mesh( filepath=box_file_path, id="currentBox", position=[0.0, -0.12, 0.09], quat_xyzw=[ -0.5, 0.5, 0.5, 0.5  ], frame_id='tool0',
                )
                time.sleep(0.5)
            

            # Move to Pre Drop Pose
            print("Moving to ", "Pre Drop Pose")
            moveit2.move_to_configuration(Pre_Drop_Joints.joint_states)
            moveit2.wait_until_executed()
            print("Reached Pre-Drop")

            # Move to Drop Pose
            print("Moving to ", dropData.name)
            moveit2.move_to_configuration(dropData.joint_states)
            moveit2.wait_until_executed()
            print("Reached Drop")

            controlGripper("OFF", box_name)

            for i in range(5):
                moveit2.remove_collision_mesh(id="currentBox")
                time.sleep(0.5)
            time.sleep(1)

            # Move to Pre Drop Pose
            print("Moving to ", "Pre Drop Pose")
            moveit2.move_to_configuration(Pre_Drop_Joints.joint_states)
            moveit2.wait_until_executed()
            print("Reached Pre-Drop")

            servoNode.destroy_node()
            jointStatesNode.destroy_node()
            break

    for aruco, drop in zip(arucoData, Drop_Joints_List):
        moveToPose(aruco.position, aruco.quaternions, aruco.name, drop)
        # print("Reached ", aruco.name)
        # moveToPose(Drop.position, Drop.quaternions, "Drop")
        # print("Reached Drop")

    print("Done")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
