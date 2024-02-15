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
from pymoveit2.robots import ur5
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

if is_sim == True:
    from linkattacher_msgs.srv import AttachLink, DetachLink
else:
    from ur_msgs.srv import SetIO  # type: ignore
    from controller_manager_msgs.srv import SwitchController

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

    box_file_path = path.join(
        path.dirname(path.realpath(__file__)), "..", "assets", "box.stl"
    )

    floor_file_path = path.join(
        path.dirname(path.realpath(__file__)), "..", "assets", "simpleRack.stl"
    )
    floor = path.join(
        path.dirname(path.realpath(__file__)), "..", "assets", "floor.stl"
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
    aruco_data_subscriber = node.create_subscription(
        String, "/aruco_data", aruco_data_updater, 10, callback_group=callback_group
    )

    twist_pub = node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)

    if is_sim == False:
        contolMSwitch = node.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )

        while not contolMSwitch.wait_for_service(timeout_sec=5.0):
            node.get_logger().warn(f"Service control Manager is not yet available...")

    time.sleep(5)

    def moveToJointStates(joint_states, position_name):
        counter = 1
        while True:
            print("Moving to ", position_name, "    [Attempt: ", counter, "]")
            moveit2.move_to_configuration(joint_states, tolerance=0.01)
            status = moveit2.wait_until_executed()
            time.sleep(0.1)
            counter += 1
            print("Joint State Difference: ", end="")
            print(
                round(round(joint_states[0], 1) - round(current_joint_states[0], 1), 1),
                round(round(joint_states[1], 1) - round(current_joint_states[1], 1), 1),
                round(round(joint_states[2], 1) - round(current_joint_states[2], 1), 1),
                round(round(joint_states[3], 1) - round(current_joint_states[3], 1), 1),
                round(round(joint_states[4], 1) - round(current_joint_states[4], 1), 1),
                round(round(joint_states[5], 1) - round(current_joint_states[5], 1), 1),
            )
            if status == True:
                break
            else:
                continue

    if is_sim == True:
        while not node.create_client(AttachLink, "/GripperMagnetON").wait_for_service(
            timeout_sec=1.0
        ):
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
                arucoData.append(ArucoData())
                arucoData[i].name = aruco_name_list[i]
                arucoData[i].id = int(re.search(r"\d+", aruco_name_list[i]).group())
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
                arucoData[i].yaw = aruco_angle_list[i]
                arucoData[i].ap = aruco_ap_list[i]
                arucoData[i].rotation_name = (
                    "Front"
                    if aruco_ap_list[i] == "ap1"
                    else "Left" if aruco_ap_list[i] == "ap2" else "Right"
                )

                if arucoData[i].ap == "ap1":
                    error = arucoData[i].yaw
                elif arucoData[i].ap == "ap2":
                    error = arucoData[i].yaw - 90
                else:
                    error = arucoData[i].yaw + 90
                arucoData[i].yaw_error = error

    print("No. of Arucos Detected: ", len(arucoData), "\n")
    for aruco in arucoData:
        print(
            "Aruco Name: ",
            aruco.name,
            "\nPosition: ",
            aruco.position,
            "\nQuaternions: ",
            list(np.around(np.array(aruco.quaternions), 2)),
            "\nYaw: ",
            aruco.yaw,
            "\nYaw Error: ",
            aruco.yaw_error,
            "\nap: ",
            aruco.ap,
            "\nRotation Name: ",
            aruco.rotation_name,
            "\n",
        )
    if is_sim == False:

        def switch_controller(useMoveit: bool):
            contolMSwitch = node.create_client(
                SwitchController, "/controller_manager/switch_controller"
            )
            # Parameters to switch controller
            switchParam = SwitchController.Request()
            if useMoveit == True:
                switchParam.activate_controllers = [
                    "scaled_joint_trajectory_controller"
                ]  # for normal use of moveit
                switchParam.deactivate_controllers = ["forward_position_controller"]
            else:
                switchParam.activate_controllers = [
                    "forward_position_controller"
                ]  # for servoing
                switchParam.deactivate_controllers = [
                    "scaled_joint_trajectory_controller"
                ]
            switchParam.strictness = 2
            switchParam.start_asap = False

            # calling control manager service after checking its availability
            while not contolMSwitch.wait_for_service(timeout_sec=5.0):
                node.get_logger().warn(
                    f"Service control Manager is not yet available..."
                )
            contolMSwitch.call_async(switchParam)
            time.sleep(1.0)
            print("[CM]: Switching to", "Moveit" if useMoveit else "Servo", "Complete")

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

    def addCollisionObject(objectType, id, position, orientation, frame_id):
        if objectType == "box":
            path = box_file_path
        else:
            path = floor_file_path
        if orientation == "Front":
            quat_xyzw = [0.0, 0.0, 0.0, 1.0]
        elif orientation == "Left":
            quat_xyzw = [0, 0, 0.7071068, 0.7071068]
        elif orientation == "Right":
            quat_xyzw = [0, 0, 0.7071068, 0.7071068]
        else:
            quat_xyzw = orientation

        for i in range(3):
            moveit2.add_collision_mesh(
                filepath=path,
                id=id,
                position=position,
                quat_xyzw=quat_xyzw,
                frame_id=frame_id,
            )

    def getCurrentPose(useEuler=False):
        tempPose = [0, 0, 0]
        tempQuats = [0, 0, 0, 0]
        transform = tf_buffer.lookup_transform("base_link", "tool0", rclpy.time.Time())
        tempPose[0] = round(transform.transform.translation.x, 7)
        tempPose[1] = round(transform.transform.translation.y, 7)
        tempPose[2] = round(transform.transform.translation.z, 7)
        tempQuats[0] = transform.transform.rotation.x
        tempQuats[1] = transform.transform.rotation.y
        tempQuats[2] = transform.transform.rotation.z
        tempQuats[3] = transform.transform.rotation.w
        if useEuler == True:
            tempQuats[0] = transform.transform.rotation.w
            tempQuats[1] = transform.transform.rotation.x
            tempQuats[2] = transform.transform.rotation.y
            tempQuats[3] = transform.transform.rotation.z
            tempQuats = tf3d.euler.quat2euler(tempQuats)
        return tempPose, tempQuats

    def controlGripper(status, box_name):
        if is_sim == True:
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
            print("ur5 ->", box_name)
            print(gripper_control.call_async(req))
            time.sleep(0.2)

            print("Gripper Status: ", status, "has been requested")
        else:
            """
            based on the state given as i/p the service is called to activate/deactivate
            pin 16 of TCP in UR5
            i/p: node, state of pin:Bool
            o/p or return: response from service call
            """
            if status == "ON":
                state = 1
            else:
                state = 0
            gripper_control = node.create_client(
                SetIO, "/io_and_status_controller/set_io"
            )
            while not gripper_control.wait_for_service(timeout_sec=1.0):
                node.get_logger().info(
                    "EEF Tool service not available, waiting again..."
                )
            req = SetIO.Request()
            req.fun = 1
            req.pin = 16
            req.state = float(state)
            gripper_control.call_async(req)
            print("Gripper Hardware Status: ", status, "has been requested")
            time.sleep(1.0)
            return state

    def checkSphericalTolerance(currentPose, targetPose, tolerance):
        currentTolerance = math.sqrt(
            (currentPose[0] - targetPose[0]) ** 2
            + (currentPose[1] - targetPose[1]) ** 2
            + (currentPose[2] - targetPose[2]) ** 2
        )
        return True if currentTolerance <= tolerance else False, currentTolerance

    def moveToPoseWithServo(
        TargetPose,
        TargetQuats,
        QuatsOnly=False,
        PoseOnly=False,
        TargetYaw=0,
        YawError=0,
    ):
        if is_sim == False:
            switch_controller(useMoveit=False)
        global servo_status
        mission_status = True
        moveit2Servo.enable()
        sphericalToleranceAchieved = False
        currentPose, currentQuats = getCurrentPose()
        _, magnitude = checkSphericalTolerance(currentPose, TargetPose, tolerance)
        magnitude *= 3
        vx, vy, vz = (
            (TargetPose[0] - currentPose[0]) / magnitude,
            (TargetPose[1] - currentPose[1]) / magnitude,
            (TargetPose[2] - currentPose[2]) / magnitude,
        )
        distance = magnitude
        totalTime = (
            distance
            / checkSphericalTolerance([0.0, 0.0, 0.0], [vx, vy, vz], tolerance)[1]
        )
        print("TargetQuats:", TargetQuats, "CurrentQuats:", currentQuats)
        TargetEuler = euler_from_quaternion(
            [TargetQuats[3], TargetQuats[0], TargetQuats[1], TargetQuats[2]]
        )
        currentEuler = euler_from_quaternion(
            [currentQuats[3], currentQuats[0], currentQuats[1], currentQuats[2]]
        )
        ax, ay, az = (
            (TargetEuler[0] - currentEuler[0]) / magnitude,
            (TargetEuler[2] - currentEuler[2]) / magnitude,
            (TargetEuler[1] - currentEuler[1]) / magnitude,
        )
        print("yawError_d: ", YawError, "yawError_r: ", math.radians(YawError))
        az = (math.radians(YawError) / totalTime) * 3
        print("TargetPose:", TargetPose, "CurrentPose:", currentPose)
        print("TargetEuler:", TargetEuler, "CurrentEuler:", currentEuler)

        print("Vx:", vx, "Vy:", vy, "Vz:", vz)
        print("Ax:", ax, "Ay:", ay, "Az:", az)

        if QuatsOnly == True:
            print("Servoing Quats Only")
            yawError = math.radians(YawError)
            az = -0.5 if yawError > 0.0 else 0.5
            print("Yaw Error: ", yawError)
            while abs(yawError) > 0.02:
                moveWithServo([0.0, 0.0, 0.0], [0.0, 0.0, az])
                # print("Vx:", vx, "Vy:", vy, "Vz:", vz)
                currentEuler = getCurrentPose(useEuler=True)[1]
                if TargetYaw == 0:
                    yawError = -(currentEuler[2] - math.pi / 2)
                elif TargetYaw == 90:
                    yawError = currentEuler[2] - math.pi
                else:
                    yawError = currentEuler[
                        2
                    ]  # math.radians(TargetYaw) - (currentEuler[2])
                print(
                    "Yaw Error: ",
                    yawError,
                    "current: ",
                    currentEuler[2],
                    "Target: ",
                    math.radians(TargetYaw),
                )
                time.sleep(0.01)
                if servo_status > 0:
                    mission_status = False
                    print("Exited While Loop due to Servo Error", servo_status)
                    break
            if is_sim == False:
                switch_controller(useMoveit=True)
            return mission_status

        elif PoseOnly == True:
            print("Servoing Pose Only")
            while sphericalToleranceAchieved == False:
                moveWithServo([vx, vy, vz], [0.0, 0.0, 0.0])
                # print("Vx:", vx, "Vy:", vy, "Vz:", vz)
                currentPose = getCurrentPose()[0]
                sphericalToleranceAchieved, _ = checkSphericalTolerance(
                    currentPose, TargetPose, tolerance
                )
                time.sleep(0.01)
                if servo_status > 0:
                    mission_status = False
                    print("Exited While Loop due to Servo Error", servo_status)
                    break
            if is_sim == False:
                switch_controller(useMoveit=True)
            return mission_status

        else:
            print("Servoing Pose and Quats")
            print("Estimated Time: ", totalTime, " seconds")
            currentTime = node.get_clock().now().nanoseconds * 1e-9
            while sphericalToleranceAchieved == False:
                moveWithServo([vx, vy, vz], [0.0, 0.0, az])
                # print("Vx:", vx, "Vy:", vy, "Vz:", vz)
                currentPose, currentQuats = getCurrentPose()
                sphericalToleranceAchieved, _ = checkSphericalTolerance(
                    currentPose, TargetPose, tolerance
                )
                # if (
                #     checkSphericalTolerance(currentQuats, TargetQuats, tolerance)
                #     == True
                # ):
                #     ax, ay, az = 0.0, 0.0, 0.0
                time.sleep(0.01)
                if servo_status > 0:
                    mission_status = False
                    print("Exited While Loop due to Servo Error", servo_status)
                    break
            print(
                "Time taken: ",
                node.get_clock().now().nanoseconds * 1e-9 - currentTime,
                " seconds",
            )
            if is_sim == False:
                switch_controller(useMoveit=True)
            return mission_status

    def moveToPose(aruco_data, drop_angles):
        aruco_name = aruco_data.name
        aruco_id = aruco_data.id
        aruco_position = aruco_data.position
        aruco_quaternions = aruco_data.quaternions
        yaw = aruco_data.yaw
        yaw_error = aruco_data.yaw_error
        aruco_ap = aruco_data.ap
        rotation_name = aruco_data.rotation_name
        box_name = "box" + str(aruco_id)

        def servo_status_updater(msg):
            global servo_status
            servo_status = msg.data
            # print("Servo Status: ", servo_status)

        def joint_states_updater(msg):
            global current_joint_states
            current_joint_states = list([states for states in msg.position])
            # print("Joint States: ", current_joint_states)

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

        time.sleep(0.2)

        global servo_status

        counter = 1
        aruco_position = [
            round(aruco_position[0], 2),
            round(aruco_position[1], 2),
            round(aruco_position[2], 2),
        ]
        if rotation_name == "Left":
            midPosition = [
                aruco_position[0],
                aruco_position[1] - 0.23,
                aruco_position[2],
            ]
        elif rotation_name == "Right":
            midPosition = [
                aruco_position[0],
                aruco_position[1] + 0.23,
                aruco_position[2],
            ]
        else:
            midPosition = [
                aruco_position[0] - 0.23,
                aruco_position[1],
                aruco_position[2],
            ]
        aruco_quaternions = [
            round(aruco_quaternions[0], 4),
            round(aruco_quaternions[1], 4),
            round(aruco_quaternions[2], 4),
            round(aruco_quaternions[3], 4),
        ]

        if rotation_name == "Left":
            moveToJointStates(Pickup_Joints_Left.joint_states, Pickup_Joints_Left.name)
        elif rotation_name == "Right":
            moveToJointStates(
                Pickup_Joints_Right.joint_states, Pickup_Joints_Right.name
            )
        else:
            moveToJointStates(
                Pickup_Joints_Front.joint_states, Pickup_Joints_Front.name
            )

        if is_sim == False:
            time.sleep(0.1)
            controlGripper("ON", box_name)
            time.sleep(0.2)

        temp_result = moveToPoseWithServo(
            TargetPose=aruco_position, TargetQuats=aruco_quaternions, YawError=yaw_error
        )
        print("Servo Result: ", temp_result)
        global_counter = 0
        while global_counter < 5:
            if temp_result == False:
                while counter < 5:
                    print(
                        "Moving to ",
                        aruco_name,
                        "    [Attempt: ",
                        counter,
                        "Global Attempt: ",
                        global_counter,
                        "]",
                    )
                    moveit2.move_to_pose(
                        position=midPosition,
                        quat_xyzw=aruco_quaternions,
                        tolerance_position=0.01,
                        tolerance_orientation=0.01,
                    )
                    status = moveit2.wait_until_executed()
                    counter += 1
                    if status == False:
                        continue
                    else:
                        break
                temp_result = moveToPoseWithServo(
                    TargetPose=aruco_position, TargetQuats=aruco_quaternions
                )
                if global_counter > 4 or counter > 4:
                    print(
                        "[ERROR !!!] Failed to reach",
                        aruco_name,
                        "after 5 attempts, skipping to next box",
                    )
                    return
                if servo_status > 0:
                    print(
                        "Servo Status ERROR:",
                        servo_status,
                        "     Continuing next iteration",
                    )
                    continue
            else:
                break
            global_counter += 1

        print("Tolerance Achieved: Reached Box")

        # print("## Pushing Box by 5cm")

        # if rotation_name == "Left":
        #     aruco_position[1] += 0.10
        # elif rotation_name == "Right":
        #     aruco_position[1] -= 0.10
        # else:
        #     aruco_position[0] += 0.10

        # temp_result = moveToPoseWithServo(TargetPose=aruco_position, TargetQuats=aruco_quaternions)
        if is_sim == True:
            time.sleep(0.1)
            controlGripper("ON", box_name)
            time.sleep(0.2)

        targetYaw = (
            90 if rotation_name == "Left" else -90 if rotation_name == "Right" else 0
        )

        temp_result = moveToPoseWithServo(
            TargetPose=aruco_position,
            TargetQuats=aruco_quaternions,
            TargetYaw=targetYaw,
            YawError=yaw_error,
            QuatsOnly=True,
        )

        for i in range(3):
            moveit2.add_collision_mesh(
                filepath=box_file_path,
                id="currentBox",
                position=[0.0, -0.1, 0.11],
                quat_xyzw=[-0.5, 0.5, 0.5, 0.5],
                frame_id="tool0",
            )
            time.sleep(0.2)

        current_position, current_quaternions = getCurrentPose()
        if rotation_name == "Left":
            midPosition = [
                current_position[0],
                current_position[1] - 0.23,
                current_position[2],
            ]
        elif rotation_name == "Right":
            midPosition = [
                current_position[0],
                current_position[1] + 0.23,
                current_position[2],
            ]
        else:
            midPosition = [
                current_position[0] - 0.23,
                current_position[1],
                current_position[2],
            ]

        print("### Pulling Box Out")
        moveToPoseWithServo(
            TargetPose=midPosition, TargetQuats=current_quaternions, PoseOnly=True
        )

        print("Tolerance Achieved: Came out")
        time.sleep(0.1)

        # Move to Pre Drop Pose
        moveToJointStates(Pre_Drop_Joints.joint_states, Pre_Drop_Joints.name)
        print("Reached Pre-Drop")

        # Move to Drop Pose
        moveToJointStates(drop_angles.joint_states, drop_angles.name)
        print("Reached Drop")

        controlGripper("OFF", box_name)

        for i in range(3):
            moveit2.remove_collision_mesh(id="currentBox")
            time.sleep(0.2)
        time.sleep(0.2)

        # Move to Pre Drop Pose

        moveToJointStates(Initial_Joints.joint_states, Initial_Joints.name)
        print("Reached Initial Pose")

        servoNode.destroy_node()
        jointStatesNode.destroy_node()

    rackCollisionObjectDistances = {
        "left": {"x": 0.0, "y": 0.0},
        "front": {"x": 0.0, "y": 0.0},
        "right": {"x": 0.0, "y": 0.0},
    }
    left_flag, front_flag, right_flag = False, False, False
    for aruco in arucoData:

        ap = aruco.ap
        print(aruco.name + ":", ap)

        if left_flag == False:
            if ap == "ap2":
                left_flag = True
                rackCollisionObjectDistances["left"] = {"x" : round(aruco.position[0],2) , "y" : round(aruco.position[1],2)+0.16}
                print(aruco.name + ":", "Left")

            # print("Left Flag: ", left_flag)
        if front_flag == False:
            if ap == "ap1":
                front_flag = True
                rackCollisionObjectDistances["front"] = {"x" : round(aruco.position[0],2) + 0.16, "y" : round(aruco.position[1],2)}
                print(aruco.name + ":", "Left")
            # print("Front Flag: ", front_flag)
        if right_flag == False:
            if ap == "ap3":
                right_flag = True
                rackCollisionObjectDistances["right"] = {"x" : round(aruco.position[0],2) , "y" : round(aruco.position[1],2)-0.16}
                print(aruco.name + ":", "Left")
            # print("Right Flag: ", right_flag)
    print(
        "Left Flag: ", left_flag, "Front Flag: ", front_flag, "Right Flag: ", right_flag
    )
    if left_flag:
        print("Adding Left Collision Object")
        addCollisionObject(
            "floor",
            "left_Rack",
            [rackCollisionObjectDistances["left"]["x"], rackCollisionObjectDistances["left"]["y"], 0.16],
            "Left",
            "base_link",
        )
    if front_flag:
        print("Adding Front Collision Object")
        addCollisionObject(
            "floor",
            "front_Rack",
            [rackCollisionObjectDistances["front"]["x"], rackCollisionObjectDistances["front"]["y"], 0.16],
            "Front",
            "base_link",
        )
    if right_flag:
        print("Adding Right Collision Object")
        addCollisionObject(
            "floor",
            "right_Rack",
            [rackCollisionObjectDistances["right"]["x"], rackCollisionObjectDistances["right"]["y"], 0.16],
            "Right",
            "base_link",
        )

    moveit2.add_collision_mesh(
        filepath=floor,
        id="Floor",
        position=[-0.04, 0.03, -0.04],
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        frame_id="base_link",
    )

    for aruco, drop in zip(arucoData, Drop_Joints_List):
        moveToPose(aruco, drop)

    print("Done")
    rclpy.spin(node)
    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
