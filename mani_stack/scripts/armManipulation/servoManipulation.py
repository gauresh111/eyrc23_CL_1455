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
from ur_msgs.srv import SetIO
from controller_manager_msgs.srv import SwitchController 
# from linkattacher_msgs.srv import AttachLink, DetachLink
import re
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
import transforms3d as tf3d
import numpy as np
from std_msgs.msg import Bool

aruco_name_list = []
global servo_status
servo_status = 5
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


def aruco_name_list_updater(msg):
    global aruco_name_list
    aruco_name_list = msg.data.split()


def main():
    rclpy.init()

    print("Starting Task 1AB Manipulation Servo")

    Initial_Joints = PredefinedJointStates()
    Initial_Joints.joint_states = [0.0, -2.39, 2.4, -3.15, -1.58, 3.15]
    Initial_Joints.name = "Initial_Joints"

    Pre_Drop_Joints = PredefinedJointStates()
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
    rack_file_path = path.join(
        path.dirname(path.realpath(__file__)), "..", "assets", "simpleRack.stl"
    )
    floor_file_path = path.join(
        path.dirname(path.realpath(__file__)), "..", "assets", "floor.stl"
    )

    tolerance = 0.02

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
    arucoPossibleAngles = {
        "left": [0.0, 0.7, 0.7, 0.0],
        "front": [0.5, 0.5, 0.5, 0.5],
        "right": [0.7, 0.0, 0.0, 0.7],
    }

    twist_pub = node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)

    contolMSwitch = node.create_client(SwitchController, "/controller_manager/switch_controller")

    while not contolMSwitch.wait_for_service(timeout_sec=5.0):
        node.get_logger().warn(f"Service control Manager is not yet available...")

    while not node.create_client(SetIO, '/io_and_status_controller/set_io'):
        node.get_logger().info('EEF Tool service not available, waiting again...')
    time.sleep(5)

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
                arucoData[i].eulerAngles = tf3d.euler.quat2euler(
                    arucoData[i].quaternions
                )
                if (
                    round(arucoData[i].quaternions[0], 1) == arucoPossibleAngles["left"][0]
                    and round(arucoData[i].quaternions[1], 1) == arucoPossibleAngles["left"][1]
                    and round(arucoData[i].quaternions[2], 1) == arucoPossibleAngles["left"][2]
                    and round(arucoData[i].quaternions[3], 1) == arucoPossibleAngles["left"][3]
                    ):
                    arucoData[i].rotationName = "Left"
                elif (
                round(arucoData[i].quaternions[0], 1) == arucoPossibleAngles["right"][0]
                and round(arucoData[i].quaternions[1], 1) == arucoPossibleAngles["right"][1]
                and round(arucoData[i].quaternions[2], 1) == arucoPossibleAngles["right"][2]
                and round(arucoData[i].quaternions[3], 1) == arucoPossibleAngles["right"][3]
                ):
                    arucoData[i].rotationName = "Right"
                else:
                    arucoData[i].rotationName = "Front"

    for aruco in arucoData:
        print(
            "Aruco Name: ",
            aruco.name,
            "\nPosition: ",
            aruco.position,
            "\nQuaternions: ",
            list(np.around(np.array(aruco.quaternions), 2)),
            "\nEuler Angles: ",
            list(np.around(np.array(aruco.eulerAngles), 2)),
            "\n",
        )
    def switch_controller(useMoveit: bool):
        contolMSwitch = node.create_client(SwitchController, "/controller_manager/switch_controller")
        # Parameters to switch controller
        switchParam = SwitchController.Request()
        if useMoveit == True :
            switchParam.activate_controllers = ["scaled_joint_trajectory_controller"] # for normal use of moveit
        else:
            switchParam.deactivate_controllers = ["forward_position_controller"] # for servoing
        switchParam.strictness = 2
        switchParam.start_asap = False

        # calling control manager service after checking its availability
        while not contolMSwitch.wait_for_service(timeout_sec=5.0):
            node.get_logger().warn(f"Service control Manager is not yet available...")
        contolMSwitch.call_async(switchParam)
        time.sleep(1.0)
        print("[CM]: Switching to","Moveit" if useMoveit else "Servo","Complete")

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
            path = rack_file_path
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
        return tempPose, tempQuats

    def controlGripper(status, box_name):
        '''
        based on the state given as i/p the service is called to activate/deactivate
        pin 16 of TCP in UR5
        i/p: node, state of pin:Bool
        o/p or return: response from service call
        '''
        if status == "ON":
            state = 1
        else:
            state = 0
        gripper_control = node.create_client(SetIO, '/io_and_status_controller/set_io')
        while not gripper_control.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('EEF Tool service not available, waiting again...')
        req         = SetIO.Request()
        req.fun     = 1
        req.pin     = 16
        req.state   = float(state)
        gripper_control.call_async(req)
        print("Gripper Status: ", status, "has been requested")
        time.sleep(1.0)
        return state

    def checkSphericalTolerance(currentPose, targetPose, tolerance):
        currentTolerance = math.sqrt(
            (currentPose[0] - targetPose[0]) ** 2
            + (currentPose[1] - targetPose[1]) ** 2
            + (currentPose[2] - targetPose[2]) ** 2
        )
        print("Distance left: ", currentTolerance, "\n")
        return True if currentTolerance <= tolerance else False, currentTolerance

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

    def moveToPoseWithServo(TargetPose, quaternions, tolerance=0.2):
        switch_controller(useMoveit=False)
        global servo_status
        mission_status = True
        moveit2Servo.enable()
        sphericalToleranceAchieved = False
        currentPose = getCurrentPose(TargetPose=TargetPose, TargetQuats=quaternions)[0]
        _, magnitude = checkSphericalTolerance(currentPose, TargetPose, tolerance)
        magnitude *= 3
        vx, vy, vz = (
            (TargetPose[0] - currentPose[0]) / magnitude,
            (TargetPose[1] - currentPose[1]) / magnitude,
            (TargetPose[2] - currentPose[2]) / magnitude,
        )
        while sphericalToleranceAchieved == False and servo_status == 0:
            moveWithServo([vx, vy, vz], [0.0, 0.0, 0.0])
            # print("Vx:", vx, "Vy:", vy, "Vz:", vz)
            currentPose = getCurrentPose(
                TargetPose=TargetPose, TargetQuats=quaternions
            )[0]
            sphericalToleranceAchieved, _ = checkSphericalTolerance(
                currentPose, TargetPose, tolerance
            )
            time.sleep(0.01)
            if servo_status > 0:
                mission_status = False
                print("Exited While Loop due to Servo Error", servo_status)
                break
        switch_controller(useMoveit=True)
        return mission_status

    def moveToPose(position, quaternions, position_name, rotation_name, dropData):
        def servo_status_updater(msg):
            global servo_status
            servo_status = msg.data
            # print("Servo Status: ", servo_status)

        def joint_states_updater(msg):
            global current_joint_states
            current_joint_states = list([states for states in msg.position])
            # print("Joint States: ", joint_states)

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
        position = [
            round(position[0], 2),
            round(position[1], 2),
            round(position[2], 2),
        ]
        # midPosition = [1 * position[0] / 2, 1 * position[1] / 2, position[2]]
        if rotation_name == "Left":
            midPosition = [position[0], position[1] - 0.23, position[2]]
        elif rotation_name == "Right":
            midPosition = [position[0], position[1] + 0.23, position[2]]
        else:
            midPosition = [position[0] - 0.23, position[1], position[2]]
        quaternions = [
            round(quaternions[0], 4),
            round(quaternions[1], 4),
            round(quaternions[2], 4),
            round(quaternions[3], 4),
        ]
        box_name = "box" + str(int(re.search(r"\d+", position_name).group()))
        # quaternions = P2.quaternions

        # x, y, z = False, False, False
        # currentPose = [0, 0, 0, 0]
        # while True:
        #     print("Moving to ", position_name, "    [Attempt: ", counter, "]")
        #     # if position_name != "Drop":
        #     moveit2.move_to_pose(
        #         position=midPosition,
        #         quat_xyzw=quaternions,
        #         tolerance_position=0.01,
        #         tolerance_orientation=0.01,
        #     )
        #     status = moveit2.wait_until_executed()
        #     counter += 1
        #     if status == False:
        #         continue
        #     else:
        #         break

        # if position_name != "Drop":
        if rotation_name == "Left":
            moveToJointStates(Pickup_Joints_Left.joint_states, Pickup_Joints_Left.name)
        elif rotation_name == "Right":
            moveToJointStates(Pickup_Joints_Right.joint_states, Pickup_Joints_Right.name)
        else:
            moveToJointStates(Pickup_Joints_Front.joint_states, Pickup_Joints_Front.name)

        temp_result = moveToPoseWithServo(TargetPose=position, quaternions=quaternions)
        print("Servo Result: ", temp_result)
        global_counter = 0
        while global_counter < 5:
            if temp_result == False:
                while True:
                    print("Moving to ", position_name, "    [Attempt: ", counter, "Global Attempt: ", global_counter, "]")
                    # if position_name != "Drop":
                    moveit2.move_to_pose(
                        position=midPosition,
                        quat_xyzw=quaternions,
                        tolerance_position=0.01,
                        tolerance_orientation=0.01,
                    )
                    status = moveit2.wait_until_executed()
                    counter += 1
                    if status == False:
                        continue
                    else:
                        break
                temp_result = moveToPoseWithServo(TargetPose=position, quaternions=quaternions)
                if global_counter > 4:
                    print("[ERROR !!!] Failed to reach",position_name,"after 5 attempts, skipping to next box")
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
        time.sleep(0.1)
        controlGripper("ON", box_name)
        time.sleep(0.2)
        # return

        for i in range(3):
            moveit2.add_collision_mesh(
                filepath=box_file_path,
                id="currentBox",
                position=[0.0, -0.12, 0.11],
                quat_xyzw=[-0.5, 0.5, 0.5, 0.5],
                frame_id="tool0",
            )
            time.sleep(0.2)

        # newMidPose = [position[0] / 2, position[1] / 2, midPosition[2]]
        moveToPoseWithServo(TargetPose=midPosition, quaternions=quaternions)
        # if servo_status > 0:
        #         print("Exited next While Loop due to Servo Error", servo_status)
        #         continue
        print("Tolerance Achieved: Came out")
        time.sleep(0.1)

        # Move to Pre Drop Pose
        moveToJointStates(Pre_Drop_Joints.joint_states, Pre_Drop_Joints.name)
        print("Reached Pre-Drop")

        # Move to Drop Pose
        moveToJointStates(dropData.joint_states, dropData.name)
        print("Reached Drop")

        controlGripper("OFF", box_name)

        for i in range(3):
            moveit2.remove_collision_mesh(id="currentBox")
            time.sleep(0.2)
        time.sleep(0.2)

        # Move to Pre Drop Pose
        # moveToJointStates(Pre_Drop_Joints.joint_states, Pre_Drop_Joints.name)

        moveToJointStates(Initial_Joints.joint_states, Initial_Joints.name)
        print("Reached Initial Pose")

        servoNode.destroy_node()
        jointStatesNode.destroy_node()


    
    collisionObjectDistances = {"left": 0.0, "front": 0.0, "right": 0.0}
    left_flag, front_flag, right_flag = False, False, False
    for aruco in arucoData:
        if left_flag == False:
            if (
                round(aruco.quaternions[0], 1) == arucoPossibleAngles["left"][0]
                and round(aruco.quaternions[1], 1) == arucoPossibleAngles["left"][1]
                and round(aruco.quaternions[2], 1) == arucoPossibleAngles["left"][2]
                and round(aruco.quaternions[3], 1) == arucoPossibleAngles["left"][3]
            ):
                left_flag = True
                collisionObjectDistances["left"] = round(aruco.position[1], 2) + 0.16
        if front_flag == False:
            print(aruco.name)
            if (
                round(aruco.quaternions[0], 1) == arucoPossibleAngles["front"][0]
                and round(aruco.quaternions[1], 1) == arucoPossibleAngles["front"][1]
                and round(aruco.quaternions[2], 1) == arucoPossibleAngles["front"][2]
                and round(aruco.quaternions[3], 1) == arucoPossibleAngles["front"][3]
            ):
                front_flag = True
                collisionObjectDistances["front"] = round(aruco.position[0], 2) + 0.16
        if right_flag == False:
            print(aruco.name)
            if (
                round(aruco.quaternions[0], 1) == arucoPossibleAngles["right"][0]
                and round(aruco.quaternions[1], 1) == arucoPossibleAngles["right"][1]
                and round(aruco.quaternions[2], 1) == arucoPossibleAngles["right"][2]
                and round(aruco.quaternions[3], 1) == arucoPossibleAngles["right"][3]
            ):
                right_flag = True
                collisionObjectDistances["right"] = round(aruco.position[2], 2) + 0.0
    print(
        "Left Flag: ", left_flag, "Front Flag: ", front_flag, "Right Flag: ", right_flag
    )
    if left_flag:
        print("Adding Left Collision Object")
        # addCollisionObject("floor", "left_floor", [0.25, 0.71, 0.16], "Left", "base_link")
        addCollisionObject(
            "floor",
            "left_Rack",
            [0.25, collisionObjectDistances["left"], 0.16],
            "Left",
            "base_link",
        )
    if front_flag:
        print("Adding Front Collision Object")
        # addCollisionObject("floor", "front_floor", [0.54, 0.07, 0.16], "Front", "base_link")
        addCollisionObject(
            "floor",
            "front_Rack",
            [collisionObjectDistances["front"], 0.07, 0.16],
            "Front",
            "base_link",
        )
    if right_flag:
        print("Adding Right Collision Object")
        # addCollisionObject("floor", "right_floor", [0.25, -0.65, 0.16], "Right", "base_link")
        addCollisionObject(
            "floor",
            "right_Rack",
            [0.25, -1 * collisionObjectDistances["right"], 0.16],
            "Right",
            "base_link",
        )

    moveit2.add_collision_mesh(
        filepath=floor_file_path,
        id="Floor",
        position=[-0.04, 0.03, -0.04],
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        frame_id="base_link",
    )

    # moveToJointStates(Initial_Joints.joint_states, Initial_Joints.name)

    for aruco, drop in zip(arucoData, Drop_Joints_List):
        moveToPose(
            aruco.position, aruco.quaternions, aruco.name, aruco.rotationName, drop
        )
        # print("Reached ", aruco.name)
        # moveToPose(Drop.position, Drop.quaternions, "Drop")
        # print("Reached Drop")

    print("Done")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()