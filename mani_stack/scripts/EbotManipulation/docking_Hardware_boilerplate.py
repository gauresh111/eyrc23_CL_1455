#!/usr/bin/env python3

## Overview

# ###
# This ROS2 script is designed to control a robot's docking behavior with a rack. 
# It utilizes odometry data, ultrasonic sensor readings, and provides docking control through a custom service. 
# The script handles both linear and angular motion to achieve docking alignment and execution.
# ###

'''
# Team ID:          < 1455 >
# Theme:            < Cosmo Logistic >
# Author List:      < joel Devasia , Gauresh Wadekar >
# Filename:         < docking_Hardware_boilerplate.py >
# Functions:        < "moveBot", "reset_odom", "reset_imu", "normalize_angle", "calculate_distance", "GlobalStopTime", "is_robot_within_tolerance", "getWhichIsGreater", "distanceSingle", "Whichaxistomove", "odomLinearDockingprocess", "odomLinearDocking", "UltraOrientation", "UltraOrientationLinear", "AngularDocking", "find_string_in_list", "cameraYawConversion", "manualMoveBot", "is_yaw_within_tolerance", "cameraOrientation", "controller_loop", "dock_control_callback" >
# Global variables: < "robot_pose", "ultrasonic_value", "aruco_name_list", "aruco_angle_list", "aruco_ap_list" >
'''

# Import necessary ROS2 packages and message types
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from mani_stack.srv import DockSw  # Import custom service message
from sensor_msgs.msg import Imu
from usb_relay.srv import RelaySw # type: ignore
from std_srvs.srv import Trigger
import math
from threading import Thread
from rclpy.time import Time
from std_msgs.msg import Bool,Float32MultiArray,String
import yaml

rclpy.init()
global robot_pose
global ultrasonic_value
from tf_transformations import euler_from_quaternion,quaternion_from_euler
global aruco_name_list
global aruco_angle_list
global aruco_ap_list
aruco_name_list = []
aruco_angle_list = []
aruco_ap_list = []
ultrasonic_value = [0.0, 0.0]
robot_pose = [0.0, 0.0, 0.0,0.0]


class pid():
    def __init__(self):
        self.angleKp = 0.04
        self.linearKp = 0.5
        self.error = 0
        self.lastError = 0
        self.odomLinear = 0.5
        self.ultraKp=0.008
    def computeAngle(self ,setPoint, Input,X,Y):
        error = Input - setPoint                                         
        output = self.angleKp * error
        
        if(output > 0.4):
            output = 0.4
        elif(output < 0.2 and output > 0.0):
            output = 0.2
        elif(output < -0.4):
            output = -0.4
        elif(output > -0.2 and output < 0.0):
            output = -0.2         
        print("Input",Input,"setPoint",setPoint,"error",error,"output",output)
        return output*-1.0
    def computeLinear(self,InputY,setPointY):
        error = InputY - setPointY                                         
        output = self.linearKp * error  
        if output < 0.1:
            output = 0.1
        # print("InputY",InputY,"setPointY",setPointY,"error",error,"output",output)
        return output*-1.0    
    def odomComputeLinear(self,Input,Setpoint):
        error = Input - Setpoint                                         
        output = self.odomLinear * error  
        if output < 0.1:
            output = 0.1
        return output*-1.0
    def UltraOrientation(self,input,isLinear):
        global ultrasonic_value
        error = input
        output = self.ultraKp * error
        output = round(output,3)
        result = False
        if output > 0.08:
            output = 0.08
        elif output < -0.08:
            output = -0.08
        if abs(round(error,3))<=3.0:
            result = True 
        mode = "Linear" if isLinear else "Angular"
       
        print(f"mode {mode} usrleft_value Left: {round(ultrasonic_value[0], 1)} usrright_value Right: {round(ultrasonic_value[1], 1)} error: {error} output: {output}")
        
        
        return output*-1.0,result
    # def computeLinear(self, Input ,setPoint):
    #     error = Input - setPoint                                          
    #     output = self.kp * error + self.kd * (error - self.lastError) + self.ki * (self.ki + error)
    #     self.lastError = error
    #     output = output + 1
    #     return output
# Define a class for your ROS2 node

class MyRobotDockingController(Node):

    def __init__(self):
        # Initialize the ROS2 node with a unique name
        super().__init__('my_robot_docking_controller')
        global robot_pose
        # Create a callback group for managing callbacks
        self.callback_group = ReentrantCallbackGroup()
        self.reset_imu()
        self.reset_imu()
        for i in range(6):                                    # Reset IMU data
            self.reset_odom()       
        self.get_logger().warn("imu and odom reset done")
        # Subscribe to odometry data for robot pose information
        

        self.dock_control_srv = self.create_service(DockSw, '/dock_control', self.dock_control_callback, callback_group=self.callback_group)
        self.speedPub = self.create_publisher(Twist, '/cmd_vel', 30)
        self.nav2speedPub = self.create_publisher(Twist, '/cmd_vel_nav', 30)
        # Initialize all  flags and parameters here
        self.is_docking = False
        self.dock_aligned=False
        self.targetX=0
        self.targetY=0
        self.targetYaw=0
        self.rackName = ""
        self.isAttach = False
        self.globalnodeClock = self.get_clock()
        self.isRackDetach=False 
        self.BoxId = None                 
        #         
        # 
        # 
        # 
        # 
        # 
       
        # Initialize a timer for the main control loop
        self.controller_timer = self.create_timer(0.1, self.controller_loop)

    
    def moveBot(self,linearSpeedX,angularSpeed):
        '''
        Purpose:
        ---
        <  This function controls the robot's linear and angular movement by setting the linear and angular velocities. >

        Input Arguments:
        ---
        `< linearSpeedX >` :  [ < float > ]
            < desired linear speed of the robot along the X-axis in meters per second >

        `< angularSpeed >` :  [ < float > ]
            < desired angular speed of the robot around the Z-axis in radians per second. >

        Returns:
        ---
        `<The function does not explicitly return any value.>`
        
        Example call:
        ---
        < moveBot(self, 0.5, -1.0) >
        '''
        twist = Twist()
        twist.linear.x = linearSpeedX
        twist.angular.z = angularSpeed
        self.speedPub.publish(twist)
        
    def reset_odom(self):
        '''
        Purpose:
        ---
        Resets the robot's odometry data. This is typically used to set the robot's position and orientation to a known starting point.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.reset_odom()
        '''
        self.get_logger().info('Resetting Odometry. Please wait...')
        self.reset_odom_ebot = self.create_client(Trigger, 'reset_odom')
        while not self.reset_odom_ebot.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/reset_odom service not available. Waiting for /reset_odom to become available.')
            
        self.request_odom_reset = Trigger.Request()
        self.odom_service_resp=self.reset_odom_ebot.call_async(self.request_odom_reset)
        # while self.odom_service_resp is None:
        #     self.GlobalStopTime(0.1)
        rclpy.spin_until_future_complete(self, self.odom_service_resp)
        if(self.odom_service_resp.result().success == True):
            self.get_logger().info(self.odom_service_resp.result().message)
        else:
            self.get_logger().warn(self.odom_service_resp.result().message)
    def reset_imu(self):
        '''
        Purpose:
        ---
        Resets the robot's IMU (Inertial Measurement Unit) data. This is typically used to set the robot's orientation to a known starting point.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None
        
        Example call:
        ---
        self.reset_imu()
        '''

        self.get_logger().info('Resetting IMU. Please wait...')
        self.reset_imu_ebot = self.create_client(Trigger, 'reset_imu')
        while not self.reset_imu_ebot.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/reset_imu service not available. Waiting for /reset_imu to become available.')

        request_imu_reset = Trigger.Request()
        self.imu_service_resp=self.reset_imu_ebot.call_async(request_imu_reset)
        # while self.imu_service_resp is None:
        #     self.GlobalStopTime(0.1)
        rclpy.spin_until_future_complete(self, self.imu_service_resp)
        if(self.imu_service_resp.result().success== True):
            self.get_logger().info(self.imu_service_resp.result().message)
        else:
            self.get_logger().warn(self.imu_service_resp.result().message)

    
    def normalize_angle(self,angle):
        '''
        Purpose:
        ---
        Normalizes an angle to the range [0, 2π]. This is useful for calculations involving angles that may wrap around beyond the full circle.

        Input Arguments:
        ---
        * `angle`: [float] - The angle to be normalized, in radians.

        Returns:
        ---
        * `normalized_angle`: [float] - The normalized angle, in degrees, within the range [0, 2π].

        Example call:
        ---
        normalized_angle = self.normalize_angle(3.14159)  # pi radians
        print(normalized_angle)  # Output: 0.0 # degrees
        '''

        global robot_pose
        if self.targetYaw == 0.0:
            return angle

        if angle<0:
            angle = angle + 360
        return angle
    
    # Main control loop for managing docking behavior
    
    def calculate_distance(self,x1, y1, x2, y2):
        '''
        Purpose:
        ---
        Calculates the Euclidean distance between two points in a 2D space.

        Input Arguments:
        ---
        * `x1`: [float] - The X coordinate of the first point.
        * `y1`: [float] - The Y coordinate of the first point.
        * `x2`: [float] - The X coordinate of the second point.
        * `y2`: [float] - The Y coordinate of the second point.

        Returns:
        ---
        * `distance`: [float] - The Euclidean distance between the two points.

        Example call:
        ---
        distance = self.calculate_distance(1.0, 2.0, 3.0, 4.0)
        print(distance)  # Output: 2.8284271247461903
        '''
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)   
    def GlobalStopTime(self,StopSeconds):
        '''
        Purpose:
        ---
        Pauses the program execution for a specified number of seconds. This can be useful for waiting between actions or synchronizing with other parts of the code.

        Input Arguments:
        ---
        * `StopSeconds`: [float] - The number of seconds to pause execution for.

        Returns:
        ---
        None

        Example call:
        ---
        self.GlobalStopTime(2.0)  # Pause for 2 seconds
        '''
        future_time = Time(seconds=self.globalnodeClock.now().nanoseconds / 1e9 + StopSeconds, clock_type=self.globalnodeClock.clock_type)
        self.globalnodeClock.sleep_until(future_time)
    def is_robot_within_tolerance(self,current_x, current_y, current_orientation, goal_x, goal_y, goal_orientation,
                                x_tolerance=0.3, y_tolerance=0.3, orientation_tolerance=10):
        '''
        Purpose:
        ---
        Checks if the robot is within a specified tolerance level of the desired position and orientation. This is useful for determining when the robot has reached its goal.

        Input Arguments:
        ---
        * `current_x`: [float] - The current X coordinate of the robot.
        * `current_y`: [float] - The current Y coordinate of the robot.
        * `current_orientation`: [float] - The current orientation of the robot, in radians.
        * `goal_x`: [float] - The X coordinate of the desired goal position.
        * `goal_y`: [float] - The Y coordinate of the desired goal position.
        * `goal_orientation`: [float] - The desired goal orientation, in radians.
        * `x_tolerance`: [float, optional] - The tolerance level for the X coordinate, in meters. Defaults to 0.3.
        * `y_tolerance`: [float, optional] - The tolerance level for the Y coordinate, in meters. Defaults to 0.3.
        * `orientation_tolerance`: [float, optional] - The tolerance level for the orientation, in degrees. Defaults to 10.

        Returns:
        ---
        * `is_within_tolerance`: [bool] - True if the robot is within the specified tolerances, False otherwise.

        Example call:
        ---
        # Check if the robot is within 0.2 meters in X and Y, and 5 degrees in orientation from the goal
        is_within_tolerance = self.is_robot_within_tolerance(1.0, 2.0, 1.57, 1.1, 2.1, 1.57, 0.2, 0.2, 5)
        print(is_within_tolerance)
        '''
        x_difference = abs(goal_x)-abs(current_x)
        y_difference = abs(goal_y)-abs(current_y)
        orientation_difference = abs(goal_orientation) - abs(current_orientation)
        print("x_difference",x_difference,"y_difference",y_difference,"orientation_difference",orientation_difference)
        x_within_tolerance = x_difference <= x_tolerance
        y_within_tolerance = y_difference <= y_tolerance
        orientation_within_tolerance = orientation_difference <= orientation_tolerance

        return x_within_tolerance and y_within_tolerance and orientation_within_tolerance
    
    def getWhichIsGreater(self,currentX,currentY):
        '''
        Purpose:
        ---
        Determines which axis (X or Y) has a greater difference between the current and target positions. This is useful for deciding which axis the robot should move along first.

        Input Arguments:
        ---
        * `currentX`: [float] - The current X coordinate of the robot.
        * `currentY`: [float] - The current Y coordinate of the robot.

        Returns:
        ---
        * `axis`: [int] - The axis with the greater difference ("0" or "1").

        Example call:
        ---
        axis = self.getWhichIsGreater(1.0, 3.0)
        print(axis)  # Output: "1"
        '''
        goalX = self.targetX
        goalY = self.targetY
        AbsdifferenceX = abs(abs(currentX) - abs(goalX))
        AbsdifferenceY = abs(abs(currentY) - abs(goalY))
        # print("AbsdifferenceX",AbsdifferenceX,"AbsdifferenceY",AbsdifferenceY)
        if AbsdifferenceX > AbsdifferenceY:
            # print("X is greater")
            return 0
        else:
            # print("Y is greater")
            return 1

        
        
        
    def distanceSingle(self,x1, x2):
        '''
        Purpose:
        ---
        Calculates the absolute difference between two values. This is a simpler version of `calculate_distance` that only works for one-dimensional data.

        Input Arguments:
        ---
        * `x1`: [float] - The first value.
        * `x2`: [float] - The second value.

        Returns:
        ---
        * `difference`: [float] - The absolute difference between the two values.

        Example call:
        ---
        difference = self.distanceSingle(5.0, 2.0)
        print(difference)  # Output: 3.0
        '''
        return math.sqrt((x1 - x2) ** 2)*1.0
    def Whichaxistomove(self):
        '''
        Purpose:
        ---
        Determines which axis (X or Y) the robot needs to move along first based on the absolute yaw angle. This is useful for determining the initial movement direction during docking.

        Input Arguments:
        ---
        None

        Returns:
        ---
        * `axis`: [int] - The axis to move along first ("1" or "0").

        Example call:
        ---
        axis = self.Whichaxistomove()
        print(axis)
        '''
        absolute_yaw = abs(self.targetYaw)
        return 1 if absolute_yaw > 200.0 else 0 if absolute_yaw > 150.0 else 1 if absolute_yaw > 80.0 else 0
    def odomLinearDockingprocess(self,InputDistance,Setpoint=0.1):
        '''
        Purpose:
        ---
        Calculates a linear speed command using a PID controller based on the input distance. This is used for controlling the robot's movement during docking using odometry data.

        Input Arguments:
        ---
        * `InputDistance`: [float] - The desired distance to move.
        * `Setpoint`: [float, optional] - The target value for the PID controller. Defaults to 0.1.

        Returns:
        ---
        * `MotorSpeed`: [float] - The calculated linear speed command for the motor.

        '''
        odomlinearPid = pid()
        if InputDistance <0.4:   
            return 0.0
        return odomlinearPid.odomComputeLinear(InputDistance,Setpoint)
    def odomLinearDocking(self):
        '''
        Purpose:
        ---
        Performs linear docking using the odometry data and the `odomLinearDockingprocess` function. This function likely controls the robot's movement along a specific axis until it reaches the desired position based on odometry readings.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.odomLinearDocking()
        '''
        global robot_pose
        reachedExtra = False    
        X1 = self.Whichaxistomove()
        while (reachedExtra == False):
            if X1 == 0:
                distance=self.distanceSingle(self.targetX,robot_pose[0])
                if distance < 0.4:
                    reachedExtra = True
                print("X: target",self.targetX,"current",robot_pose[0],"distance",distance)
            elif X1 == 1:
                distance=self.distanceSingle(self.targetY,robot_pose[1])
                if distance < 0.4:
                    reachedExtra = True
                print("Y: target",self.targetY,"current",robot_pose[1],"distance",distance)
            speed=self.odomLinearDockingprocess(distance)
            self.moveBot(speed,0.0)
            self.GlobalStopTime(0.1)
    def UltraOrientation(self):
        '''
        Purpose:
        ---
        Performs ultrasonic sensor-based orientation adjustment using a PID controller. This function likely uses ultrasonic sensor readings to adjust the robot's orientation during docking.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.UltraOrientation()
        '''
        global ultrasonic_value
        reached = False
        ultrasonicPid = pid()
        while (reached == False):
            m = (ultrasonic_value[1] - ultrasonic_value[0])
            angularValue,reached = ultrasonicPid.UltraOrientation(m,False)
            print("m:",m)
            self.moveBot(0.0,angularValue)
            self.GlobalStopTime(0.1)
    def UltraOrientationLinear(self,Setpoint):
        '''
        Purpose:
        ---
        Performs linear movement based on ultrasonic sensor readings until a specified distance is reached. This function likely uses ultrasonic sensor readings to control the robot's linear movement until it reaches a certain distance.

        Input Arguments:
        ---
        * `Setpoint`: [float] - The target value for the PID controller, likely related to the desired distance.

        Returns:
        ---
        None

        Example call:
        ---
        self.UltraOrientationLinear(0.2)  # Move until 0.2 meters away based on ultrasonic sensor readings
        '''
        global ultrasonic_value
        reached = False
        ultrasonicPid = pid()
        linearValue = -0.05
        while (reached == False):
            m = (ultrasonic_value[1] - ultrasonic_value[0])
            angularValue ,check = ultrasonicPid.UltraOrientation(m,True)
            self.moveBot(linearValue,0.0)
            avgUltraSonic = (ultrasonic_value[0]+ultrasonic_value[1])/2
            if avgUltraSonic <Setpoint:
                reached = True
            self.GlobalStopTime(0.1)    
    def AngularDocking(self):   
        '''
        Purpose:
        ---
        Performs angular docking using the IMU data and a PID controller. This function likely uses the IMU (Inertial Measurement Unit) data to adjust the robot's orientation until it reaches the desired angle during docking.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.AngularDocking()
        '''
        yaw = False
        botPid = pid()
        while (yaw == False):
            angle=botPid.computeAngle(int(self.normalize_angle(self.targetYaw)),int(self.normalize_angle(robot_pose[2])),robot_pose[0],robot_pose[1])
            self.moveBot(0.0,angle)
            yaw = True if(int(self.normalize_angle(self.targetYaw)) == int(self.normalize_angle(robot_pose[2]))) else False
            self.GlobalStopTime(0.1)
        self.moveBot(0.0,0.0)
        self.moveBot(0.0,0.0)
    
    def find_string_in_list(self,string, list):
        '''
        Purpose:
        ---
        Searches for a string in a list and returns its index if found, or -1 otherwise. This function is likely used for searching through lists of strings, possibly for configuration or data manipulation.

        Input Arguments:
        ---
        * `string`: [str] - The string to search for.
        * `list`: [list] - The list to search in.

        Returns:
        ---
        * `index`: [int] - The index of the string in the list if found, or -1 if not found.

        Example call:
        ---
        index = self.find_string_in_list("key", ["value1", "key", "value3"])
        print(index)  # Output: 1
        '''
        for index, item in enumerate(list):
            if item == string:
                return index
        return -1
    def cameraYawConversion(self,yaw):
        '''
        Purpose:
        ---
        Converts the camera yaw angle to a format suitable for the robot. This function likely adjusts the camera yaw reading to match the robot's internal coordinate system or control requirements.

        Input Arguments:
        ---
        * `yaw`: [float] - The camera yaw angle.

        Returns:
        ---
        * `converted_yaw`: [float] - The converted yaw angle suitable for the robot.

        Example call:
        ---
        converted_yaw = self.cameraYawConversion(45.0)
        print(converted_yaw)
        '''
        if self.targetYaw >= 178.0 and self.targetYaw <= 182.0:
            return 180 - yaw

        if yaw>0:
            yaw =  360 - yaw
        return int(yaw)
    def manualMoveBot(self):
        '''
        Purpose:
        ---
        Allows manual control of the robot through user input. This function likely provides a way to manually control the robot's movement during testing or development.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.manualMoveBot()
        '''
        global robot_pose,aruco_name_list,aruco_angle_list,aruco_ap_list
        # value = input("Move Bot")
        target_rack = "obj_"+self.BoxId#"rack3"
        #targetrack = "obj_3"
        self.GlobalStopTime(1.0)
        
        # if value == "y":
        for i in range(3):
            rackIndex = self.find_string_in_list(target_rack,aruco_name_list)
            counter = 1
            while rackIndex == -1:
                counter = counter + 1
                if counter > 100:
                    self.moveBot(0.0,0.0)
                    self.moveBot(0.0,0.0)
                    return None
                rackIndex = self.find_string_in_list(target_rack,aruco_name_list)
                print("running manual mode")
                print("rackIndex",rackIndex)
                print("target_rack",target_rack)
                print("aruco_ap_list",aruco_ap_list)
                print("aruco_name_list",aruco_name_list)
                print("x",robot_pose[0],"y",robot_pose[1])
                # print("cameraYaw",cameraYaw)
                self.GlobalStopTime(0.1)
                self.moveBot(-0.05,0.0)
            self.GlobalStopTime(1.0)
            self.moveBot(0.0,0.0)
            self.moveBot(0.0,0.0)
        return None
    
    def is_yaw_within_tolerance(self,current_yaw, target_yaw, tolerance=5):
        '''
        Purpose:
        ---
        Checks if the current yaw angle is within a specified tolerance level of the target yaw angle. This function is likely used for determining when the robot's orientation is close enough to the desired angle during docking.

        Input Arguments:
        ---
        * `current_yaw`: [float] - The current yaw angle of the robot.
        * `target_yaw`: [float] - The target yaw angle.
        * `tolerance`: [float, optional] - The tolerance level for the yaw angle, in degrees. Defaults to 5.

        Returns:
        ---
        * `is_within_tolerance`: [bool] - True if the current yaw is within the specified tolerance, False otherwise.

        Example call:
        ---
        is_within_tolerance = self.is_yaw_within_tolerance(30.0, 45.0)
        print(is_within_tolerance)  # Output: True (depending on the tolerance value)
        '''
        # Calculate the difference between the yaw angles
        yaw_diff = abs(current_yaw - target_yaw)

        # Check if the difference is within the tolerance
        return yaw_diff <= tolerance
    def cameraOrientation(self):
        '''
        Purpose:
        ---
        Performs docking using the camera data to adjust the robot's orientation. This function likely uses the camera to detect specific features or landmarks to adjust the robot's orientation during docking.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.cameraOrientation()
        '''
        global aruco_name_list,aruco_angle_list,aruco_ap_list
        self.manualMoveBot()
        botPid = pid()
        target_rack = "obj_"+self.BoxId        
        rackIndex = self.find_string_in_list(target_rack,aruco_name_list)
        targetYAw = int(self.normalize_angle(self.targetYaw))
        counter = 1
        while rackIndex == -1:
            counter = counter + 1
            rackIndex = self.find_string_in_list(target_rack,aruco_name_list)
            print("rackIndex",rackIndex)
            print("target_rack",target_rack)
            print("aruco_ap_list",aruco_ap_list)
            print("counter",counter)
            # print("cameraYaw",cameraYaw)
            self.GlobalStopTime(0.1)
            if counter >100:
                return None
                
        yaw = False
        while yaw == False:
            rackIndex = self.find_string_in_list(target_rack,aruco_name_list)
            cameraYaw = self.cameraYawConversion(aruco_angle_list[rackIndex])
            print("rackIndex",rackIndex)
            print("cameraYaw",cameraYaw)
            print("targetYaw",targetYAw)
            print("yaw Checker",yaw)
            angle=botPid.computeAngle(targetYAw,cameraYaw,robot_pose[0],robot_pose[1])
            self.moveBot(0.0,-angle)
            yaw = self.is_yaw_within_tolerance((cameraYaw),targetYAw)
            print("yaw Checker",yaw)
            self.GlobalStopTime(0.1)
        for i in range(5):
            self.moveBot(0.0,0.0)
    def controller_loop(self):

        # The controller loop manages the robot's linear and angular motion 
        # control to achieve docking alignment and execution
        '''
        Purpose:
        ---
        The main control loop that manages the robot's overall docking behavior. This function likely coordinates and calls other functions responsible for various docking tasks like movement, sensor readings, and orientation adjustments.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.controller_loop()  # This function is likely called continuously within the program.
        '''
        # print("controller loop")
        def odometry_callback(msg):
        # Extract and update robot pose information from odometry message
            global robot_pose
            robot_pose[0] = round(msg.pose.pose.position.x,2)
            robot_pose[1] = round(msg.pose.pose.position.y,2)
            robot_pose[3] = round(msg.pose.pose.position.z,2)
        
        def imu_callback(msg):
            global robot_pose
            quaternion_array = msg.orientation
            orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
            _, _, yaw_new = euler_from_quaternion(orientation_list)
            # yaw = msg.data
            # if yaw > 3.14:
            #     yaw_new = (6.28 - yaw) * 1
            # else:
            #     yaw_new = yaw * -1
            # # print(yaw_new)
            yaw = math.degrees(yaw_new)
            robot_pose[2] = round(yaw,2)
        def ultrasonic_callback(msg):
            global ultrasonic_value
            ultrasonic_value[0] = round(msg.data[4],4)
            ultrasonic_value[1] = round(msg.data[5],4)  
            # print("ultrasonic_value",ultrasonic_value)
        def aruco_data_updater(msg):
            global aruco_name_list
            global aruco_angle_list
            global aruco_ap_list
            data = yaml.safe_load(msg.data)
            aruco_name_list = data.get("id")
            aruco_angle_list = data.get("angle")
            aruco_ap_list = data.get("ap")
        if self.is_docking:
            # ...
            # Implement control logic here for linear and angular motion
            # For example P-controller is enough, what is P-controller go check it out !
            # ...
            global robot_pose
            global ultrasonic_value
            dockingNode = Node("GlobalNodeDocking")
            callback_group = ReentrantCallbackGroup()
            docking_executor = MultiThreadedExecutor(1)
            docking_executor.add_node(dockingNode)
            executor_thread = Thread(target=docking_executor.spin, daemon=True, args=())
            executor_thread.start()
            dockingNode.odom_sub = dockingNode.create_subscription(Odometry, '/odom', odometry_callback, 10,callback_group=callback_group)
            dockingNode.imu_sub = dockingNode.create_subscription(Imu, 'sensors/imu1',imu_callback, 10,callback_group=callback_group)
            dockingNode.ultra_sub = dockingNode.create_subscription(Float32MultiArray, '/ultrasonic_filter', ultrasonic_callback, 10,callback_group=callback_group)
            dockingNode.aruco_data_subscriber = dockingNode.create_subscription(String, "/aruco_data", aruco_data_updater, 10, callback_group=callback_group)

            dockingNodeClock = dockingNode.get_clock()
            dockingNode.trigger_usb_relay = dockingNode.create_client(RelaySw, 'usb_relay_sw')
            while not dockingNode.trigger_usb_relay.wait_for_service(timeout_sec=1.0):
                dockingNode.get_logger().warn('USB Trigger Service not available, waiting...')

            def StopTime(StopSeconds):
                future_time = Time(seconds=dockingNodeClock.now().nanoseconds / 1e9 + StopSeconds, clock_type=dockingNodeClock.clock_type)
                dockingNodeClock.sleep_until(future_time)  
            def stopBot(time,linearSpeedX=0.0,angularSpeed=0.0):
                for i in range(2):
                    self.moveBot(linearSpeedX,angularSpeed)   
                    StopTime(time)  
            def switch_eletromagent(relayState):
                dockingNode.get_logger().info('Changing state of the relay to '+str(relayState))
                request_relay = RelaySw.Request()
                request_relay.relaychannel = True
                request_relay.relaystate = relayState
                
                dockingNode.future=dockingNode.trigger_usb_relay.call_async(request_relay)
                
                while(dockingNode.future.result() is  None):
                    stopBot(0.1)
              
                if(dockingNode.future.result().success== True):
                    dockingNode.get_logger().info(dockingNode.future.result().message)
                else:
                    dockingNode.get_logger().warn(dockingNode.future.result().message)
                
            def rackAttach():
                switch_eletromagent(False)
                stopBot(0.1)
            
                switch_eletromagent(True)
                stopBot(0.1)
                # self.UltraOrientation()
                stopBot(3.0)
                
                self.UltraOrientationLinear(Setpoint=16.0)
                # self.UltraOrientationLinear(Setpoint=16.0)
                stopBot(1.0)
                stopBot(1.2,-0.05,0.0)
                stopBot(2.0)
                
                
                # self.AngularDocking()
                
            for i in range(2):
                self.moveBot(0.0,0.0)   
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.nav2speedPub.publish(twist)
                StopTime(0.1) 
            StopTime(2.0)
            while ultrasonic_value[0] <2.0 or ultrasonic_value[1] < 2.0:
                StopTime(0.1)
                print("waitng for ultraSonic",ultrasonic_value)
            if self.isRackDetach:
                rackAttach()
                self.is_docking = False
                self.dock_aligned=True
                return None
            print("ultrasonic_value_left",ultrasonic_value[0],"ultrasonic_value_right",ultrasonic_value[1])
            self.AngularDocking()
            stopBot(0.1)
            # #orientation done
            if self.isAttach:
                rackAttach()
                stopBot(1.2,0.05,0.0)
                stopBot(0.1)
            else:
                
                self.odomLinearDocking()
                stopBot(0.1) 
                
                self.cameraOrientation() 
                stopBot(0.3)
                switch_eletromagent(False)
                #moving ebot back from rack
                stopBot(0.4,0.8,0.0)
                stopBot(0.1)
            
            self.is_docking = False
            self.dock_aligned=True
            ## docking and orientation done
            dockingNode.destroy_node()
            pass
    
    # Callback function for the DockControl service
    def dock_control_callback(self, request, response):
        # Extract desired docking parameters from the service request
        #
        #
        '''
        Purpose:
        ---
        A callback function that handles requests from the `DockControl` service and starts the docking process. This function is likely triggered by an external system or user interaction to initiate the robot's docking behavior.

        Input Arguments:
        ---
        * `request`: [object] - The request object containing information from the `DockControl` service.
        * `response`: [object] - The response object used to send information back to the `DockControl` service.

        Returns:
        ---
        None

        Example call:
        ---
        # This function is likely called automatically within the program when a request is received from the `DockControl` service.
        '''
        self.targetX = request.goal_x
        self.targetY = request.goal_y
        self.targetYaw = request.orientation
        self.rackName = request.rack_no
        self.isAttach = request.rack_attach
        self.isRackDetach = request.is_rack_detached
        self.BoxId = str(request.box_id)
        # Reset flags and start the docking process
        #
        #
        
        for i in range(2):
            self.moveBot(0.0,0.0)
        
        self.is_docking = True
        self.controller_loop()
        
        # Log a message indicating that docking has started
        self.get_logger().info("Docking started!")

        # Create a rate object to control the loop frequency
        rate = self.create_rate(2, self.get_clock())
        # Wait until the robot is aligned for docking
        while not self.dock_aligned:
            # self.get_logger().info("Waiting for alignment...")
            
            rate.sleep()
        # Set the service response indicating success
        response.success = True
        
        response.message = "Docking control completed "
        print(request)
        return response

# Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    '''
    Purpose:
    ---
    The main entry point of the program. This function is typically where the program execution begins and coordinates the overall docking process.

    Input Arguments:
    ---
    * `args`: [list, optional] - A list of arguments passed to the program from the command line. Defaults to None.

    Returns:
    ---
    None

    Example call:
    ---
    if __name__ == "__main__":
        main()
    '''
    my_robot_docking_controller = MyRobotDockingController()
    executor = MultiThreadedExecutor(2)
    executor.add_node(my_robot_docking_controller)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    def ExitCallBack(msg):
        if msg.data == True:
            raise SystemExit
    exitDocking=my_robot_docking_controller.create_subscription(Bool, '/ExitNav',ExitCallBack, 10)
    try:
        rclpy.spin(my_robot_docking_controller)
    except SystemExit:
        print("SystemExit")
        my_robot_docking_controller.destroy_node()
        rclpy.shutdown()
        exit(0)
    my_robot_docking_controller.destroy_node()
    rclpy.shutdown()
    exit(0)
if __name__ == '__main__':
    main()