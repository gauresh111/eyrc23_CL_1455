#!/usr/bin/env python3

## Overview

# ###
# This ROS2 script is designed to control a robot's docking behavior with a rack. 
# It utilizes odometry data, ultrasonic sensor readings, and provides docking control through a custom service. 
# The script handles both linear and angular motion to achieve docking alignment and execution.
# ###

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

from usb_relay.srv import RelaySw # type: ignore
from std_srvs.srv import Trigger
import math
from threading import Thread
from rclpy.time import Time
from std_msgs.msg import Bool,Float32MultiArray,Float32

rclpy.init()
global robot_pose
global ultrasonic_value
from tf_transformations import euler_from_quaternion,quaternion_from_euler
ultrasonic_value = [0.0, 0.0]
robot_pose = [0.0, 0.0, 0.0,0.0]


class pid():
    def __init__(self):
        self.angleKp = 0.04
        self.linearKp = 0.5
        self.error = 0
        self.lastError = 0
        self.odomLinear = 0.5
        self.ultraKp=0.01
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
        if output < 0.3:
            output = 0.3
        return output*-1.0
    def UltraOrientation(self,input,isLinear):
        global ultrasonic_value
        error = input
        output = self.ultraKp * error
        output = round(output,3)
        result = False
        if output > 0.6:
            output = 0.6
        elif output < -0.6:
            output = -0.6
        if abs(round(error,3))<=5.0:
            result = True 
        mode=""
        if isLinear:
            mode="Linear"
        else:
            mode="Angular"
        print("mode",mode,"usrleft_value Left:",round(ultrasonic_value[0],1)," usrright_value Right:",round(ultrasonic_value[1],1)," error:",error," output:",output)
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
        #         
        # 
        # 
        # 
        # 
        # 
       
        # Initialize a timer for the main control loop
        self.controller_timer = self.create_timer(0.1, self.controller_loop)

    
    def moveBot(self,linearSpeedX,angularSpeed):
        twist = Twist()
        twist.linear.x = linearSpeedX
        twist.angular.z = angularSpeed
        self.speedPub.publish(twist)
        
    
    
    def normalize_angle(self,angle):
        """Normalizes an angle to the range [-π, π].
    
        Args:
            angle: A float representing the angle in radians.

        Returns:
            A float representing the normalized angle in radians.
        """
        global robot_pose
        if self.targetYaw == 0.0:
            return angle

        if angle<0:
            angle = angle + 360
        return angle
    
    # Main control loop for managing docking behavior
    
    def calculate_distance(self,x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)   
    def GlobalStopTime(self,StopSeconds):
        future_time = Time(seconds=self.globalnodeClock.now().nanoseconds / 1e9 + StopSeconds, clock_type=self.globalnodeClock.clock_type)
        self.globalnodeClock.sleep_until(future_time)
    def is_robot_within_tolerance(self,current_x, current_y, current_orientation, goal_x, goal_y, goal_orientation,
                                x_tolerance=0.3, y_tolerance=0.3, orientation_tolerance=10):
        """
        Check if the robot is within tolerance for X axis, Y axis, and Orientation.

        Parameters:
        - current_x: Current X axis position of the robot.
        - current_y: Current Y axis position of the robot.
        - current_orientation: Current orientation of the robot.
        - goal_x: Goal X axis position for the robot.
        - goal_y: Goal Y axis position for the robot.
        - goal_orientation: Goal orientation for the robot.
        - x_tolerance: Tolerance for the X axis (default is 3.0).
        - y_tolerance: Tolerance for the Y axis (default is 3.0).
        - orientation_tolerance: Tolerance for the orientation (default is 10).

        Returns:
        - True if the robot is within tolerance, False otherwise.
        """
        x_difference = abs(goal_x)-abs(current_x)
        y_difference = abs(goal_y)-abs(current_y)
        orientation_difference = abs(goal_orientation) - abs(current_orientation)
        print("x_difference",x_difference,"y_difference",y_difference,"orientation_difference",orientation_difference)
        x_within_tolerance = x_difference <= x_tolerance
        y_within_tolerance = y_difference <= y_tolerance
        orientation_within_tolerance = orientation_difference <= orientation_tolerance

        return x_within_tolerance and y_within_tolerance and orientation_within_tolerance
    
    def getWhichIsGreater(self,currentX,currentY):
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
        return math.sqrt((x1 - x2) ** 2)*1.0
    def Whichaxistomove(self):
        yaw = abs(self.targetYaw) 
        if yaw > 200.0:
            return 1
        elif yaw > 150.0:
            return 0
        elif yaw > 80.0:
            return 1
        else:
            return 0
    def odomLinearDockingprocess(self,InputDistance,Setpoint=0.1):
        odomlinearPid = pid()
        if InputDistance <0.12:   
            return 0.0
        return odomlinearPid.odomComputeLinear(InputDistance,Setpoint)
    def odomLinearDocking(self):
        global robot_pose
        reachedExtra = False    
        X1 = self.Whichaxistomove()
        while (reachedExtra == False):
            if X1 == 0:
                distance=self.distanceSingle(self.targetX,robot_pose[0])
                if distance < 0.02:
                    reachedExtra = True
                print("X: target",self.targetX,"current",robot_pose[0],"distance",distance)
            elif X1 == 1:
                distance=self.distanceSingle(self.targetY,robot_pose[1])
                if distance < 0.02:
                    reachedExtra = True
                print("Y: target",self.targetY,"current",robot_pose[1],"distance",distance)
            speed=self.odomLinearDockingprocess(distance)
            self.moveBot(speed,0.0)
            self.GlobalStopTime(0.1)
    def UltraOrientation(self):
        global ultrasonic_value
        reached = False
        ultrasonicPid = pid()
        while (reached == False):
            m = (ultrasonic_value[1] - ultrasonic_value[0])
            angularValue,reached = ultrasonicPid.UltraOrientation(m,False)
            print("m:",m)
            self.moveBot(0.0,angularValue)
            self.GlobalStopTime(0.1)
    def UltraOrientationLinear(self):
        global ultrasonic_value
        reached = False
        ultrasonicPid = pid()
        linearValue = -0.08
        while (reached == False):
            m = (ultrasonic_value[1] - ultrasonic_value[0])
            angularValue ,check = ultrasonicPid.UltraOrientation(m,True)
            self.moveBot(linearValue,angularValue)
            avgUltraSonic = (ultrasonic_value[0]+ultrasonic_value[1])/2
            if avgUltraSonic <19.0:
                reached = True
            self.GlobalStopTime(0.1)    
    def AngularDocking(self):   
        yaw = False
        botPid = pid()
        while (yaw == False):
            angle=botPid.computeAngle(int(self.normalize_angle(self.targetYaw)),int(self.normalize_angle(robot_pose[2])),robot_pose[0],robot_pose[1])
            self.moveBot(0.0,angle)
            yaw = True if(int(self.normalize_angle(self.targetYaw)) == int(self.normalize_angle(robot_pose[2]))) else False
            self.GlobalStopTime(0.1)
    def controller_loop(self):

        # The controller loop manages the robot's linear and angular motion 
        # control to achieve docking alignment and execution
        
        # print("controller loop")
        def odometry_callback(msg):
        # Extract and update robot pose information from odometry message
            global robot_pose
            robot_pose[0] = round(msg.pose.pose.position.x,2)
            robot_pose[1] = round(msg.pose.pose.position.y,2)
            robot_pose[3] = round(msg.pose.pose.position.z,2)
        
        def imu_callback(msg):
            global robot_pose
            # quaternion_array = msg.orientation
            # orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
            # _, _, yaw = euler_from_quaternion(orientation_list)
            yaw = msg.data
            if yaw > 3.14:
                yaw_new = (6.28 - yaw) * 1
            else:
                yaw_new = yaw * -1
            # print(yaw_new)
            yaw = math.degrees(yaw_new)
            robot_pose[2] = round(yaw,2)
        def ultrasonic_callback(msg):
            global ultrasonic_value
            ultrasonic_value[0] = round(msg.data[4],4)
            ultrasonic_value[1] = round(msg.data[5],4)  
            # print("ultrasonic_value",ultrasonic_value)
        if self.is_docking:
            # ...
            # Implement control logic here for linear and angular motion
            # For example P-controller is enough, what is P-controller go check it out !
            # ...
            global robot_pose
            global ultrasonic_value
            dockingNode = Node("GlobalNodeDocking")
            
            docking_executor = MultiThreadedExecutor(1)
            docking_executor.add_node(dockingNode)
            executor_thread = Thread(target=docking_executor.spin, daemon=True, args=())
            executor_thread.start()
            dockingNode.odom_sub = dockingNode.create_subscription(Odometry, '/odom', odometry_callback, 10)
            dockingNode.imu_sub = dockingNode.create_subscription(Float32, '/orientation', imu_callback, 10)
            dockingNode.ultra_sub = dockingNode.create_subscription(Float32MultiArray, 'ultrasonic_sensor_std_float', ultrasonic_callback, 10)
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
                switch_eletromagent(True)
                self.UltraOrientation()
                stopBot(0.1)
                self.UltraOrientationLinear()
                stopBot(0.1)
                stopBot(0.5,-0.1,0.0)
                stopBot(0.1)
                
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
            else:
                # self.odomLinearDocking()
                stopBot(0.1) 
                stopBot(0.5,-0.1,0.0) #implement odom docking and camera docking
                stopBot(0.4) 
                switch_eletromagent(False)
                #moving ebot back from rack
                stopBot(0.1,2.0,0.0)
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
        self.targetX = request.goal_x
        self.targetY = request.goal_y
        self.targetYaw = request.orientation
        self.rackName = request.rack_no
        self.isAttach = request.rack_attach
        self.isRackDetach = request.is_rack_detached
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