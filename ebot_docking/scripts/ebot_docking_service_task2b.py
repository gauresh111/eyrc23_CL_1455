#!/usr/bin/env python3

## Overview

# ###
# This ROS2 script is designed to control a robot's docking behavior with a rack. 
# It utilizes odometry data, ultrasonic sensor readings, and provides docking control through a custom service. 
# The script handles both linear and angular motion to achieve docking alignment and execution.
# ###

# Import necessary ROS2 packages and message types
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from ebot_docking.srv import DockSw  # Import custom service message
import math, statistics
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import time
from threading import Thread
import numpy as np
from linkattacher_msgs.srv import AttachLink , DetachLink
rclpy.init()

global robot_pose
robot_pose = [0.0, 0.0, 0.0,0.0]


class pid():
    def __init__(self):
        self.angleKp = 0.012
        self.linearKp = 0.012
        self.error = 0
        self.lastError = 0
    def computeAngle(self ,setPoint, Input,X,Y):
        error = Input - setPoint                                         
        output = self.angleKp * error
        
        if(output > 0.6):
            output = 0.6
        if(output < 0.1 and output > 0.0):
            output = 0.1
        if(output < -0.6):
            output = -0.6
        if(output > -0.1 and output < 0.0):
            output = -0.1          
        print("Input",Input,"setPoint",setPoint,"error",error,"output",output)
        return output*-1.0
    def computeLinear(self,InputY,setPointY):
        error = InputY - setPointY                                         
        output = self.linearKp * error
        if(output > 0.5):
            output = 0.5
        if(output < 0.2 and output > 0.0):
            output = 0.2
        if(output < -0.5):
            output = -0.5
        if(output > -0.2 and output < 0.0):
            output = -0.2  
        if(error==0.0):
            output = 0.0
        print("Input",InputY,"setPoint",setPointY,"error",error,"output",output)  
        return output    
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
        

        # Subscribe to ultrasonic sensor data for distance measurements
        self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
        # Add another one here
        self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)

        # Create a ROS2 service for controlling docking behavior, can add another custom service message
        self.dock_control_srv = self.create_service(DockSw, 'dock_control', self.dock_control_callback, callback_group=self.callback_group)
        self.isDocked = self.create_publisher(Bool, '/dockingSuccesfull', 10)
        self.speedPub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.link_attach_cli = self.create_client(AttachLink, '/ATTACH_LINK')
        self.lind_detached_cli = self.create_client(DetachLink, '/DETACH_LINK')
        while not self.link_attach_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Link attacher service not available, waiting again...')

        # Initialize all  flags and parameters here
        self.is_docking = False
        self.dock_aligned=False
        self.usrleft_value=0
        self.usrright_value=0
        self.turn=1
        self.speed=0.5
        self.targetX=0
        self.targetY=0
        self.targetYaw=0
        self.rackName = ""
        self.isAttach = False
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
    # Callback function for the left ultrasonic sensor
    def ultrasonic_rl_callback(self, msg):
        self.usrleft_value = msg.range

    def ultrasonic_rr_callback(self, msg):
        self.usrright_value = msg.range
    # Callback function for the right ultrasonic sensor
    #
    #
    def attachRack(self,rackName):
            req = AttachLink.Request()
            req.model1_name =  'ebot'     
            req.link1_name  = 'ebot_base_link'       
            req.model2_name =  rackName       
            req.link2_name  = 'link' 
            self.link_attach_cli.call_async(req)   
    def detachRack(self,rackName):
            req = DetachLink.Request()
            req.model1_name =  'ebot'     
            req.link1_name  = 'ebot_base_link'       
            req.model2_name =  rackName       
            req.link2_name  = 'link' 
            self.lind_detached_cli.call_async(req)
    def is_at_goal(self,current_x, current_y, goal_x, goal_y, tolerance=0.04):
        
        distance = ((current_x - goal_x)**2 + (current_y - goal_y)**2)**0.5
        return distance <= tolerance
    def getRemaningDistance(self, current_x, current_y, goal_x, goal_y):
        distance = ((current_x - goal_x)**2 + (current_y - goal_y)**2)**0.5
        return distance
    # Utility function to normalize angles within the range of -π to π (OPTIONAL)
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
    
    def tracjetory(self,goal_x,goal_y,odom_x,odom_y,odom_yaw,goal_yaw):
        dx = goal_x - odom_x
        dy = goal_y - odom_y
        dist = math.sqrt(dx * dx + dy * dy)

        dx_odom = math.cos(odom_yaw) * dx + math.sin(odom_yaw) * dy
        dy_odom = -math.sin(odom_yaw) * dx + math.cos(odom_yaw) * dy

        vel_x = 0.12 * dx_odom
        if(vel_x > 0.35):
            vel_x = 0.35
        if(vel_x > 0.0  and vel_x < 0.26):
            vel_x = 0.26
        if(vel_x < 0.0  and vel_x > -0.26):
            vel_x = -0.26
        if(vel_x < -0.35):
            vel_x = -0.35         
        

        dyaw = math.atan2(dy_odom, dx_odom)

        if (dist < 0.1 ): 
            
            dyaw = goal_yaw - odom_yaw
            if (dyaw > 3.14):
                dyaw -= 2 * 3.14
            elif (dyaw < -3.14):
                dyaw += 2 * 3.14
            

        vel_yaw = 0.07 * dyaw
        return vel_x,vel_yaw*-1
    # Main control loop for managing docking behavior
    def is_bot_at_goal_position(self,bot_x, bot_y, goal_x, goal_y, tolerance=0.08):
        """Checks if the bot is at the goal position.

        Args:
            bot_x: The bot's current X position.
            bot_y: The bot's current Y position.
            goal_x: The goal X position.
            goal_y: The goal Y position.
            tolerance: The tolerance value.

        Returns:
            A Boolean value indicating whether the bot is at the goal position.
        """

        x_diff = abs(bot_x - goal_x)
        y_diff = abs(bot_y - goal_y)
        print("x_diff",x_diff,"y_diff",y_diff)
        return x_diff <= tolerance and y_diff <= tolerance
    def controller_loop(self):

        # The controller loop manages the robot's linear and angular motion 
        # control to achieve docking alignment and execution
        
        # print("controller loop")
        def odometry_callback(msg):
        # Extract and update robot pose information from odometry message
            global robot_pose
            robot_pose[0] = msg.pose.pose.position.x
            robot_pose[1] = msg.pose.pose.position.y
            quaternion_array = msg.pose.pose.orientation
            orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
            _, _, yaw = euler_from_quaternion(orientation_list)
            yaw = math.degrees(yaw)
            robot_pose[2] = yaw
            robot_pose[3] = msg.pose.pose.position.z

        if self.is_docking:
            # ...
            # Implement control logic here for linear and angular motion
            # For example P-controller is enough, what is P-controller go check it out !
            # ...
            botPid = pid()
            yaw = False
            global robot_pose
            dockingNode = Node("GlobalNodeDocking")
            callback_group = ReentrantCallbackGroup()
            executor = MultiThreadedExecutor(2)
            executor.add_node(dockingNode)
            executor_thread = Thread(target=executor.spin, daemon=True, args=())
            executor_thread.start()
            dockingNode.odom_sub = dockingNode.create_subscription(Odometry, '/odom', odometry_callback, 10)
            dockingNode.odom_sub
            time.sleep(0.5)
<<<<<<< HEAD:ebot_docking/scripts/ebot_docking_service_task2b.py
            if self.targetX != self.targetY:
                yaw = False
                while (yaw == False):
                    angle=botPid.computeAngle(int(self.normalize_angle(self.targetYaw)),int(self.normalize_angle(robot_pose[2])),robot_pose[0],robot_pose[1])
                    self.moveBot(0.0,angle)
                    yaw = True if(int(self.normalize_angle(self.targetYaw)) == int(self.normalize_angle(robot_pose[2]))) else False
                    
                    time.sleep(0.01)
                for i in range(5):
                    
                    self.moveBot(0.0,0.0)
                #orientation done
                
                
                reached = False
                
                while (reached == False):
                    # linearSpeed = botPid.computeLinear(round(robot_pose[1],2),round(self.targetY,2))
                    reached = self.is_bot_at_goal_position(round(robot_pose[0],2),round(robot_pose[1],2),round(self.targetX,2),round(self.targetY,2))
                    X,Yaw = self.tracjetory(self.targetX,self.targetY,robot_pose[0],robot_pose[1],math.radians(robot_pose[2]),math.radians(self.targetYaw))
                    print(X,Yaw)
                    self.moveBot(X,Yaw)
                yaw = False
                while (yaw == False):
                    angle=botPid.computeAngle(int(self.normalize_angle(self.targetYaw)),int(self.normalize_angle(robot_pose[2])),robot_pose[0],robot_pose[1])
                    self.moveBot(0.0,angle)
                    yaw = True if(int(self.normalize_angle(self.targetYaw)) == int(self.normalize_angle(robot_pose[2]))) else False
                    
                    time.sleep(0.01)
                for i in range(5):
                    
                    self.moveBot(0.0,0.0)
                if self.isAttach:
                    self.attachRack(self.rackName)
                else :
                    self.detachRack(self.rackName)
=======
        
            yaw = False
            while (yaw == False):
                angle=botPid.computeAngle(int(self.normalize_angle(self.targetYaw)),int(self.normalize_angle(robot_pose[2])),robot_pose[0],robot_pose[1])
                self.moveBot(0.0,angle)
                # yaw = True if(int(self.normalize_angle(self.targetYaw)) == int(self.normalize_angle(robot_pose[2]))) else False
                time.sleep(0.01)
>>>>>>> 407fdf3 (yaw done):ebot_docking/scripts/ebot_docking_boilerplate.py
            for i in range(5):
                
                self.moveBot(0.0,0.0)
            #orientation done
            
            
            reached = False
            
            while (reached == False):
                # linearSpeed = botPid.computeLinear(round(robot_pose[1],2),round(self.targetY,2))
                reached = self.is_bot_at_goal_position(round(robot_pose[0],2),round(robot_pose[1],2),round(self.targetX,2),round(self.targetY,2))
                X,Yaw = self.tracjetory(self.targetX,self.targetY,robot_pose[0],robot_pose[1],math.radians(robot_pose[2]),math.radians(self.targetYaw))
                print(X,Yaw)
                # self.moveBot(X,Yaw)
            yaw = False
            while (yaw == False):
                angle=botPid.computeAngle(int(self.normalize_angle(self.targetYaw)),int(self.normalize_angle(robot_pose[2])),robot_pose[0],robot_pose[1])
                self.moveBot(0.0,angle)
                yaw = True if(int(self.normalize_angle(self.targetYaw)) == int(self.normalize_angle(robot_pose[2]))) else False
                
                time.sleep(0.01)
            for i in range(5):
                
                self.moveBot(0.0,0.0)
            # if self.isAttach:
            #     self.attachRack(self.rackName)
            # else :
            #     self.detachRack(self.rackName)
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
        # Reset flags and start the docking process
        #
        #
        for i in range(10):
            msg = Bool()
            msg.data = False
            self.isDocked.publish(msg)
        for i in range(5):
            self.moveBot(0.0,0.0)
        
        self.is_docking = True
        self.controller_loop()
        
        # Log a message indicating that docking has started
        self.get_logger().info("Docking started!")

        # Create a rate object to control the loop frequency
        rate = self.create_rate(2, self.get_clock())
        print(request)
        # Wait until the robot is aligned for docking
        while not self.dock_aligned:
            # self.get_logger().info("Waiting for alignment...")
            
            rate.sleep()

        # Set the service response indicating success
        response.success = True
        for i in range(10):
            msg = Bool()
            msg.data = True
            self.isDocked.publish(msg)
        
        response.message = "Docking control initiated"
        return response

# Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    

    my_robot_docking_controller = MyRobotDockingController()

    executor = MultiThreadedExecutor(2)
    executor.add_node(my_robot_docking_controller)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    rclpy.spin(my_robot_docking_controller)
    my_robot_docking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
