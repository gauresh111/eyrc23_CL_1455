#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from threading import Thread
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from scipy.spatial.transform import Rotation as R
from mani_stack.srv import DockSw  # Import custom service message
from tf_transformations import euler_from_quaternion
import math
from std_msgs.msg import Bool
def main():
    rclpy.init()
    navigator = BasicNavigator()
    node = Node("moveBot")
    executor = rclpy.executors.MultiThreadedExecutor(1)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    
    global positionToGO
    positionToGO = {
      'Pre_docking_pose':{'xyz' : [1.05, 2.04,0.0] , 'quaternions': [0.0, 0.0, 0.0, 1.0], 'XYoffsets': [0.0, 0.0] , 'XYoffsets': [0.0, 0.0], 'Yaw': 1.57}, 
      'Arm_pose':{'xyz': [5.0, 0.0,0.0], 'quaternions': [0.0, 0.0, 0.9999996829318346, 0.0007963267107332633], 'XYoffsets': [0.0, 0.0] , 'XYoffsets': [0.0, 0.0], 'Yaw': 1.57}
    }
    def getGoalPoseStamped(goal):
        global positionToGO
        Goal = positionToGO[goal]
        goalPose = PoseStamped()
        goalPose.header.frame_id = 'map'
        goalPose.header.stamp = navigator.get_clock().now().to_msg()
        goalPose.pose.position.x = Goal['xyz'][0]
        goalPose.pose.position.y = Goal['xyz'][1]
        goalPose.pose.position.z = Goal['xyz'][2]
        goalPose.pose.orientation.x = Goal['quaternions'][0]
        goalPose.pose.orientation.y = Goal['quaternions'][1]
        goalPose.pose.orientation.z = Goal['quaternions'][2]
        goalPose.pose.orientation.w = Goal['quaternions'][3]
        print(goalPose)
        return goalPose  
    def moveToGoal(goalPose,rack_no,israck,positionName):
        global positionToGO
        dockingNodecli = rclpy.create_node("NodeDockingClient")
        dockingNodecli.dockingClient = dockingNodecli.create_client(DockSw, '/dock_control')
        while not dockingNodecli.dockingClient.wait_for_service(timeout_sec=1.0):
            print('docking Client service not available, waiting again...')
        dockingNodecli.dockingRequest = DockSw.Request()    
        navigator.goToPose(goalPose)
        i = 0

        # Keep doing stuff as long as the robot is moving towards the goal
        while not navigator.isTaskComplete():
            i = i + 1
        result = navigator.getResult()
        quaternion_array = goalPose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        yaw = math.degrees(yaw)
        Xoff = positionToGO[positionName]['XYoffsets'][0]
        Yoff = positionToGO[positionName]['XYoffsets'][1]
        dockingNodecli.dockingRequest.linear_dock = True
        dockingNodecli.dockingRequest.orientation_dock = True
        dockingNodecli.dockingRequest.goal_x = round(goalPose.pose.position.x+Xoff,2)
        dockingNodecli.dockingRequest.goal_y = round(goalPose.pose.position.y+Yoff,2)
        dockingNodecli.dockingRequest.orientation = round(yaw,2)
        dockingNodecli.dockingRequest.rack_no = rack_no
        dockingNodecli.dockingRequest.rack_attach=israck
        future = dockingNodecli.dockingClient.call_async(dockingNodecli.dockingRequest)
        rclpy.spin_until_future_complete(dockingNodecli, future)
        dockingNodecli.destroy_node()
        
        navigator.clearAllCostmaps()
         
    navigator.waitUntilNav2Active()
    
    moveToGoal(getGoalPoseStamped("Pre_docking_pose"),"Pre_docking_pose",True,"Pre_docking_pose")
        
        #goes to ap   
    node.get_logger().info("Going to Ap")
    moveToGoal(getGoalPoseStamped("Arm_pose"),"Arm_pose",False,"Arm_pose")
        
   
    def ExitCallBack(msg):
        if msg.data:
            raise SystemExit
    exitNav2 = node.create_subscription(Bool, '/ExitNav',ExitCallBack, 10)
    try:
        rclpy.spin(node)
    except SystemExit:
        print("SystemExit")
        node.destroy_node()
        navigator.lifecycleShutdown()
        rclpy.shutdown()
        exit(0)
if __name__ == '__main__':
    main()
