#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import rclpy
from rclpy.node import Node
from threading import Thread
import time
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_simple_commander.robot_navigator import BasicNavigator
from scipy.spatial.transform import Rotation as R
from ebot_docking.srv import DockSw  # Import custom service message
from tf_transformations import euler_from_quaternion
import math
from geometry_msgs.msg import Polygon,Point32
from ebot_docking.srv import RackSw
from std_msgs.msg import Bool
def main():
    rclpy.init()
    navigator = BasicNavigator()
    node = Node("moveBot")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()
    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(1)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    
    global botPosition, botOrientation
    botPosition = []
    botOrientation =[] 
    global positionToGO
    positionToGO = {
      'initalPose':{'xyz': [0.0, 0.0, 0.0], 'quaternions': [0.0, 0.0, 0.0, 1.0], 'XYoffsets': [0.0, 0.0],'Yaw':180},
      'ap1': {'xyz': [-0.2, -2.45, 0.0], 'quaternions': [0.0, 0.0, 0.9999996829318346, 0.0007963267107332633], 'XYoffsets': [1.0, 0.0], 'Yaw': 3.14},
      'ap2': {'xyz': [1.45,-4.38, 0.0], 'quaternions': [0.0, 0.0, -0.706825181105366, 0.7073882691671998], 'XYoffsets': [0.0, 1.0], 'Yaw': -1.57}, 
      'ap3': {'xyz': [1.45,-0.55, 0.0], 'quaternions': [0.0, 0.0, 0.706825181105366, 0.7073882691671998], 'XYoffsets': [0.0, -1.0], 'Yaw': 1.57}
     }
    withRackFootprint = [ [0.31, 0.40],[0.31, -0.40],[-0.31, -0.40],[-0.31, 0.40] ]
    withoutRackFootprint = [ [0.21, 0.195],[0.21, -0.195],[-0.21, -0.195],[-0.21, 0.195] ]
    def add_docking_position(name, xyz, quaternions, xy_offsets,yaw):
        global positionToGO
        positionToGO[name] = {
            'xyz': xyz,
            'quaternions': quaternions,
            'XYoffsets': xy_offsets,
            'Yaw':yaw
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
    def change_footprint(new_footprint,msg):
        # Initialize ROS node
        nodeFootprint = rclpy.create_node('change_footprint_node')
        # Create a service client to set parameters
        nodeFootprint.localFootPrintPub=nodeFootprint.create_publisher(Polygon, '/local_costmap/footprint', 10)
        nodeFootprint.globalFootPrintPub=nodeFootprint.create_publisher(Polygon, '/global_costmap/footprint', 10)
        p = Polygon()
        p.points = [Point32(x=new_footprint[0][0], y=new_footprint[0][1]),
                            Point32(x=new_footprint[1][0], y=new_footprint[1][1]),
                            Point32(x=new_footprint[2][0], y=new_footprint[2][1]),
                            Point32(x=new_footprint[3][0], y=new_footprint[3][1])]
        for i in range (3):
            nodeFootprint.globalFootPrintPub.publish(p)
            nodeFootprint.localFootPrintPub.publish(p)
            time.sleep(0.1)
            print("publishing:" ,msg)
            
        nodeFootprint.destroy_node()
    def moveToGoal(goalPose,rack_no,israck,positionName,init_pose):
        
        global botPosition, botOrientation
        global positionToGO
        dockingNodecli = rclpy.create_node("NodeDockingClient")
        dockingNodecli.dockingClient = dockingNodecli.create_client(DockSw, '/dock_control')
        while not dockingNodecli.dockingClient.wait_for_service(timeout_sec=1.0):
            print('docking Client service not available, waiting again...')
        dockingNodecli.dockingRequest = DockSw.Request()
        
        if not israck:
            if rack_no=="initalPose":
                change_footprint(withoutRackFootprint,"withoutRackFootprint")
            else:
                change_footprint(withRackFootprint,"withRackFootprint")
        else:
            change_footprint(withoutRackFootprint,"withoutRackFootprint")
        
            
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
        time.sleep(0.2)
    navigator.setInitialPose(getGoalPoseStamped("initalPose"))
    navigator.waitUntilNav2Active()
    def Rack_control_callback(Request,Response):
        global positionToGO
        node.get_logger().info("Request Arrived")
        RackRequest=Request.rack_name
        ApRequest=Request.ap_name
        x=Request.x
        y=Request.y
        yaw=Request.yaw
        x_offset=Request.offset_x
        y_offset=Request.offset_y
        xyz=[x,y,0.0]
        euler = [0,0,yaw]
        quaternions = R.from_euler('xyz', euler).as_quat().tolist()
        offsetXY=[x_offset,y_offset]
        add_docking_position(RackRequest,xyz,quaternions,offsetXY,yaw)
        #goes to rack
        node.get_logger().info("Going to Rack")
        moveToGoal(getGoalPoseStamped(RackRequest),RackRequest,True,RackRequest,getGoalPoseStamped(RackRequest))
        
        #goes to ap   
        node.get_logger().info("Going to Ap")
        moveToGoal(getGoalPoseStamped(ApRequest),RackRequest,False,ApRequest,getGoalPoseStamped(ApRequest))
        
        Response.success = True
        Response.message = "Success"
        node.get_logger().info("Request done with Succes")
        return Response
    dock_control_srv = node.create_service(RackSw, '/RackNav2Sw', Rack_control_callback, callback_group=callback_group)
    rclpy.spin(node)
    rclpy.shutdown()
    navigator.lifecycleShutdown()
    exit(0)
if __name__ == '__main__':
    main()
