#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from threading import Thread
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from scipy.spatial.transform import Rotation as R
from ebot_docking.srv import DockSw  # Import custom service message
from tf_transformations import euler_from_quaternion
import math
from geometry_msgs.msg import Polygon,Point32
from std_msgs.msg import Bool,Float32MultiArray
from ebot_docking.srv import RackSw
from rclpy.callback_groups import ReentrantCallbackGroup

global ultrasonic_value

import time
def main():
    rclpy.init()
    navigator = BasicNavigator()
    callback_group = ReentrantCallbackGroup()
    node = Node("moveBot")
    executor = rclpy.executors.MultiThreadedExecutor(1)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    
    global positionToGO
    positionToGO = {
      'ap1':{'xyz': [4.63, -0.1,0.0], 'quaternions': [0.0, 0.0, 0.1, 0.0000], 'XYoffsets': [1.0,0.0], 'Yaw': 0.0},
      'ap2':{'xyz': [6.2, -1.9,0.0], 'quaternions': [0.0, 0.0, -0.706825181105366, 0.7073882691671998], 'XYoffsets': [0.0,1.1] , 'Yaw': 90}
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
        if not israck:
            change_footprint(withRackFootprint,"withRackFootprint")
        else:
            change_footprint(withoutRackFootprint,"withoutRackFootprint")
        
        
        navigator.goToPose(goalPose)
        i = 0
        UltraSoniceForNAv2 = Node("UltraSoniceForNAv2")
        UltraSoniceForNAv2_executor = rclpy.executors.MultiThreadedExecutor(1)
        UltraSoniceForNAv2_executor.add_node(UltraSoniceForNAv2)
        executor_thread = Thread(target=UltraSoniceForNAv2_executor.spin, daemon=True, args=())
        executor_thread.start()
        # Keep doing stuff as long as the robot is moving towards the goal
        global ultrasonic_value
        ultrasonic_value = [0.0,0.0]
        def ultrasonic_callback(msg):
            global ultrasonic_value
            ultrasonic_value[0] = round(msg.data[4],4)
            ultrasonic_value[1] = round(msg.data[5],4)
        UltraSoniceForNAv2.ultra_sub = UltraSoniceForNAv2.create_subscription(Float32MultiArray, 'ultrasonic_sensor_std_float', ultrasonic_callback, 10)  
    
        while not navigator.isTaskComplete():
            if positionName == "Arm_pose":
                print("moving to ",positionName," ",ultrasonic_value)
                if (ultrasonic_value[0]+ultrasonic_value[1])/2 >=16.0:
                    navigator.cancelTask()
                    
                    UltraSoniceForNAv2.dockingClient = UltraSoniceForNAv2.create_client(DockSw, '/dock_control')
                    UltraSoniceForNAv2.dockingRequest = DockSw.Request()
                    UltraSoniceForNAv2.dockingRequest.rack_no = rack_no
                    UltraSoniceForNAv2.dockingRequest.is_rack_detached = True
                    future = UltraSoniceForNAv2.dockingClient.call_async(UltraSoniceForNAv2.dockingRequest)
                    rclpy.spin_until_future_complete(UltraSoniceForNAv2, future)
                    navigator.goToPose(goalPose)
                rclpy.spin_once(UltraSoniceForNAv2, timeout_sec=0.1)

        
        UltraSoniceForNAv2.destroy_node()
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
        dockingNodecli.dockingRequest.is_rack_detached = False
        future = dockingNodecli.dockingClient.call_async(dockingNodecli.dockingRequest)
        rclpy.spin_until_future_complete(dockingNodecli, future)
        dockingNodecli.destroy_node()
        navigator.clearAllCostmaps()
         
         
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
        moveToGoal(getGoalPoseStamped(RackRequest),RackRequest,True,RackRequest)
        
        #goes to ap   
        node.get_logger().info("Going to Ap")
        moveToGoal(getGoalPoseStamped(ApRequest),RackRequest,False,ApRequest)
        
        Response.success = True
        Response.message = "Success"
        node.get_logger().info("Request done with Succes")
        return Response
    def ExitCallBack(msg):
        if msg.data:
            raise SystemExit
    dock_control_srv = node.create_service(RackSw, '/RackNav2Sw', Rack_control_callback, callback_group=callback_group)
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
