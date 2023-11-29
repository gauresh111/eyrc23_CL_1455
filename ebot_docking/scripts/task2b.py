#!/usr/bin/env python3

from nav_msgs.msg import Odometry
import rclpy
from threading import Thread
import time
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
import tf2_ros
from rclpy.duration import Duration # Handles time for ROS 2
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool
from ebot_docking.srv import DockSw  # Import custom service message
from tf_transformations import euler_from_quaternion
import math
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import Polygon,Point32
from ebot_docking.srv import RackSw
def main():
    rclpy.init()
    navigator = BasicNavigator()
    node = Node("moveBot")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()
    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    tf_buffer = tf2_ros.buffer.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)
    global botPosition, botOrientation
    botPosition = []
    botOrientation =[] 
    global positionToGO
    positionToGO = {
        'initalPose':{'xyz': [0.0, 0.0, 0.0], 'quaternions': [0.0, 0.0, 0.0, 1.0], 'XYoffsets': [0.0, 0.0]},
        'rack1':{'xyz': [0.0, 4.4, 0.0], 'quaternions': [0.0, 0.0, 1.0, 0.0], 'XYoffsets': [1.26, 0.0]}, 
        'rack2':{'xyz': [2.03, 2.06, 0.0], 'quaternions': [0.0, 0.0, -0.7068252, 0.7073883], 'XYoffsets': [0.0, 1.24]},
        'rack3':{'xyz': [2.13, -7.09, 0.0], 'quaternions': [0.0, 0.0, 0.7068252, 0.7073883], 'XYoffsets': [0.0, -1.0]}, 
        'ap1':{'xyz': [0.0, -2.45, 0.0], 'quaternions': [0.0, 0.0, 1.0, 0.0], 'XYoffsets': [0.7, 0.0]}, 
        'ap2':{'xyz': [1.37, -4.15, 0.0], 'quaternions': [0.0, 0.0, -0.7068252, 0.7073883], 'XYoffsets': [0.0, 0.8]}, 
        'ap3':{'xyz': [1.37, -1.04, 0.0], 'quaternions': [0.0, 0.0, 0.7068252, 0.7073883], 'XYoffsets': [0.0, -0.72]}
            }
    withRackFootprint = [ [0.31, 0.40],[0.31, -0.40],[-0.31, -0.40],[-0.31, 0.40] ]
    withoutRackFootprint = [ [0.21, 0.195],[0.21, -0.195],[-0.21, -0.195],[-0.21, 0.195] ]
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
    global isDock
    isDock = False
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
    def poseUpdate(data):
        # print("current pose:", data.pose.pose.position.x)
        global botPosition, botOrientation
        botPosition=[
            data.pose.pose.position.x,
            data.pose.pose.position.y,
            data.pose.pose.position.z
        ]  # type: ignore
        botOrientation = [
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        ]
    def isDocking(data):
        global isDock 
        isDock = data.data
        # print(isDock)
        print(isDock,type(isDock),"callback")
    node.subscription = node.create_subscription(
        Odometry, "/odom", poseUpdate, 10
    )
    node.subscription
    node.isDockingSub = node.create_subscription(
        Bool, "/dockingSuccesfull", isDocking, 10
    )
    node.isDockingSub
    node.publisher = node.create_publisher(PoseStamped, '/dockingRequest', 10)
    node.dockingClient = node.create_client(DockSw, '/dock_control')
    while not node.dockingClient.wait_for_service(timeout_sec=1.0):
        print('service not available, waiting again...')
    node.dockingRequest = DockSw.Request()
    
    print("her")
    navigator.setInitialPose(getGoalPoseStamped("initalPose"))

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()
    # Go to the goal pose
    
    def moveToGoal(goalPose,rack_no,israck,positionName):
        global botPosition, botOrientation
        global positionToGO
        
        
        # goalPose.pose.position.z=0.0
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
        print(result)
        quaternion_array = goalPose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        yaw = math.degrees(yaw)
        Xoff = positionToGO[positionName]['XYoffsets'][0]
        Yoff = positionToGO[positionName]['XYoffsets'][1]
        node.dockingRequest.linear_dock = True
        node.dockingRequest.orientation_dock = True
        node.dockingRequest.goal_x = round(goalPose.pose.position.x+Xoff,2)
        node.dockingRequest.goal_y = round(goalPose.pose.position.y+Yoff,2)
        node.dockingRequest.orientation = round(yaw,2)
        node.dockingRequest.rack_no = rack_no
        node.dockingRequest.rack_attach=israck
        future = node.dockingClient.call_async(node.dockingRequest)
        print(node.dockingRequest)
        time.sleep(0.5)
        print(future.result())
        while(future.result() is  None):
            try:
                print(future)
            except:
                pass
        # while isDock!=True:
        #     print("waiting")
                    # node.publisher.publish(goalPose)

    def Rack_control_callback(Request,Response):
        node.get_logger().info("Request Arrived")
        RackRequest=Request.rack_name
        ApRequest=Request.ap_name
        
        #goes to rack
        moveToGoal(getGoalPoseStamped(RackRequest),RackRequest,True,RackRequest)
        node.get_logger().info("Going to Rack")
        #goes to ap    
        moveToGoal(getGoalPoseStamped(ApRequest),RackRequest,False,ApRequest)
        node.get_logger().info("Going to Ap")
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
