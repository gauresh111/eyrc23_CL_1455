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


def main():
    rclpy.init()
    navigator = BasicNavigator()
    node = Node("moveBot")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()
    initalPose = PoseStamped()
    initalPose.header.frame_id = 'map'
    initalPose.header.stamp = navigator.get_clock().now().to_msg()
    initalPose.pose.position.x = 0.00
    initalPose.pose.position.y = 0.00
    initalPose.pose.position.z = 0.00
    initalPose.pose.orientation.x = 0.0
    initalPose.pose.orientation.y = 0.0
    initalPose.pose.orientation.z = 0.0
    initalPose.pose.orientation.w = 1.0
    # initalPose.pose.target = "initalPose"
# Create MoveIt 2 interface

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
    global XrackOFfset, YrackOffset
    XrackOFfset, YrackOffset = [0.0,1.08,0.0,0.0,0.3,0.0,0.0,0.0],[0.0,0.0,1.08,-0.81,0.0,0.0,0.0,0.0]
    global isDock
    isDock = False
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
    
    rack1 = PoseStamped()
    rack1.header.frame_id = 'map'
    rack1.header.stamp = navigator.get_clock().now().to_msg()
    rack1.pose.position.x = 0.00
    rack1.pose.position.y = 4.35
    rack1.pose.position.z = 1.0
    rack1.pose.orientation.x = 0.0
    rack1.pose.orientation.y = 0.0
    rack1.pose.orientation.z = 0.9999997
    rack1.pose.orientation.w = 0.0
    
    
    rack2 = PoseStamped()
    rack2.header.frame_id = 'map'
    rack2.header.stamp = navigator.get_clock().now().to_msg()
    rack2.pose.position.x = 2.03
    rack2.pose.position.y = 2.06
    rack2.pose.position.z = 2.0
    rack2.pose.orientation.x = 0.0
    rack2.pose.orientation.y = 0.0
    rack2.pose.orientation.z = -0.7068252
    rack2.pose.orientation.w = 0.7073883
    
    
    rack3 = PoseStamped()
    rack3.header.frame_id = 'map'
    rack3.header.stamp = navigator.get_clock().now().to_msg()
    rack3.pose.position.x = 2.03
    rack3.pose.position.y = -7.09
    rack3.pose.position.z = 3.0
    rack3.pose.orientation.x = 0.0
    rack3.pose.orientation.y = 0.0
    rack3.pose.orientation.z = 0.7068252
    rack3.pose.orientation.w = 0.7073883
    
    ap1 = PoseStamped()
    ap1.header.frame_id = 'map'
    ap1.header.stamp = navigator.get_clock().now().to_msg()
    ap1.pose.position.x = 0.00
    ap1.pose.position.y = -2.45
    ap1.pose.position.z = 4.0
    ap1.pose.orientation.x = 0.0
    ap1.pose.orientation.y = 0.0
    ap1.pose.orientation.z = 0.9999997
    ap1.pose.orientation.w = 0.0
    
    ap2 = PoseStamped()
    ap2.header.frame_id = 'map'
    ap2.header.stamp = navigator.get_clock().now().to_msg()
    ap2.pose.position.x = 2.03
    ap2.pose.position.y = -7.09
    ap2.pose.position.z = 3.0
    ap2.pose.orientation.x = 0.0
    ap2.pose.orientation.y = 0.0
    ap2.pose.orientation.z = 0.7068252
    ap2.pose.orientation.w = 0.7073883
    
    ap3 = PoseStamped()
    ap3.header.frame_id = 'map'
    ap3.header.stamp = navigator.get_clock().now().to_msg()
    ap3.pose.position.x = 2.03
    ap3.pose.position.y = -7.09
    ap3.pose.position.z = 3.0
    ap3.pose.orientation.x = 0.0
    ap3.pose.orientation.y = 0.0
    ap3.pose.orientation.z = 0.7068252
    ap3.pose.orientation.w = 0.7073883
    
    navigator.setInitialPose(initalPose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()
    # Go to the goal pose
    
    def moveToGoal(goalPose,rack_no,israck):
        global botPosition, botOrientation
        navigator.goToPose(goalPose)

        i = 0

        # Keep doing stuff as long as the robot is moving towards the goal
        while not navigator.isTaskComplete():
            i = i + 1
            feedback = navigator.getFeedback()
            # if feedback and i % 5 == 0:
            #     if round(goalPose.pose.position.x,1)==0 and round(botPosition[1],1) == round(goalPose.pose.position.y,1):
                    
            #         navigator.cancelTask()
        # Do something depending on the return code
        result = navigator.getResult()
        print(result)
        quaternion_array = goalPose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        yaw = math.degrees(yaw)
        goalPose.pose.position.x += XrackOFfset[int(goalPose.pose.position.z)]
        goalPose.pose.position.y += YrackOffset[int(goalPose.pose.position.z)]
        node.dockingRequest.linear_dock = True
        node.dockingRequest.orientation_dock = True
        node.dockingRequest.goal_x = goalPose.pose.position.x
        node.dockingRequest.goal_y = goalPose.pose.position.y
        node.dockingRequest.orientation = yaw
        node.dockingRequest.rack_no = rack_no
        node.dockingRequest.rack_attach=israck
        future = node.dockingClient.call_async(node.dockingRequest)
        time.sleep(0.5)
        while isDock!=True:
            print("waiting")
                    # node.publisher.publish(goalPose)
    moveToGoal(rack1,"rack1",True)
    moveToGoal(ap1,"rack1",False)
    moveToGoal(initalPose,"initalPose",False)
    moveToGoal(rack2,"rack2",True)
    moveToGoal(initalPose,"initalPose",False)
    moveToGoal(rack3,"rack3",True)
    # while True:
    #     print(isDock,type(isDock),"callback")
    
    # Shut down the ROS 2 Navigation Stack
    navigator.lifecycleShutdown()
    rclpy.spin(node)
    rclpy.shutdown()
    exit(0)


if __name__ == '__main__':
    main()
