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



def main():
    rclpy.init()
    navigator = BasicNavigator()
    node = Node("moveBot")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()
    initalPose = PoseStamped()
    initalPose.header.frame_id = 'map'
    initalPose.header.stamp = navigator.get_clock().now().to_msg()
    initalPose.pose.position.x = 0.0
    initalPose.pose.position.y = 0.0
    initalPose.pose.position.z = 0.0
    initalPose.pose.orientation.x = 0.0
    initalPose.pose.orientation.y = 0.0
    initalPose.pose.orientation.z = 0.0
    initalPose.pose.orientation.w = 1.0
    
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
        
    node.subscription = node.create_subscription(
        Odometry, "/odom", poseUpdate, 10
    )
    node.subscription
    print("her")
        
    
    goalPose1 = PoseStamped()
    goalPose1.header.frame_id = 'map'
    goalPose1.header.stamp = navigator.get_clock().now().to_msg()
    goalPose1.pose.position.x = 1.8
    goalPose1.pose.position.y = 1.5
    goalPose1.pose.position.z = 0.0
    goalPose1.pose.orientation.x = 0.0
    goalPose1.pose.orientation.y = 0.0
    goalPose1.pose.orientation.z = 0.7068252
    goalPose1.pose.orientation.w = 0.7073883
    
    
    goalPose2 = PoseStamped()
    goalPose2.header.frame_id = 'map'
    goalPose2.header.stamp = navigator.get_clock().now().to_msg()
    goalPose2.pose.position.x = 2.0
    goalPose2.pose.position.y = -7.0
    goalPose2.pose.position.z = 0.0
    goalPose2.pose.orientation.x = 0.0
    goalPose2.pose.orientation.y = 0.0
    goalPose2.pose.orientation.z = -0.7068252
    goalPose2.pose.orientation.w = 0.7073883
    
    goalPose3 = PoseStamped()
    goalPose3.header.frame_id = 'map'
    goalPose3.header.stamp = navigator.get_clock().now().to_msg()
    goalPose3.pose.position.x = -3.0
    goalPose3.pose.position.y = 2.5
    goalPose3.pose.position.z = 0.0
    goalPose3.pose.orientation.x = 0.0
    goalPose3.pose.orientation.z = 0.7068252
    goalPose3.pose.orientation.y = 0.0
    goalPose3.pose.orientation.w = 0.7073883
    
    goalPose4 = PoseStamped()
    goalPose4.header.frame_id = 'map'
    goalPose4.header.stamp = navigator.get_clock().now().to_msg()
    goalPose4.pose.position.x = -1.0
    goalPose4.pose.position.y = -5.0
    goalPose4.pose.position.z = 0.0
    goalPose4.pose.orientation.x = 0.0
    goalPose4.pose.orientation.z = 0.7068252
    goalPose4.pose.orientation.y = 0.0
    goalPose4.pose.orientation.w = 0.7073883
    
    goalPose5 = PoseStamped()
    goalPose5.header.frame_id = 'map'
    goalPose5.header.stamp = navigator.get_clock().now().to_msg()
    goalPose5.pose.position.x = -1.0
    goalPose5.pose.position.y = -8.0
    goalPose5.pose.position.z = 0.0
    goalPose5.pose.orientation.x = 0.0
    goalPose5.pose.orientation.z = 0.7068252
    goalPose5.pose.orientation.y = 0.0
    goalPose5.pose.orientation.w = 0.7073883
    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goalPose1)
    navigator.setInitialPose(initalPose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()
    # Go to the goal pose
    def moveToGoal(goalPose):
        global botPosition, botOrientation
        navigator.goToPose(goalPose)

        i = 0

        # Keep doing stuff as long as the robot is moving towards the goal
        while not navigator.isTaskComplete():
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Distance remaining: ' + '{:.2f}'.format(
                    feedback.distance_remaining) + ' meters.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=60.0):
                    navigator.clearAllCostmaps()

        # Do something depending on the return code
        result = navigator.getResult()
        print(result)
        time.sleep(1)
        angles = R.from_quat(botOrientation)
        angles = list(angles.as_euler("xyz", degrees=True))
        print("BOt POSITION ",botPosition[0],botPosition[1])
        print("BOT ORIENTATION",angles[2])
        allowed_pose_tolerance = 0.3
        allowed_orientation_tolerance = 10
        target = [ goalPose.pose.position.x,goalPose.pose.position.y]
        print(target)
        if (abs(abs(botPosition[0]) - abs(target[0])) <= allowed_pose_tolerance and
                abs(abs(botPosition[1]) - abs(target[1])) <= allowed_pose_tolerance and
                abs(abs(angles[2]) - 90) <= allowed_orientation_tolerance):
                   
            print("ACCEPTED")
        else:
            print("NOT ACCEPTED")

    moveToGoal(goalPose1)
    # moveToGoal(goalPose2)
    # moveToGoal(goalPose3)
    # # moveToGoal(goalPose4)
    # # moveToGoal(goalPose3)
    # moveToGoal(goalPose5)
    
    # Shut down the ROS 2 Navigation Stack
    navigator.lifecycleShutdown()
    
    
    # print("done")
    # print(botPosition)
    # print(botOrientation)
    rclpy.spin(node)
    rclpy.shutdown()
    exit(0)


if __name__ == '__main__':
    main()
