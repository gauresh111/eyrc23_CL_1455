#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 1.0
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # Go to our demos first goal pose
    goal_pose_1 = PoseStamped()
    goal_pose_1.header.frame_id = 'map'
    goal_pose_1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose_1.pose.position.x = 1.8
    goal_pose_1.pose.position.y = 1.5
    quats_xyzw = tf3d.euler.euler2quat(0, 0, 1.57)
    goal_pose_1.pose.orientation.x = quats_xyzw[3]
    goal_pose_1.pose.orientation.y = quats_xyzw[0]
    goal_pose_1.pose.orientation.z = quats_xyzw[1]
    goal_pose_1.pose.orientation.w = quats_xyzw[2]

    goal_pose_2 = PoseStamped()
    goal_pose_2.header.frame_id = 'map'
    goal_pose_2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose_2.pose.position.x = 0.0
    goal_pose_2.pose.position.y = 0.0
    goal_pose_2.pose.orientation.w = 1.0

    goal_pose_3 = PoseStamped()
    goal_pose_3.header.frame_id = 'map'
    goal_pose_3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose_3.pose.position.x = 2.0
    goal_pose_3.pose.position.y = 0.5
    goal_pose_3.pose.orientation.w = 1.0

    goal_pose_4 = PoseStamped()
    goal_pose_4.header.frame_id = 'map'
    goal_pose_4.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose_4.pose.position.x = 0.0
    goal_pose_4.pose.position.y = 0.0
    goal_pose_4.pose.orientation.w = 1.0

    goal_pose_5 = PoseStamped()
    goal_pose_5.header.frame_id = 'map'
    goal_pose_5.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose_5.pose.position.x = -2.0
    goal_pose_5.pose.position.y = -0.5
    goal_pose_5.pose.orientation.w = 1.0


    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    navigator.goToPose(goal_pose_1)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        # i = i + 1
        # feedback = navigator.getFeedback()
        # if feedback and i % 5 == 0:
        #     print('Estimated time of arrival: ' + '{0:.0f}'.format(
        #           Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
        #           + ' seconds.')

            # # Some navigation timeout to demo cancellation
            # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
            #     navigator.cancelTask()

            # # Some navigation request change to demo preemption
            # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
            #     goal_pose.pose.position.x = -3.0
            #     navigator.goToPose(goal_pose)
        i += 1
        feedback = navigator.getFeedback()
        if i % 5 == 0:
            print(feedback)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()