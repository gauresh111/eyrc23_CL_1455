#!/usr/bin/env python3

'''
*****************************************************************************************
*  Filename:			duplicate_imu.py
*  Created:				19/10/2023 (For eYRC Cosmo Logistic)
*  Last Modified:	    19/10/2023
*  Modified by:         Archit
*  Author:				Archit e-Yantra Team
*****************************************************************************************
'''

import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class CopyImu(Node):

    def __init__(self):
        super().__init__('copy_imu')

        # self.copy_imu_sub = self.create_subscription(Imu, 'sensors/imu', self.imu_cb, 10)
        self.copy_imu_sub_orient = self.create_subscription(Float32, 'orientation', self.orient_cb, 10)
        self.new_imu_pub = self.create_publisher(Imu, 'sensors/imu1', 10)

    # def imu_cb(self, msg):
    #     ori = msg.orientation
    #     euler_angles = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
    #     print(f'IMU = {euler_angles[2]}')
    #     # self.get_logger().info(f'Sucessfully copied imu!')

    def orient_cb(self, msg):
        yaw = msg.data
        if yaw > 3.14:
            yaw_new = (6.28 - yaw) * 1
        else:
            yaw_new = yaw * -1
        print(yaw)
        quat = quaternion_from_euler(0.0, 0.0, yaw_new)
        new_imu = Imu()
        new_imu.orientation.x = quat[0]
        new_imu.orientation.y = quat[1]
        new_imu.orientation.z = quat[2]
        new_imu.orientation.w = quat[3]
        new_imu.header.frame_id = 'ebot_base'
        new_imu.header.stamp = self.get_clock().now().to_msg()
        self.new_imu_pub.publish(new_imu)
        self.get_logger().info(f'Sucessfully copied imu!')

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CopyImu()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("###### Keyboard interrupt detected. Closing script. ######")


if __name__ == '__main__':
    main()