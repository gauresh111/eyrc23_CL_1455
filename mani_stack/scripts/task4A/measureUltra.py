#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from threading import Thread
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool,Float32MultiArray
global ultrasonic_value
ultrasonic_value = [0.0,0.0]
def main():
    rclpy.init()
    dockNode = Node("Docking")        
    executor = MultiThreadedExecutor(1)
    executor.add_node(dockNode)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    dockNode.speedPub = dockNode.create_publisher(Twist, '/cmd_vel', 30)
    def moveBot(linearSpeedX,angularSpeed):
        twist = Twist()
        twist.linear.x = linearSpeedX
        twist.angular.z = angularSpeed
        dockNode.speedPub.publish(twist)
    def ultrasonic_callback(msg):
            global ultrasonic_value
            ultrasonic_value[0] = round(msg.data[4],4)
            ultrasonic_value[1] = round(msg.data[5],4)
    dockNode.ultra_sub = dockNode.create_subscription(Float32MultiArray, 'ultrasonic_sensor_std_float', ultrasonic_callback, 10)  
    time.sleep(2.0)
    while True:
        try:
            moveBot(-0.05,0.0)
            print("left: ",ultrasonic_value[0]," right: ",ultrasonic_value[1])
        except KeyboardInterrupt:
            moveBot(0.0,0.0)
            moveBot(0.0,0.0)
            print("left: ",ultrasonic_value[0]," right: ",ultrasonic_value[1])
            dockNode.destroy_node()
            rclpy.shutdown()
            exit(0)
            break
if __name__ == '__main__':
    main()