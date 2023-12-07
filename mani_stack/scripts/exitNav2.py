#!/usr/bin/env python3
from threading import Thread
import time
import rclpy
from rclpy.node import Node
import subprocess
import signal
from std_msgs.msg import Bool
import os
import pyproc2
def main():
    rclpy.init()
    node = Node("exitNav2")
    executor = rclpy.executors.MultiThreadedExecutor(1)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    def Exitcallback(msg):
        if msg.data:
            print("Exitcallback",msg.data)
    node.Exit = node.create_subscription(Bool, '/ExitNav',Exitcallback, 30)
    eyantra_Wahrehouse_pid=pyproc2.find("task3a.launch.py").pid 
    print(eyantra_Wahrehouse_pid)
    rclpy.spin(node)
   
    node.destroy_node()
    rclpy.shutdown()
    exit(0)

if __name__ == '__main__':
    main()