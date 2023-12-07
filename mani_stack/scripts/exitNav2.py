#!/usr/bin/env python3
from threading import Thread
import time
import rclpy
from rclpy.node import Node
import subprocess
import signal
from std_msgs.msg import Bool
def main():
    rclpy.init()
    node = Node("exitNav2")
    executor = rclpy.executors.MultiThreadedExecutor(1)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    childNav2launch = subprocess.Popen(["ros2", "launch", "mani_stack", "SpwanNav2.launch.py"])
    time.sleep(5)
    childtask3b = subprocess.Popen(["ros2", "launch", "mani_stack", "task3b.launch.py"])
    #child.wait() #You can use this line to block the parent process untill the child process finished.
    
    print("parent process")
    print(childNav2launch.poll())
    
    print(childtask3b.poll())
    print('The PID of child: ', childNav2launch.pid)
    print("The PID of child: ", childtask3b.pid)
    
    def Exitcallback(msg):
        if msg.data:
            print("Exitcallback",msg.data)
            raise SystemExit
    node.Exit = node.create_subscription(Bool, '/ExitNav',Exitcallback, 30)
    try:
        rclpy.spin(node)
    except SystemExit:
        print("SystemExit")
        childNav2launch.send_signal(signal.SIGINT)
        childtask3b.send_signal(signal.SIGINT)
    node.destroy_node()
    rclpy.shutdown()
    exit(0)

if __name__ == '__main__':
    main()