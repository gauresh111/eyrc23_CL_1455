#!/usr/bin/env python3
import re
from threading import Thread
import time

import psutil
import rclpy
from rclpy.node import Node
import subprocess
import signal
from std_msgs.msg import Bool
import os
from subprocess import check_output, CalledProcessError
def main():
    rclpy.init()
    node = Node("exitNav2")
    executor = rclpy.executors.MultiThreadedExecutor(1)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    def get_pid(name):
        result = check_output(["pgrep","-af",name]).decode('ASCII')
        pid_list=re.findall(r'\b\d+\b', result)
        return pid_list
    def kill_process():
        programName = ["nav2_cmd.py","ebot_nav2_yaml.py","ebot_docking_boilerplate.py","task3b.launch.py","SpwanNav2.launch.py","ekf_node","component_container_isolated","exitNav2.py"]    
        for i in range(len(programName)):
            program_name = programName[i]
            pid = get_pid(program_name)
            if program_name == "exitNav2.py":
               raise SystemExit
            elif pid:
                print("pid",pid)
                for j in range(len(pid)):
                    try:
                        
                        os.kill(int(pid[j]), signal.SIGTERM)
                        print("killed",pid[j],program_name)
                    except OSError as e:
                        print("No process running with name: ", program_name)
            else:
                print("No process running with name: ", program_name)
    def Exitcallback(msg):
        if msg.data:
            print("Exitcallback",msg.data)
            kill_process()
    node.Exit = node.create_subscription(Bool, '/ExitNav',Exitcallback, 30)
    try:
        rclpy.spin(node)
    except SystemExit:
        node.destroy_node()
        rclpy.shutdown()
        exit(0)

if __name__ == '__main__':
    main()