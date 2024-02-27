#!/usr/bin/env python3

'''
# Team ID:          < 1455 >
# Theme:            < Cosmo Logistic >
# Author List:      < Joel Devasia , Gauresh Wadekar >
# Filename:         < docking_reset.py >
# Functions:        < "main" >
# Global variables: < none >
'''


import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from usb_relay.srv import RelaySw # type: ignore
from threading import Thread
import time

def main():
    rclpy.init()
    print("arduino reset")
    ResetNode = Node("Arduino_reset")        
    executor = MultiThreadedExecutor(1)
    executor.add_node(ResetNode)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    ResetNode.MagentCli = ResetNode.create_client(RelaySw, '/usb_relay_sw')
    req = RelaySw.Request()
    req.relaychannel = 1
    req.relaystate = True
    future = ResetNode.MagentCli.call_async(req)
    while(future.result() is  None):
        time.sleep(0.2)
    time.sleep(2.0)
    req = RelaySw.Request()
    req.relaychannel = 1
    req.relaystate = False
    ResetNode.MagentCli.call_async(req)
    print("arduino reset done")
    
if __name__ == '__main__':
    main()