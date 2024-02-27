#!/usr/bin/env python3

'''
# Team ID:          < 1455 >
# Theme:            < Cosmo Logistic >
# Author List:      < Joel Devasia , Gauresh Wadekar >
# Filename:         < ultraFilter.py >
# Functions:        < ultrasonic_callback  , movingAverage , ultrasonic_publisher>
# Global variables: < heading , ultrasonic_value , readings , Leftreadings , Rightreadings , leftTotal , rightTotal , readIndex >
'''
import rclpy
from rclpy.node import Node
from threading import Thread
from std_msgs.msg import Float32MultiArray

rclpy.init()
global heading
global ultrasonic_value , readings , Leftreadings , Rightreadings , leftTotal , rightTotal , readIndex
Leftreadings = [0 for i in range(20)]
Rightreadings = [0 for i in range(20)]
leftTotal = 0
rightTotal = 0
ultrasonic_value = [0.0, 0.0]
heading = 0.0
readIndex = 0

def main():
    duplicateNode = Node('duplicate_node')
    executor = rclpy.executors.MultiThreadedExecutor(3)
    executor.add_node(duplicateNode)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    
    def ultrasonic_callback(msg):
        ''' 
        Purpose: to read the ultrasonic sensor data and update the ultrasonic_value
        args: msg
        return: None
        
        '''
        global ultrasonic_value
        ultrasonic_value[0] = round(msg.data[4],4)
        ultrasonic_value[1] = round(msg.data[5],4)  
    duplicateNode.ultra_sub = duplicateNode.create_subscription(Float32MultiArray, 'ultrasonic_sensor_std_float', ultrasonic_callback, 10)
    duplicateNode.ultrasonic_pub = duplicateNode.create_publisher(Float32MultiArray, '/ultrasonic_filter', 1)
    
    def movingAverage():
        '''
        Purpose: to calculate the moving average of the ultrasonic sensor data
        args: None
        return: leftaverage, rightaverage
        '''
        global readings, ultrasonic_value, Leftreadings, Rightreadings, leftTotal, rightTotal ,readIndex
        average =0
        
        leftTotal = leftTotal - Leftreadings[readIndex]
        rightTotal -= Rightreadings[readIndex]
        
        
        Leftreadings[readIndex] = ultrasonic_value[0]
        Rightreadings[readIndex] = ultrasonic_value[1]
        
        
        leftTotal = leftTotal + Leftreadings[readIndex]
        rightTotal = rightTotal + Rightreadings[readIndex]
        
        readIndex = readIndex + 1
        if (readIndex >= 20): 
            readIndex = 0
        leftaverage = leftTotal / 20
        rightaverage = rightTotal / 20
        return leftaverage, rightaverage
    def ultrasonic_publisher():
        '''
        Purpose: to publish the ultrasonic sensor data after calculating the moving average
        args: None
        return: None
        '''
        left,right = movingAverage()
        msg = Float32MultiArray()
        minValue = min(left,right)
        msg.data = [0.0,0.0,0.0,0.0,minValue,minValue]
        duplicateNode.ultrasonic_pub.publish(msg)
        
        
        
    rate = duplicateNode.create_rate(5, duplicateNode.get_clock())
    while True:
        ultrasonic_publisher()
        rate.sleep()
        # rclpy.spin_(duplicateNode)
    duplicateNode.destroy_node()
    rclpy.shutdown()
    exit(0)

if __name__ == '__main__':
    main()

