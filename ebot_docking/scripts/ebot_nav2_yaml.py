#!/usr/bin/env python3
import os
from nav_msgs.msg import Odometry
import rclpy
from threading import Thread
import time
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
import tf2_ros
from rclpy.duration import Duration # Handles time for ROS 2
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool
from ebot_docking.srv import RackSw  # Import custom service message
from tf_transformations import euler_from_quaternion
import math
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import Polygon,Point32
import yaml
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from std_msgs.msg import Bool
config_folder_name = 'ebot_docking'

global dockingPosition
dockingPosition = {
        'ap1':{'xyz': [0.0, -2.45, 0.0], 'quaternions': [0.0, 0.0, 1.0, 0.0], 'XYoffsets': [0.7, 0.0],'Yaw':180}, 
        'ap2':{'xyz': [1.37, -4.15, 0.0], 'quaternions': [0.0, 0.0, -0.7068252, 0.7073883], 'XYoffsets': [0.0, 0.8],'Yaw':-1.57}, 
        'ap3':{'xyz': [1.37, -1.04, 0.0], 'quaternions': [0.0, 0.0, 0.7068252, 0.7073883], 'XYoffsets': [0.0, -0.72],'Yaw':1.57}           
}
def load_yaml(file_path):
    """Load a yaml file into a dictionary"""
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None
def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path
config_file = get_package_file(config_folder_name, 'config.yaml')
def add_docking_position(name, xyz, quaternions, xy_offsets,yaw):
    global dockingPosition
    dockingPosition[name] = {
        'xyz': xyz,
        'quaternions': quaternions,
        'XYoffsets': xy_offsets,
        'Yaw':yaw
    }
def switch_case(value,cordinates):
    x, y = cordinates[0],cordinates[1]
    offsetXY=[]
    if value > 160:
      
        if x > 0:
            x -= 1.0
            offsetXY=[1.0,0.0]
        else:
            x += 1.0
            offsetXY=[-1.0,0.0]
    elif value >0:
       
        if  y > 0:
            y -= 1.0
            offsetXY=[0.0,1.0]
        else:
            y += 1.0
            offsetXY=[0.0,-1.0]
    elif value > -160:
        
        if y > 0:
            y -= 1.0
            offsetXY=[0.0,1.0]
        else:
            y += 1.0
            offsetXY=[0.0,-1.0]
    else:
        if x > 0:
            x -= 1.0
            offsetXY=[1.0,0.0]
        else:
            x += 1.0
            offsetXY=[-1.0,0.0]
        

    return x,y,offsetXY
def find_string_in_list(string, list):
    for index, item in enumerate(list):
        if item == string:
            return index
    return -1
    
def main():
    rclpy.init()
    node = Node("moveBotYaml")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()
    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(3)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.racksApsPub=node.create_publisher(Bool, '/StartArnManipulation', 10)
    node.nav2RackClient = node.create_client(RackSw, '/RackNav2Sw')
    while not node.nav2RackClient.wait_for_service(timeout_sec=1.0):
        print(' Nav2 Client service not available, waiting again...')
    node.nav2RackRequest = RackSw.Request()
    global dockingPosition
    config_yaml = load_yaml(config_file)
    global rackPresent,package_id
    rackPresent = 0
    racknameData = []
    package_id=[]
    for data in config_yaml["position"]:
        racknameData.append(list(data.keys())[0])
    package_id = config_yaml["package_id"]
    
    for data in range(len(package_id)):
        rackIndex = find_string_in_list("rack" + str(package_id[data]),racknameData)
        rackName = racknameData[rackIndex]
        #get xyz of rack
        xyz = [config_yaml["position"][rackIndex][rackName][0],config_yaml["position"][rackIndex][rackName][1],0]
        #get quaternions from eucler of rack
        yaw = config_yaml["position"][rackIndex][rackName][2]
        euler = [0,0,yaw]
        quaternions = R.from_euler('xyz', euler).as_quat().tolist()
        degree = math.degrees(yaw)
        x,y,offsetXY=switch_case(math.ceil(degree),xyz)
        xyz=[x,y,0.0]
        add_docking_position(rackName,xyz,quaternions,offsetXY,yaw)
     
    def distance(p1, p2):
        return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5
    def findNearestAp(X,Y):
        global dockingPosition
        nearest_ap = None
        min_distance = float('inf')
        for ap in dockingPosition.keys():
            #get distance
            
            search_rack = ap.find("r",)
            if search_rack == -1:
                x1=dockingPosition[ap]["xyz"][0]
                y1=dockingPosition[ap]["xyz"][1]
                latestDistance = distance([x, y], [x1, y1])
                if latestDistance < min_distance:
                    min_distance = latestDistance
                    nearest_ap = ap
        del dockingPosition[nearest_ap]        
        return nearest_ap
    for data in range(len(package_id)):
        print("Start Loop")
        
        rackName="rack"+str(package_id[data])
        x=dockingPosition[rackName]['xyz'][0]
        y=dockingPosition[rackName]['xyz'][1]
        ap=findNearestAp(x,y)
        yaw=dockingPosition[rackName]['Yaw']
        x_offset=dockingPosition[rackName]['XYoffsets'][0]
        y_offset=dockingPosition[rackName]['XYoffsets'][1]
        node.nav2RackRequest.rack_name = rackName
        node.nav2RackRequest.box_id = package_id[data]
        node.nav2RackRequest.ap_name = ap
        node.nav2RackRequest.x = x
        node.nav2RackRequest.y = y
        node.nav2RackRequest.yaw = yaw
        node.nav2RackRequest.offset_x = x_offset
        node.nav2RackRequest.offset_y = y_offset
        print("going to racks",node.nav2RackRequest)
        future = node.nav2RackClient.call_async(node.nav2RackRequest)
        while(future.result() is  None):
            try:
                # node.aruco_name_publisher.publish(box_string)
                print("going to racks",node.nav2RackRequest)
            except KeyboardInterrupt:
                rclpy.spin(node)
                rclpy.shutdown()
                exit(0)
        print("Start Arn Manipulation")
        for i in range(10):
            msg = Bool()
            msg.data = True
            node.racksApsPub.publish(msg)
            time.sleep(0.1)
    
    print("done")
    rclpy.spin(node)
    rclpy.shutdown()
    exit(0)
if __name__ == '__main__':
    main()
