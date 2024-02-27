'''
# Team ID:          < 1455 >
# Theme:            < Cosmo Logistic >
# Author List:      < Joel Devasia , Gauresh Wadekar >
# Filename:         < ebot_yaml.py >
# Functions:        < "load_yaml", "get_package_file", "add_docking_position", "switch_case", "find_string_in_list", "main", "getRackFromCamera", "aruco_data_updater", "distance", "findNearestAp" >
# Global variables: < aruco_ap_list, aruco_angle_list , aruco_name_list , dockingPosition >
'''

#!/usr/bin/env python3
import os
import rclpy
from threading import Thread
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from mani_stack.srv import RackSw ,ManipulationSw # Import custom service message
import yaml
import re
from ament_index_python.packages import get_package_share_directory
config_folder_name = 'mani_stack'
from std_msgs.msg import String,Bool
global dockingPosition
global aruco_name_list
global aruco_angle_list
global aruco_ap_list
aruco_name_list = []
aruco_angle_list = []
aruco_ap_list = []

dockingPosition = {
       'ap1':{'xyz': [4.63, -0.21,0.0], 'quaternions': [0.0, 0.0, 0.1, 0.0000], 'XYoffsets': [1.2,0.0], 'Yaw': 0.0},
      'ap2':{'xyz': [6.25, -1.8,0.0], 'quaternions': [0.0, 0.0, -0.706825181105366, 0.7073882691671998], 'XYoffsets': [0.0,1.0] , 'Yaw': 90}
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
    '''
    Purpose: Adds a new docking position to the dockingPosition dictionary.
    Arguments:
        name (str): The name of the docking position.
        xyz (list): A list of three coordinates (x, y, z) for the position.
        quaternions (list): A list of four quaternions representing the orientation.
        xy_offsets (list): A list of two offsets (x, y) for the position.
        yaw (float): The yaw angle of the position.
    Returns:
        None.
    '''
    global dockingPosition
    dockingPosition[name] = {
        'xyz': xyz,
        'quaternions': quaternions,
        'XYoffsets': xy_offsets,
        'Yaw':yaw
    }
def switch_case(yaw,cordinates):
    '''
    Purpose: Adjusts coordinates based on yaw angle.
    Arguments:
        yaw (float): The yaw angle.
        cordinates (list): A list of coordinates (x, y).
    Returns:
        tuple: A tuple containing adjusted coordinates (x, y) and offsets (x_offset, y_offset).
'''
    x, y = cordinates[0],cordinates[1]
    offsetXY=[]
    
    if yaw == 3.14:
        #180
        x -= 0.9
        offsetXY=[0.9,0.0]
      
    elif yaw == 1.57:
       #90
        y+=0.9
        offsetXY=[0.0,0.9]
    elif yaw == -1.57:
        #-90
        y-=0.9
        offsetXY=[0.0,-0.9]
    else:
        #-180
        x+=0.9
        offsetXY=[-0.9,0.0]    
    return x,y,offsetXY
def find_string_in_list(string, list):
    '''
    Purpose: Finds the index of a string within a list.
    Arguments:
        string (str): The string to search for.
        list (list): The list to search.
    Returns:
        int: The index of the string if found, otherwise -1.

    '''
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
    global rackPresentSub
    rackPresentSub = []
    def getRackFromCamera(data):
        '''
        Purpose: Processes camera data to identify present racks.
        Arguments:
            data (str): Camera data as a string.
        Returns:
            None.
        '''
        global rackPresentSub     
        rackPresentSub=data.data.split()
        rackPresentSub=set(rackPresentSub)
    def aruco_data_updater(msg):
        '''
        Purpose: Updates Aruco marker data.
        Arguments:
            msg (String message): Aruco data message.
        Returns:
            None.
        '''
        global aruco_name_list
        global aruco_angle_list
        global aruco_ap_list
        data = yaml.safe_load(msg.data)
        aruco_name_list = data.get("id")
        aruco_angle_list = data.get("angle")
        aruco_ap_list = data.get("ap")
    node.racksApsPub=node.create_publisher(Bool, '/StartArnManipulation', 10)
    node.nav2RackClient = node.create_client(RackSw, '/RackNav2Sw')
    node.ArmManipulationClient = node.create_client(ManipulationSw, '/ArmManipulationSw')
    node.ExitNavPub=node.create_publisher(Bool, '/ExitNav', 30)
    node.aruco_data_subscriber = node.create_subscription(String, "/aruco_data", aruco_data_updater, 10 ,callback_group=callback_group)
     
    node.getpresentRacksSub = node.create_subscription(String, '/ap_list',getRackFromCamera, 10)
    node.getpresentRacksSub
    while not node.nav2RackClient.wait_for_service(timeout_sec=1.0):
        print(' Nav2 Client service not available, waiting again...')
    while not node.ArmManipulationClient.wait_for_service(timeout_sec=1.0):
        print(' ArmManipulation Client service not available, waiting again...')
    node.nav2RackRequest = RackSw.Request()
    node.ArmManipulationRequest = ManipulationSw.Request()
    global dockingPosition
    config_yaml = load_yaml(config_file)
    global rackPresent,package_id
    rackPresent = 0
    racknameData = []
    package_id=[]
    for data in config_yaml["position"]:
        racknameData.append(list(data.keys())[0])
    package_id = config_yaml["package_id"]
    totalRacks = len(package_id)
    for data in range(len(package_id)):
        rackIndex = find_string_in_list("rack" + str(package_id[data]),racknameData)
        rackName = racknameData[rackIndex]
        #get xyz of rack
        xyz = [config_yaml["position"][rackIndex][rackName][0],config_yaml["position"][rackIndex][rackName][1],0]
        #get quaternions from eucler of rack
        yaw = config_yaml["position"][rackIndex][rackName][2]
        euler = [0,0,yaw]
        quaternions = R.from_euler('xyz', euler).as_quat().tolist()
        x,y,offsetXY=switch_case(yaw,xyz)
        xyz=[x,y,0.0]
        add_docking_position(rackName,xyz,quaternions,offsetXY,yaw)
    print(dockingPosition)
    rackPresentSub=[-1]
    while(-1 in rackPresentSub):
            time.sleep(0.1)
            print("waiting for ap list Node")
            print("rackPresentSub",rackPresentSub)
            
    if "Box" not in rackPresentSub:
        for boxPresent in rackPresentSub:
            global aruco_name_list
            global aruco_angle_list
            global aruco_ap_list
            rackIndex = find_string_in_list(boxPresent,aruco_ap_list)
            getName = aruco_name_list[rackIndex]
            BoxNumber = int(re.search(r"\d+", getName).group())
            node.ArmManipulationRequest.ap_name = boxPresent
            node.ArmManipulationRequest.box_id = int(BoxNumber)
            node.ArmManipulationRequest.total_racks = totalRacks
            node.ArmManipulationRequest.starting = True
            print("going to racks",node.ArmManipulationRequest)
            try: 
                del dockingPosition[boxPresent] 
            except:
                print("Rack not found")       
            try:
                del dockingPosition["rack"+str(int(getName[-1]))]
            except:
                print("Rack not found")
            package_id.remove(int(getName[-1]))
            futureArm = node.ArmManipulationClient.call_async(node.ArmManipulationRequest)
    def distance(p1, p2):
        '''
        Purpose: Calculates the Euclidean distance between two points.
        Arguments:
            p1 (list): A list of coordinates for the first point.
            p2 (list): A list of coordinates for the second point.
        Returns:
            float: The distance between the points.
        '''
        return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5
    def findNearestAp(X,Y):
        '''
        Purpose: Finds the nearest access point (AP) to a given location.
        Arguments:
            X (float): The x-coordinate of the location.
            Y (float): The y-coordinate of the location.
        Returns:
            str: The name of the nearest access point.
                
        '''
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
        node.ArmManipulationRequest.ap_name = ap
        node.ArmManipulationRequest.box_id = package_id[data]
        node.ArmManipulationRequest.total_racks = totalRacks
        node.ArmManipulationRequest.starting = False
        print("going to racks",node.nav2RackRequest)
        futureArm = node.ArmManipulationClient.call_async(node.ArmManipulationRequest)
        counter=0
        for i in range(2):
            msg = Bool()
            msg.data = False
            node.racksApsPub.publish(msg)
            time.sleep(0.1)
        while(futureArm.result() is  None and counter<10):
            try:
                # node.aruco_name_publisher.publish(box_string)
                print(futureArm.result())
                time.sleep(1)
            except KeyboardInterrupt:
                rclpy.spin(node)
                rclpy.shutdown()
                exit(0)
            counter+=1
        print("Arm Manipulation Response: ",futureArm.result())
        futureNav2 = node.nav2RackClient.call_async(node.nav2RackRequest)
        while(futureNav2.result() is  None):
            try:
                # node.aruco_name_publisher.publish(box_string)
               time.sleep(1)
            except KeyboardInterrupt:
                rclpy.spin(node)
                rclpy.shutdown()
                exit(0)
        # print("Start Arn Manipulation")
        print("Rack Client Response: ",futureNav2.result())
        for i in range(2):
            msg = Bool()
            msg.data = True
            node.racksApsPub.publish(msg)
            time.sleep(0.1)
    print("done")
    for i in range(20):
        msg = Bool()
        msg.data = True
        node.ExitNavPub.publish(msg)
        time.sleep(0.1)
    node.destroy_node()
    rclpy.shutdown()
    exit(0)
if __name__ == '__main__':
    main()
