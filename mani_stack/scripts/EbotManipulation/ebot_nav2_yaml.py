#!/usr/bin/env python3
import os
import rclpy
from threading import Thread
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from mani_stack.srv import RackSw  # Import custom service message
import yaml
from ament_index_python.packages import get_package_share_directory
config_folder_name = 'mani_stack'
from std_msgs.msg import String,Bool
global dockingPosition
dockingPosition = {
      'ap1': {'xyz': [-0.2, -2.45, 0.0], 'quaternions': [0.0, 0.0, 0.9999996829318346, 0.0007963267107332633], 'XYoffsets': [1.0, 0.0], 'Yaw': 3.14},
      'ap2': {'xyz': [1.45,-4.50, 0.0], 'quaternions': [0.0, 0.0, -0.706825181105366, 0.7073882691671998], 'XYoffsets': [0.0, 1.0], 'Yaw': -1.57},
      'ap3': {'xyz': [1.45,-0.42, 0.0], 'quaternions': [0.0, 0.0, 0.706825181105366, 0.7073882691671998], 'XYoffsets': [0.0, -1.0], 'Yaw': 1.57}
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
def switch_case(yaw,cordinates):
    x, y = cordinates[0],cordinates[1]
    offsetXY=[]
    
    if yaw == 3.14:
        #180
        x -= 1.0
        offsetXY=[1.0,0.0]
      
    elif yaw == 1.57:
       #90
        y+=1.0
        offsetXY=[0.0,1.0]
    elif yaw == -1.57:
        #-90
        y-=1.0
        offsetXY=[0.0,-1.0]
    else:
        #-180
        x+=1.0
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
    
    node.nav2RackClient = node.create_client(RackSw, '/RackNav2Sw')
    node.ExitNavPub=node.create_publisher(Bool, '/ExitNav', 30)
    while not node.nav2RackClient.wait_for_service(timeout_sec=1.0):
        print(' Nav2 Client service not available, waiting again...')
    node.nav2RackRequest = RackSw.Request()
    
    global dockingPosition
    config_yaml = load_yaml(config_file)
    global package_id
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
        x,y,offsetXY=switch_case(yaw,xyz)
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
    for data in package_id:
        print("Start Loop")
        
        rackName="rack"+str(data)
        x=dockingPosition[rackName]['xyz'][0]
        y=dockingPosition[rackName]['xyz'][1]
        ap=findNearestAp(x,y)
        yaw=dockingPosition[rackName]['Yaw']
        x_offset=dockingPosition[rackName]['XYoffsets'][0]
        y_offset=dockingPosition[rackName]['XYoffsets'][1]
        node.nav2RackRequest.rack_name = rackName
        node.nav2RackRequest.box_id = data
        node.nav2RackRequest.ap_name = ap
        node.nav2RackRequest.x = x
        node.nav2RackRequest.y = y
        node.nav2RackRequest.yaw = yaw
        node.nav2RackRequest.offset_x = x_offset
        node.nav2RackRequest.offset_y = y_offset
        
        print("going to racks",node.nav2RackRequest)
        
        
        futureNav2 = node.nav2RackClient.call_async(node.nav2RackRequest)
        while(futureNav2.result() is  None):
            try:
                # node.aruco_name_publisher.publish(box_string)
               time.sleep(1)
            except KeyboardInterrupt:
                rclpy.spin(node)
                rclpy.shutdown()
                exit(0)
        
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