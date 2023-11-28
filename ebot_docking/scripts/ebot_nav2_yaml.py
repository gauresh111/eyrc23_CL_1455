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
from ebot_docking.srv import DockSw  # Import custom service message
from tf_transformations import euler_from_quaternion
import math
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import Polygon,Point32
import yaml
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
config_folder_name = 'ebot_docking'

global dockingPosition
dockingPosition = {
        'initalPose':{'xyz': [0.0, 0.0, 0.0], 'quaternions': [0.0, 0.0, 0.0, 1.0], 'XYoffsets': [0.0, 0.0]},
        'ap1':{'xyz': [0.0, -2.45, 0.0], 'quaternions': [0.0, 0.0, 1.0, 0.0], 'XYoffsets': [0.7, 0.0]}, 
        'ap2':{'xyz': [1.37, -4.15, 0.0], 'quaternions': [0.0, 0.0, -0.7068252, 0.7073883], 'XYoffsets': [0.0, 0.8]}, 
        'ap3':{'xyz': [1.37, -1.04, 0.0], 'quaternions': [0.0, 0.0, 0.7068252, 0.7073883], 'XYoffsets': [0.0, -0.72]}           
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
def add_docking_position(name, xyz, quaternions, xy_offsets):
    global dockingPosition
    dockingPosition[name] = {
        'xyz': xyz,
        'quaternions': quaternions,
        'XYoffsets': xy_offsets
    }
def switch_case(value,cordinates):
    x, y = cordinates[0],cordinates[1]
    offsetXY=[]
    if value > 160:
        print("up")
        if x > 0:
            x -= 1.0
            offsetXY=[1.0,0.0]
        else:
            x += 1.0
            offsetXY=[-1.0,0.0]
    elif value >0:
        print("right")
        if  y > 0:
            y -= 1.0
            offsetXY=[0.0,1.0]
        else:
            y += 1.0
            offsetXY=[0.0,-1.0]
    elif value > -160:
        print("left")
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
        print("down")

    return x,y,offsetXY
    
    
def main():
    rclpy.init()
    navigator = BasicNavigator()
    node = Node("moveBotYaml")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()
    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(3)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    
    global dockingPosition
    config_yaml = load_yaml(config_file)
    
    withRackFootprint = [ [0.31, 0.40],[0.31, -0.40],[-0.31, -0.40],[-0.31, 0.40] ]
    withoutRackFootprint = [ [0.21, 0.195],[0.21, -0.195],[-0.21, -0.195],[-0.21, 0.195] ]
    global rackPresent
    rackPresent = 0
    for rack in config_yaml['position']:
        # rackName = 
        # add_docking_position('initalPose', [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], [0.0, 0.0])
        listNO = 1
        append = False
        #get name of rack
        while append ==False:
            try :
                rackName = "%s%d" % ("rack",listNO)
                #get xyz of rack
                xyz = [rack[rackName][0],rack[rackName][1],0]
                #get quaternions from eucler of rack
                euler = [0,rack[rackName][2],0]
                quaternions = R.from_euler('xyz', euler).as_quat().tolist()
                degree = math.degrees(rack[rackName][2])
                x,y,offsetXY=switch_case(math.ceil(degree),xyz)
                xyz=[x,y,0.0]
                rackPresent +=1
                append = True
            except:
                pass
            listNO += 1
        add_docking_position(rackName,xyz,quaternions,offsetXY)
        # print("rackName",rackName,"xyz",xyz,"quaternions",quaternions,"degree",degree,"offsetXY",offsetXY)
    
    def getGoalPoseStamped(goal):
        global dockingPosition
        Goal = dockingPosition[goal]
        goalPose = PoseStamped()
        goalPose.header.frame_id = 'map'
        goalPose.header.stamp = navigator.get_clock().now().to_msg()
        goalPose.pose.position.x = Goal['xyz'][0]
        goalPose.pose.position.y = Goal['xyz'][1]
        goalPose.pose.position.z = Goal['xyz'][2]
        goalPose.pose.orientation.x = Goal['quaternions'][0]
        goalPose.pose.orientation.y = Goal['quaternions'][1]
        goalPose.pose.orientation.z = Goal['quaternions'][2]
        goalPose.pose.orientation.w = Goal['quaternions'][3]
        # print(goalPose)
        return goalPose  
    global isDock
    isDock = False
    def change_footprint(new_footprint,msg):
        # Initialize ROS node
        nodeFootprint = rclpy.create_node('change_footprint_node')
        # Create a service client to set parameters
        nodeFootprint.localFootPrintPub=nodeFootprint.create_publisher(Polygon, '/local_costmap/footprint', 10)
        nodeFootprint.globalFootPrintPub=nodeFootprint.create_publisher(Polygon, '/global_costmap/footprint', 10)
        p = Polygon()
        p.points = [Point32(x=new_footprint[0][0], y=new_footprint[0][1]),
                            Point32(x=new_footprint[1][0], y=new_footprint[1][1]),
                            Point32(x=new_footprint[2][0], y=new_footprint[2][1]),
                            Point32(x=new_footprint[3][0], y=new_footprint[3][1])]
        for i in range (3):
            nodeFootprint.globalFootPrintPub.publish(p)
            nodeFootprint.localFootPrintPub.publish(p)
            time.sleep(0.1)
            print("publishing:" ,msg)
            
        nodeFootprint.destroy_node()
    def isDocking(data):
        global isDock 
        isDock = data.data
        # print(isDock)
        print(isDock,type(isDock),"callback")
    
    node.isDockingSub = node.create_subscription(
        Bool, "/dockingSuccesfull", isDocking, 10
    )
    global rackPresentSub
    rackPresentSub = []
    def getRack(data):
        global rackPresentSub
        data_list = data.data.split()
        rackPresentSub = set(data_list)
        # print(rackPresentSub)
    
    
    node.getRackSub = node.create_subscription(String,"/ap_list",getRack,10)
    node.isDockingSub
    node.publisher = node.create_publisher(PoseStamped, '/dockingRequest', 10)
    node.dockingClient = node.create_client(DockSw, '/dock_control')
    while not node.dockingClient.wait_for_service(timeout_sec=1.0):
        print('service not available, waiting again...')
    node.dockingRequest = DockSw.Request()
    
    navigator.setInitialPose(getGoalPoseStamped("initalPose"))

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()
    change_footprint(withoutRackFootprint,"withoutRackFootprint")
    # Go to the goal pose
    
    def getMissingPosition(givenList):
        if len(givenList)>=3 :
            return [0]
        if givenList == [-2]:
            return ["ap1"]
        positionList = ["ap1","ap2","ap3"]
        missingPosition = []
        for position in positionList:
            if position not in givenList:
                missingPosition.append(position)
        return missingPosition

    def moveToGoal(goalPose,rack_no,israck,positionName):
        global botPosition, botOrientation
        global dockingPosition
        
        
        # goalPose.pose.position.z=0.0
        if not israck:
            if rack_no=="initalPose":
                change_footprint(withoutRackFootprint,"withoutRackFootprint")
            else:
                change_footprint(withRackFootprint,"withRackFootprint")
        else:
            change_footprint(withoutRackFootprint,"withoutRackFootprint")
        navigator.goToPose(goalPose)
        i = 0

        # Keep doing stuff as long as the robot is moving towards the goal
        while not navigator.isTaskComplete():
            i = i + 1
        result = navigator.getResult()
        print(result)
        quaternion_array = goalPose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        yaw = math.degrees(yaw)
        Xoff = dockingPosition[positionName]['XYoffsets'][0]
        Yoff = dockingPosition[positionName]['XYoffsets'][1]
        node.dockingRequest.linear_dock = True
        node.dockingRequest.orientation_dock = True
        node.dockingRequest.goal_x = round(goalPose.pose.position.x+Xoff,2)
        node.dockingRequest.goal_y = round(goalPose.pose.position.y+Yoff,2)
        node.dockingRequest.orientation = round(yaw,2)
        node.dockingRequest.rack_no = rack_no
        node.dockingRequest.rack_attach=israck
        future = node.dockingClient.call_async(node.dockingRequest)
        print(node.dockingRequest)
        time.sleep(0.5)
        while isDock!=True:
            print("waiting")
    for rackspresent in range(rackPresent):
        rackPresentSub=[-1]
        rackList=1
        foundRack = False
        #get name of rack
        while foundRack ==False:
            
            rackName = "%s%d" % ("rack",rackList)
            value = dockingPosition.get(rackName)
            if value:
                
                foundRack = True
                while(-1 in rackPresentSub):
                    time.sleep(0.1)
                # print(rackPresentSub)
                # print("Key found:",rackName, value)
                getApRack = getMissingPosition(rackPresentSub)
                getApRack=getApRack[0]
                if getApRack==0:
                    break
                print(getApRack[0])
                moveToGoal(getGoalPoseStamped(rackName),rackName,True,rackName)
                moveToGoal(getGoalPoseStamped(getApRack),rackName,False,getApRack)
                del dockingPosition[rackName]
            else:
                print("Key not found")
            
            rackList +=1
    print("done")
    rclpy.spin(node)
    rclpy.shutdown()
    navigator.lifecycleShutdown()
    exit(0)
if __name__ == '__main__':
    main()
