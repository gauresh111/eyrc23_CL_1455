#!/usr/bin/env python3


"""
*****************************************************************************************
*
*        		===============================================
*           		    Cosmo Logistic (CL) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script should be used to implement Task 1A of Cosmo Logistic (CL) Theme (eYRC 2023-24).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
"""

# Team ID:          [ CL#1455 ]
# Author List:		[ Joel Devasia, Gauresh Wadekar ]
# Filename:		    task1a.py
# Functions:
# 			        [ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node
#                   Example:
# 			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /etc... ]


################### IMPORT MODULES #######################

import rclpy
from std_msgs.msg import String
import sys
import cv2
import math
import tf2_ros
import transforms3d as tf3d
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CompressedImage, Image
from scipy.spatial.transform import Rotation as R

################### GLOBAL VARIABLES #######################


arucoImageWindow = None


##################### FUNCTION DEFINITIONS #######################


def calculate_rectangle_area(topLeft, topRight, bottomRight, bottomLeft):
    """
    Description:    Function to calculate area or detected aruco

    Args:
        coordinates (list):     coordinates of detected aruco (4 set of (x,y) coordinates)

    Returns:
        area        (float):    area of detected aruco
        width       (float):    width of detected aruco
    """

    ############ Function VARIABLES ############

    # You can remove these variables after reading the instructions. These are just for sample.

    area = None
    width = None

    ############ ADD YOUR CODE HERE ############

    # INSTRUCTIONS & HELP :
    # 	->  Recevice coordiantes from 'detectMarkers' using cv2.aruco library
    #       and use these coordinates to calculate area and width of aruco detected.
    # 	->  Extract values from input set of 4 (x,y) coordinates
    #       and formulate width and height of aruco detected to return 'area' and 'width'.

    ############################################
    length = math.sqrt(
        (topLeft[0] - topRight[0]) ** 2 + (topLeft[1] - topRight[1]) ** 2
    )
    width = math.sqrt(
        (topLeft[0] - bottomLeft[0]) ** 2 + (topLeft[1] - bottomLeft[1]) ** 2
    )
    area = length * width

    return area, length, width


def detect_aruco(image):
    """
    Description:    Function to perform aruco detection and return each detail of aruco detected
                    such as marker ID, distance, angle, width, center point location, etc.

    Args:
        image                   (Image):    Input image frame received from respective camera topic

    Returns:
        center_aruco_list       (list):     Center points of all aruco markers detected
        distance_from_rgb_list  (list):     Distance value of each aruco markers detected from RGB camera
        angle_aruco_list        (list):     Angle of all pose estimated for aruco marker
        width_aruco_list        (list):     Width of all detected aruco markers
        ids                     (list):     List of all aruco marker IDs detected in a single frame
    """

    ############ Function VARIABLES ############
    # How to use global variables in functions
    global arucoImageWindow

    # ->  You can remove these variables if needed. These are just for suggestions to let you get started

    # Use this variable as a threshold value to detect aruco markers of certain size.
    # Ex: avoid markers/boxes placed far away from arm's reach position
    aruco_area_threshold = 1500

    # The camera matrix is defined as per camera info loaded from the plugin used.
    # You may get this from /camer_info topic when camera is spawned in gazebo.
    # Make sure you verify this matrix once if there are calibration issues.
    cam_mat = np.array(
        [
            [931.1829833984375, 0.0, 640.0],
            [0.0, 931.1829833984375, 360.0],
            [0.0, 0.0, 1.0],
        ]
    )

    # The distortion matrix is currently set to 0.
    # We will be using it during Stage 2 hardware as Intel Realsense Camera provides these camera info.
    dist_mat = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

    # We are using 150x150 aruco marker sizeangle
    size_of_aruco_m = 0.15

    # You can remove these variables after reading the instructions. These are just for sample.
    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = []

    ############ ADD YOUR CODE HERE ############

    # INSTRUCTIONS & HELP :

    # 	->  Convert input BGR image to GRAYSCALE for aruco detection
    try:
        arucoImageWindow = image.copy()
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # threshold_image = cv2.adaptiveThreshold(gray_image, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 49, 2)
    except:
        return (
            center_aruco_list,
            distance_from_rgb_list,
            angle_aruco_list,
            width_aruco_list,
            ids,
        )

    #   ->  Use these aruco parameters-
    #       ->  Dictionary: 4x4_50 (4x4 only until 50 aruco IDs)
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    try:
        arucoParams = cv2.aruco.DetectorParameters_create()
    except:
        arucoParams = cv2.aruco.DetectorParameters()

    #   ->  Detect aruco marker in the image and store 'corners' and 'ids'
    #       ->  HINT: Handle cases for empty markers detection.
    (corners, markerIds, rejected) = cv2.aruco.detectMarkers(
        gray_image, arucoDict, parameters=arucoParams
    )

    #   ->  Draw detected marker on the image frame which will be shown later
    cv2.aruco.drawDetectedMarkers(arucoImageWindow, corners, markerIds)

    #   ->  Loop over each marker ID detected in frame and calculate area using function defined above (calculate_rectangle_area(coordinates))
    # markerCorners = np.array(corners)
    corners = np.array(corners)

    corners = corners.reshape(-1, 4, 2)
    if corners is None or markerIds is None:
        # print("No aruco detected", rclpy.clock.Clock().now().to_msg())
        return (
            center_aruco_list,
            distance_from_rgb_list,
            angle_aruco_list,
            width_aruco_list,
            ids,
        )
    # for corner, id, markerCorner in zip(corners, markerIds, markerCorners):
    for corner, id in zip(corners, markerIds):
        area, length, width = calculate_rectangle_area(
            corner[0], corner[1], corner[2], corner[3]
        )

        #   ->  Remove tags which are far away from arm's reach positon based on some threshold defined
        if area < aruco_area_threshold:
            continue

        ids.append(id[0])

        width_aruco_list.append(width)

        #   ->  Calculate center points aruco list using math and distance from RGB camera using pose estimation of aruco marker
        #       ->  HINT: You may use numpy for center points and 'estimatePoseSingleMarkers' from cv2 aruco library for pose estimation
        center = np.mean(corner, axis=0)
        center = list(center)
        center_aruco_list.append(center)

        # markerCorner.reshape(4, 2)
        # obj_pts = np.array(
        #     [
        #         [0, 0, 0],
        #         [size_of_aruco_m, 0, 0],
        #         [size_of_aruco_m, size_of_aruco_m, 0],
        #         [0, size_of_aruco_m, 0],
        #     ],
        #     dtype=np.float32,
        # )
        # ret, rvec, tvec = cv2.solvePnP(obj_pts, markerCorner, cam_mat, dist_mat)

        rvec, tvec, objectPoints = cv2.aruco.estimatePoseSingleMarkers(
            [corner], size_of_aruco_m, cam_mat, dist_mat
        )

        rvec = np.array(rvec)
        rvec = rvec.reshape(3)
        tvec = np.array(tvec)
        tvec = tvec.reshape(3)

        angles = R.from_rotvec(rvec)
        angles = list(angles.as_euler("xyz", degrees=False))
        angle_aruco_list.append(angles)

        distance = tvec[2]
        distance_from_rgb_list.append(distance)

        #   ->  Draw frame axes from coordinates received using pose estimation
        #       ->  HINT: You may use 'cv2.drawFrameAxes'
        cv2.drawFrameAxes(arucoImageWindow, cam_mat, dist_mat, rvec, tvec, 0.2)

    return (
        center_aruco_list,
        distance_from_rgb_list,
        angle_aruco_list,
        width_aruco_list,
        ids,
    )
    ############################################


##################### CLASS DEFINITION #######################


class aruco_tf(Node):
    """
    ___CLASS___

    Description:    Class which servers purpose to define process for detecting aruco marker and publishing tf on pose estimated.
    """

    def __init__(self):
        """
        Description:    Initialization of class aruco_tf
                        All classes have a function called __init__(), which is always executed when the class is being initiated.
                        The __init__() function is called automatically every time the class is being used to create a new object.
                        You can find more on this topic here -> https://www.w3schools.com/python/python_classes.asp
        """

        super().__init__("aruco_tf_publisher")  # registering node
        print("Aruco tf publisher node registered")

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.colorimagecb, 10
        )
        self.depth_cam_sub = self.create_subscription(
            Image, "/camera/aligned_depth_to_color/image_raw", self.depthimagecb, 10
        )
        print("Subscribed to topics")
        self.aruco_name_publisher = self.create_publisher(
            String, "/aruco_list", 10
        )
        self.Ap_name_publisher = self.create_publisher(String, "/ap_list", 10)
        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2  # rate of time to process image (seconds)
        self.bridge = CvBridge()  # initialise CvBridge object for image conversion
        self.tf_buffer = (
            tf2_ros.buffer.Buffer()
        )  # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(
            self
        )  # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(
            image_processing_rate, self.process_image
        )  # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
        self.cv_image = None  # colour raw image variable (from colorimagecb())
        self.depth_image = None  # depth image variable (from depthimagecb())
        self.offsetQuaternions = [0, 0, 0, 0]
        self.offsetEuler = [0, 0, 0]

    def depthimagecb(self, data):
        """
        Description:    Callback function for aligned depth camera topic.
                        Use this function to receive image depth data and convert to CV2 image

        Args:
            data (Image):    Input depth image frame received from aligned depth camera topic

        Returns:
        """

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        # 	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT: You may use CvBridge to do the same

        ############################################
        try:
            image = self.bridge.imgmsg_to_cv2(data, "passthrough")
            self.depth_image = image.copy()
        except:
            pass

    def colorimagecb(self, data):
        """
        Description:    Callback function for colour camera raw topic.
                        Use this function to receive raw image data and convert to CV2 image

        Args:
            data (Image):    Input coloured raw image frame received from image_raw camera topic

        Returns:
        """

        ############ ADD YOUR CODE HERE ############

        # 	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT:   You may use CvBridge to do the same
        #               Check if you need any rotation or flipping image as input data maybe different than what you expect to be.
        #               You may use cv2 functions such as 'flip' and 'rotate' to do the same

        ############################################
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.cv_image = image.copy()
        except:
            pass

    def normalizeAngle(self,angle: int):
        if angle < -180:
            angle += 360
        elif angle > 180:
            angle -= 360
        return angle
    
    def nearest_angle(self,angle):
        if angle < -180:
            angle += 360
        elif angle > 180:
            angle -= 360

        if abs(angle) < 45:
            nearest = 0
        elif abs(angle) < 135:
            nearest = 90 if angle > 0 else -90
        else:
            nearest = 180 if angle > 0 else -180

        return nearest
    def get_rack_name(self,angle):
        if angle == 0:
            rack_name = "ap1"
        elif angle == 90:
            rack_name = "ap2"
        elif angle == -90:
            rack_name = "ap3"
        return rack_name
    def process_image(self):
        """
        Description:    Timer function used to detect aruco markers and publish tf on estimated poses.

        Args:
        Returns:
        """

        ############ Function VARIABLES ############

        # These are the variables defined from camera info topic such as image pixel size, focalX, focalY, etc.
        # Make sure you verify these variable values once. As it may affect your result.
        # You can find more on these variables here -> http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html

        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375

        aruco_name_list = []
        aruco_angle_list = []
        rackName = []
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        # 	->  Get aruco center, distance from rgb, angle, width and ids list from 'detect_aruco_center' defined above
        centers, distances, angles, widths, ids = detect_aruco(self.cv_image)

        #   ->  Loop over detected box ids received to calculate position and orientation transform to publish TF
        for center, distance, angle, width, id in zip(
            centers, distances, angles, widths, ids
        ):

            #   ->  Use this equation to correct the input aruco angle received from cv2 aruco function 'estimatePoseSingleMarkers' here
            #       It's a correction formula-
            # angle_aruco = (0.788*angle_aruco) - ((angle_aruco**2)/3160)
            angle[1] = angle[1] + (0.2 * angle[1]) - ((angle[1] ** 2) / 3160)
            # if id  == 3:
            #     print(angle[1])

            #   ->  Then calculate quaternions from roll pitch yaw (where, roll and pitch are 0 while yaw is corrected aruco_angle)
            newMarkerAngle = [0 + np.pi / 2 - self.offsetEuler[1], 0, 0 + np.pi / 2]

            quaternions = tf3d.euler.euler2quat(
                newMarkerAngle[0], newMarkerAngle[1], newMarkerAngle[2]
            )

            #   ->  Use center_aruco_list to get realsense depth and log them down. (divide by 1000 to convert mm to m)
            depth_distance = self.depth_image[int(center[1])][int(center[0])] / 1000

            #   ->  Use this formula to rectify x, y, z based on focal length, center value and size of image
            #       x = distance_from_rgb * (sizeCamX - cX - centerCamX) / focalX
            #       y = distance_from_rgb * (sizeCamY - cY - centerCamY) / focalY
            #       z = distance_from_rgb
            #       where,
            #               cX, and cY from 'center_aruco_list'
            #               distance_from_rgb is depth of object calculated in previous step
            #               sizeCamX, sizeCamY, centerCamX, centerCamY, focalX and focalY are defined above
            x = depth_distance * (sizeCamX - center[0] - centerCamX) / focalX
            y = depth_distance * (sizeCamY - center[1] - centerCamY) / focalY
            z = depth_distance

            #   ->  Now, mark the center points on image frame using cX and cY variables with help of 'cv2.cirle' function
            cv2.circle(arucoImageWindow, (int(center[0]), int(center[1])), 5, (0, 0, 255), -1)

            #   ->  Here, till now you receive coordinates from camera_link to aruco marker center position.
            #       So, publish this transform w.r.t. camera_link using Geometry Message - TransformStamped
            #       so that we will collect it's position w.r.t base_link in next step.
            #       Use the following frame_id-
            #           frame_id = 'camera_link'
            #           child_frame_id = 'cam_<marker_id>'          Ex: cam_20, where 20 is aruco marker ID
            transformStamped = TransformStamped()
            transformStamped.header.stamp = self.get_clock().now().to_msg()
            transformStamped.header.frame_id = "camera_link"
            transformStamped.child_frame_id = "cam_fixed_" + str(id)
            transformStamped.transform.translation.x = z
            transformStamped.transform.translation.y = x
            transformStamped.transform.translation.z = y
            transformStamped.transform.rotation.x = quaternions[1]
            transformStamped.transform.rotation.y = quaternions[2]
            transformStamped.transform.rotation.z = quaternions[3]
            transformStamped.transform.rotation.w = quaternions[0]
            self.br.sendTransform(transformStamped)
            try:
                trans = self.tf_buffer.lookup_transform(
                    "base_link", "camera_link", rclpy.time.Time()
                )
                # qX, qY, qZ, qW = trans.transform.rotation.geometry_msgs.msg.Quaternion
                self.offsetQuaternions = [
                    trans.transform.rotation.w,
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                ]
                temp = tf3d.euler.quat2euler(self.offsetQuaternions)
                self.offsetEuler = [temp[0], temp[1], temp[2]]
            except:
                print("no base_link to camera_link transform found")
                pass

            finalQuat = tf3d.euler.euler2quat(0, -angle[1], 0)

            transformStamped = TransformStamped()
            transformStamped.header.stamp = self.get_clock().now().to_msg()
            transformStamped.header.frame_id = "cam_fixed_" + str(id)
            transformStamped.child_frame_id = "cam_" + str(id)
            transformStamped.transform.translation.x = 0.0
            transformStamped.transform.translation.y = 0.0
            transformStamped.transform.translation.z = 0.0
            transformStamped.transform.rotation.x = finalQuat[1]
            transformStamped.transform.rotation.y = finalQuat[2]
            transformStamped.transform.rotation.z = finalQuat[3]
            transformStamped.transform.rotation.w = finalQuat[0]
            self.br.sendTransform(transformStamped)

            # Get TF orientation of camera_link wrt base_link using listener

            #   ->  Then finally lookup transform between base_link and obj frame to publish the TF
            #       You may use 'lookup_transform' function to pose of obj frame w.r.t base_link
            try:
                tranform = self.tf_buffer.lookup_transform(
                    "base_link", "cam_" + str(id), rclpy.time.Time()
                )
            #   ->  And now publish TF between object frame and base_link
            #       Use the following frame_id-
            #           frame_id = 'base_link'
            #           child_frame_id = 'obj_<marker_id>'          Ex: obj_20, where 20 is aruco marker ID
                transformStamped = TransformStamped()
                transformStamped.header.stamp = self.get_clock().now().to_msg()
                transformStamped.header.frame_id = "base_link"
                transformStamped.child_frame_id = "obj_" + str(id)
                transformStamped.transform.translation.x = (
                    tranform.transform.translation.x
                )
                transformStamped.transform.translation.y = (
                    tranform.transform.translation.y
                )
                transformStamped.transform.translation.z = (
                    tranform.transform.translation.z
                )
                transformStamped.transform.rotation.x = tranform.transform.rotation.x
                transformStamped.transform.rotation.y = tranform.transform.rotation.y
                transformStamped.transform.rotation.z = tranform.transform.rotation.z
                transformStamped.transform.rotation.w = tranform.transform.rotation.w
                self.br.sendTransform(transformStamped)
                aruco_name_list.append("obj_" + str(id))
                angleDegree=int(math.degrees(-angle[1]))
                print("angleDegree:", angleDegree)
                tempValue = self.normalizeAngle(angleDegree)
                # aruco_angle_list.append(self.normalizeAngle(1))
                print("angle:", tempValue)
                aruco_angle_list.append(angleDegree)
                rackName.append(self.get_rack_name(self.nearest_angle(angleDegree)))
            except Exception as e:
                print(e)
                pass        

        #   ->  At last show cv2 image window having detected markers drawn and center points located using 'cv2.imshow' function.
        #       Refer MD book on portal for sample image -> https://portal.e-yantra.org/
        
        try:
            # cv2.imshow("aruco_image", arucoImageWindow)
            # cv2.waitKey(1)
            tempStr = " "
            tempName = " "
            aruco_string = String()
            aruco_string.data =  tempStr.join(aruco_name_list)
            rack_string = String()
            rack_string.data =  tempName.join(rackName)
            if len(rack_string.data) == 0:
                rack_string.data = "-2"
            self.Ap_name_publisher.publish(rack_string)
            print("Rack_string:",rack_string)
            print("Aruco_List:", aruco_string)
            # if len(aruco_name_list)!=0:
            #     for name, angle in zip(aruco_name_list, aruco_angle_list):
            #         print(name+":", angle, end=" ")
            #     print()
            print(aruco_name_list)
            print(aruco_angle_list)
            self.aruco_name_publisher.publish(aruco_string)
        except:
            pass
        #   ->  NOTE:   The Z axis of TF should be pointing inside the box (Purpose of this will be known in task 1B)
        #               Also, auto eval script will be judging angular difference aswell. So, make sure that Z axis is inside the box (Refer sample images on Portal - MD book)

        ############################################


##################### FUNCTION DEFINITION #######################


def main():
    """
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    """
    rclpy.init(args=sys.argv)  # initialisation
    node = rclpy.create_node("aruco_tf_process")  # creating ROS node
    node.get_logger().info("Node created: Aruco tf process")  # logging information
    aruco_tf_class = aruco_tf()  # creating a new object for class 'aruco_tf'
    rclpy.spin(aruco_tf_class)  # spining on the object to make it alive in ROS 2 DDS
    aruco_tf_class.destroy_node()  # destroy node after spin ends
    rclpy.shutdown()  # shutdown process


if __name__ == "__main__":
    """
    Description:    If the python interpreter is running that module (the source file) as the main program,
                    it sets the special __name__ variable to have a value “__main__”.
                    If this file is being imported from another module, __name__ will be set to the module's name.
                    You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/
    """

    main()