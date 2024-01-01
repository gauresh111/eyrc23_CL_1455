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
    ############ Function VARIABLES ############
    # How to use global variables in functions
    # arucoImageWindow = None

    # ->  You can remove these variables if needed. These are just for suggestions to let you get started

    # Use this variable as a threshold value to detect aruco markers of certain size.
    # Ex: avoid markers/boxes placed far away from arm's reach position
    aruco_area_threshold = 100

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
    arucoImageWindow = image.copy()

    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray_image, (5, 5), 0)
    blur = cv2.bilateralFilter(blur, 9, 75, 75)
    threshold_image = cv2.threshold(blur, 50, 255, cv2.THRESH_TOZERO)[1]
    threshold_Otsu_image = cv2.threshold(blur, 50, 255, cv2.THRESH_OTSU)[1]
    adaptive_threshold_image = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 49, 2)



    #   ->  Use these aruco parameters-
    #       ->  Dictionary: 4x4_50 (4x4 only until 50 aruco IDs)
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    arucoParams = cv2.aruco.DetectorParameters()
    # arucoParams.perspectiveRemoveIgnoredMarginPerCell = 0.25
    # arucoParams.perspectiveRemovePixelPerCell = 10
    # arucoParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    # arucoParams.cornerRefinementWinSize = 10
    # arucoParams.maxErroneousBitsInBorderRate = 0.5
    # arucoParams.polygonalApproxAccuracyRate = 0.01

    #   ->  Detect aruco marker in the image and store 'corners' and 'ids'
    #       ->  HINT: Handle cases for empty markers detection.
    for image, name in zip(
        [
            threshold_image,
            threshold_Otsu_image,
            adaptive_threshold_image,
        ],
        [
            "threshold_image",
            "threshold_Otsu_image",
            "adaptive_threshold_image",
        ],
    ):
        corners, markerIds, rejected = cv2.aruco.detectMarkers(
            image, arucoDict, parameters=arucoParams
        )
        (corners, markerIds, rejected) = cv2.aruco.detectMarkers(
            image, arucoDict, parameters=arucoParams
        )

        #   ->  Draw detected marker on the image frame which will be shown later
        cv2.aruco.drawDetectedMarkers(image, corners, markerIds)

        #   ->  Loop over each marker ID detected in frame and calculate area using function defined above (calculate_rectangle_area(coordinates))
        # markerCorners = np.array(corners)
        corners = np.array(corners)

        corners = corners.reshape(-1, 4, 2)
        if corners is None or markerIds is None:
            print("No aruco detected", name)
            return (
                arucoImageWindow,
                threshold_image,
                threshold_Otsu_image,
                adaptive_threshold_image,
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

            cv2.drawFrameAxes(image, cam_mat, dist_mat, rvec, tvec, 0.2)

    return (
        arucoImageWindow,
        threshold_image,
        threshold_Otsu_image,
        adaptive_threshold_image,
    )

    # return (

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

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2  # rate of time to process image (seconds)
        self.bridge = CvBridge()  # initialise CvBridge object for image conversion
        self.timer = self.create_timer(
            image_processing_rate, self.process_image
        )  # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
        self.cv_image = None  # colour raw image variable (from colorimagecb())

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

        aruco_name_list = []

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        # 	->  Get aruco center, distance from rgb, angle, width and ids list from 'detect_aruco_center' defined above
        try:
            (
                arucoImageWindow,
                threshold_image,
                threshold_Otsu_image,
                adaptive_threshold_image,
            ) = detect_aruco(self.cv_image)
            arucoImageWindow = cv2.resize(arucoImageWindow, (640, 360))
            threshold_image = cv2.resize(threshold_image, (640, 360))
            threshold_Otsu_image = cv2.resize(threshold_Otsu_image, (640, 360))
            adaptive_threshold_image = cv2.resize(adaptive_threshold_image, (640, 360))

            # cv2.imshow("arucoImageWindow", arucoImageWindow)
            # cv2.imshow("threshold_image", threshold_image)
            # cv2.imshow("threshold_Otsu_image", threshold_Otsu_image)
            # cv2.imshow("adaptive_threshold_image", adaptive_threshold_image)
            arucoImageWindow = cv2.cvtColor(arucoImageWindow, cv2.COLOR_BGR2GRAY)
            arucoAndthres = np.concatenate((arucoImageWindow, threshold_image), axis=1)
            otsuAndadaptive = np.concatenate(
                (threshold_Otsu_image, adaptive_threshold_image), axis=1
            )
            img = np.concatenate((arucoAndthres, otsuAndadaptive), axis=0)
            cv2.imshow("arucoAndthres", img)
            cv2.waitKey(1)
        except Exception as e:
            print("process_image:", e)

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
