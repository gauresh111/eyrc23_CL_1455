# Welcome to eYRC Cosmo Logistic Theme 2023-24

### Packages
This repository contains three packages (as of now):
1. *aws-robomaker-small-warehouse-world*: Contains warehouse rack and package models

2. *ebot_description*: Contains mobile robot (ebot) description model

3. *eyantra_warehouse*: Contains warehouse world model

### Task 0

To launch task 0, use this command-

```sh
ros2 launch ebot_description ebot_gazebo_launch.py
```

This should open gazebo application having mobile robot (*named as ebot*) spawned inside a warehouse.

### Task 1

Task 1 is subdivided into three parts:

- Task 1A: Detecttion of Aruco Markers on the packages placed in the rack
To launch task 1A, use this commands in seperate terminals-

    ```sh
    ros2 launch ur_description ur5_gazebo_launch.py
    ```
    ```sh
    ros2 launch ur5_moveit spawn_ur5_launch_moveit.py
    ```
    ```sh
    ros2 run ur_description task1a.py
    ```
- Task 1B: Manipulation of Arm to reach the package pose
To launch task 1B, use this commands in seperate terminals-

    ```sh
    ros2 launch ur_description ur5_gazebo_launch.py
    ```
    ```sh
    ros2 launch ur5_moveit spawn_ur5_launch_moveit.launch.py
    ```
    ```sh
    ros2 run mani_stack task1b.py
    ```
- Task 1C: Navigation of ebot in the warehouse
To launch task 1C, use this commands in seperate terminals-
    ```sh
    ros2 launch ebot_description ebot_gazebo_launch.py
    ```
    ```sh
    ros2 launch ebot_nav2 ebot_bringup_launch.py
    ```
    ```sh
    ros2 run ebot_nav2 nav2_cmd.py
    ```

### Task 2
Task 2 is subdivided into two parts:

- Task 2A: Pickup randomly placed Aruco Markers from the rack and place them in the Drop position-
To launch task 2A, use this commands in seperate terminals-

    ```sh
    ros2 launch ur_description ur5_gazebo_launch.py
    ```
    ```sh
    ros2 launch ur5_moveit spawn_ur5_launch_moveit.launch.py
    ```
    ```sh
    ros2 run mani_stack task1ab-perception.py
    ```
    ```sh
    ros2 run pymoveit2 ex_collision_object.py
    ```
    ```sh
    ros2 run mani_stack task1ab-manipulation.py
    ```
    or to run with servo
    
    ```sh
    ros2 run mani_stack task1ab-manipulation-servo.py
    ```
    ```sh
    ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
    ```

- Task 2B: Moving Racks to the picking area using ebot-
To launch task 2B, use this commands in seperate terminals-

    ```sh
    ros2 launch ebot_description ebot_gazebo_launch.py
    ```
    ```sh
    ros2 launch ebot_nav2 ebot_bringup_launch.py
    ```
    ```sh
    ros2 run ebot_nav2 ebot_nav2_cmd_task2b.py
    ```
    ```sh
    ros2 run ebot_docking ebot_docking_service_task2b.py
    ```


