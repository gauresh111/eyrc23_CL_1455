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



