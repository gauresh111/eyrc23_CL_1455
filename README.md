# Welcome to eYRC Cosmo Logistic Theme 2023-24

### Packages
This repository contains three packages (as of now):
1. *aws-robomaker-small-warehouse-world*: Contains warehouse rack and package models

2. *ebot_description*: Contains mobile robot (ebot) description model

3. *eyantra_warehouse*: Contains warehouse world model

### task 4A

###build command

```sh
    colcon build --packages-select mani_stack
```
```sh
    colcon build --packages-select ebot_real_nav2
```
```sh
    colcon build --packages-select usb_relay
```
```sh
    colcon build --packages-select ebot_docking
```
###Run command
```sh
    ros2 launch ebot_real_nav2 ebot_nav2_brinup.launch.py
```
```sh
    ros2 run mani_stack docking_Hardware_boilerplate.py
```
```sh
    ros2 run mani_stack EbotTask4A.py
```

