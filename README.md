# Welcome to eYRC Cosmo Logistic Theme 2023-24

### Packages
This repository contains three packages (as of now):
1. *aws-robomaker-small-warehouse-world*: Contains warehouse rack and package models

2. *ebot_description*: Contains mobile robot (ebot) description model

3. *eyantra_warehouse*: Contains warehouse world model

### task 5

###build command

```sh
colcon build --packages-select mani_stack && source install/setup.bash
```
```sh
colcon build --packages-select ebot_real_nav2 && source install/setup.bash
```
```sh
colcon build --packages-select usb_relay && source install/setup.bash
```
```sh
colcon build --packages-select ebot_docking && source install/setup.bash
```

###Run command
```sh
source install/setup.bash && ros2 launch ebot_real_nav2 ebot_nav2_brinup.launch.py
```
```sh
source install/setup.bash && ros2 run mani_stack docking_Hardware_boilerplate.py
```
```sh
source install/setup.bash && ros2 run mani_stack EbotTask4A.py
```
```sh
source install/setup.bash && ros2 run mani_stack ebot_yaml.py
```
```sh
source install/setup.bash && ros2 run mani_stack simpleManipulation.py
```
```sh
source install/setup.bash && ros2 run mani_stack perception.py
```
###service reset command
```sh
ros2 service call /reset_odom std_srvs/srv/Trigger
```

```sh
ros2 service call /reset_imu std_srvs/srv/Trigger
```