#!/usr/bin/env python3

# ros2 run mani_stack switchController.py --ros-args -p useMoveit:="True"

from controller_manager_msgs.srv import SwitchController
import time
import rclpy
from rclpy.node import Node
def main():
    rclpy.init()
    node = Node("controller_manager_switcher")
    node.declare_parameter(
        "useMoveit", True,
    )
    useMoveit = (
        node.get_parameter("useMoveit").get_parameter_value().bool_value
    )
    print("Switching to", "Moveit" if useMoveit else "Servo")

    contolMSwitch = node.create_client(
        SwitchController, "/controller_manager/switch_controller"
    )
    # Parameters to switch controller
    switchParam = SwitchController.Request()
    if useMoveit == True:
        switchParam.activate_controllers = [
            "scaled_joint_trajectory_controller"
        ]  # for normal use of moveit
        switchParam.deactivate_controllers = ["forward_position_controller"]
    else:
        switchParam.activate_controllers = [
            "forward_position_controller"
        ]  # for servoing
        switchParam.deactivate_controllers = [
            "scaled_joint_trajectory_controller"
        ]
    switchParam.strictness = 2
    switchParam.start_asap = False

    # calling control manager service after checking its availability
    while not contolMSwitch.wait_for_service(timeout_sec=5.0):
        node.get_logger().warn(
            f"Service control Manager is not yet available..."
        )
    contolMSwitch.call_async(switchParam)
    time.sleep(1.0)
    print(
        "[CM]: Switching to", "Moveit" if useMoveit else "Servo", "Complete"
    )
        
if __name__ == "__main__":
    main()
        