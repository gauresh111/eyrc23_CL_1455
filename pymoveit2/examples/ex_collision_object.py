#!/usr/bin/env python3
"""
Example of adding and removing a collision object with a mesh geometry.
Note: Python module `trimesh` is required for this example (`pip install trimesh`).
`ros2 run pymoveit2 ex_collision_object.py --ros-args -p action:="add" -p position:="[0.5, 0.0, 0.5]" -p quat_xyzw:="[0.0, 0.0, -0.707, 0.707]"`
`ros2 run pymoveit2 ex_collision_object.py --ros-args -p action:="add" -p filepath:="./my_favourity_mesh.stl"`
`ros2 run pymoveit2 ex_collision_object.py --ros-args -p action:="remove"`
"""

from os import path
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5

DEFAULT_EXAMPLE_MESH = path.join(
    path.dirname(path.realpath(__file__)), "assets", "suzanne.stl"
)
box = path.join(
    path.dirname(path.realpath(__file__)), "assets", "box.stl"
)
# rack = path.join(
#     path.dirname(path.realpath(__file__)), "assets", "rack.stl"
# )
rack = path.join(
    path.dirname(path.realpath(__file__)), "assets", "simpleRack.stl"
)
floor = path.join(
    path.dirname(path.realpath(__file__)), "assets", "floor.stl"
)
def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_collision_object")

    # Declare parameter for joint positions
    node.declare_parameter(
        "action",
        "add",
    )

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Get parameters
    action_parameter = node.get_parameter("action").get_parameter_value().string_value
    action = action_parameter
    # action="add"

    if "add" == action:
        for i in range(3):
            # moveit2.add_collision_mesh(
            #     filepath=box, id="currentBox", position=[0.0, -0.12, 0.09], quat_xyzw=[ -0.5, 0.5, 0.5, 0.5  ], frame_id='tool0',
            # )
            #right_side#
            # moveit2.add_collision_mesh(
            #     filepath=rack, id="Right_rack", position=[0.25, -0.65, -0.57], quat_xyzw=[0, 0, 0.717356,0.696707], frame_id=ur5.base_link_name()
            # )
            # moveit2.add_collision_mesh(
            #     filepath=rack, id="Right_rack", position=[0.25, -0.65, 0.16], quat_xyzw=[0, 0, 0.717356,0.696707], frame_id=ur5.base_link_name()
            # )
        
            #left_side#
            # moveit2.add_collision_mesh(
            #     filepath=rack, id="Left_rack", position=[0.25, 0.71, -0.57], quat_xyzw=[0, 0, 0.717356,0.696707],
            #     frame_id=ur5.base_link_name()
            # )
            # moveit2.add_collision_mesh(
            #     filepath=rack, id="Left_rack", position=[0.25, 0.71, 0.16], quat_xyzw=[0, 0, 0.717356,0.696707],
            #     frame_id=ur5.base_link_name()
            # )
            # center_side
            # moveit2.add_collision_mesh(
            #     filepath=rack, id="Center_rack", position=[0.54, 0.07, -0.57], quat_xyzw=[0, 0, 0, 1],
            #     frame_id=ur5.base_link_name()
            # )
            # moveit2.add_collision_mesh(
            #     filepath=rack, id="Center_rack", position=[0.54, 0.07, 0.16], quat_xyzw=[0, 0, 0, 1],
            #     frame_id=ur5.base_link_name()
            # )
            # moveit2.add_collision_mesh(
            #     filepath=box, id="Box_1", position=[0.48, 0.10, 0.55], quat_xyzw=[0, 0, 0, 1],
            #     frame_id=ur5.base_link_name()
            # )
            # moveit2.add_collision_mesh(
            #     filepath=box, id="Box_2", position=[0.30, -0.56, 0.54], quat_xyzw=[0 ,0 ,0.706825 ,0.707388],
            #     frame_id=ur5.base_link_name()
            # )
            # moveit2.add_collision_mesh(
            #     filepath=floor, id="Floor", position=[-0.04, 0.03, -0.04], quat_xyzw=[0.0, 0.0, 0.0, 1.0],
            #     frame_id=ur5.base_link_name()
            # )
            # ##Left Floor
            # moveit2.add_collision_mesh(
            #     filepath=floor, id="Floor_Left", position=[-0.10, 0.60, 0.30], quat_xyzw=[0.7071081, 0, 0, 0.7071055 ],
            #     frame_id=ur5.base_link_name()
            # )
            ##Middle Floor
            # moveit2.add_collision_mesh(
            #     filepath=floor, id="Floor_Center", position=[0.40, 0.05, 0.4], quat_xyzw=[ 0, 0.7071081, 0, 0.7071055 ],
            #     frame_id=ur5.base_link_name()
            # )
            ##Right Floor
            # moveit2.add_collision_mesh(
            #     filepath=floor, id="Floor_Right", position=[-0.10, -0.47, 0.30], quat_xyzw=[0.7071081, 0, 0, 0.7071055 ],
            #     frame_id=ur5.base_link_name()
            # )
            moveit2.add_collision_mesh(
                    filepath=box,
                    id="currentBox",
                    position=[0.0, -0.12, 0.11],
                    quat_xyzw=[-0.5, 0.5, 0.5, 0.5],
                    frame_id="tool0",
                )
        
    else:
        for i in range(3):
            # print("remove")
            # moveit2.remove_collision_mesh(id="Right_rack")
            # moveit2.remove_collision_mesh(id="Left_rack")
            # moveit2.remove_collision_mesh(id="Center_rack")
            print("Removing collision object")
            moveit2.remove_collision_mesh(id="currentBox")
            # moveit2.remove_collision_mesh(id="Box_2")
            # moveit2.remove_collision_mesh(id="Floor")
            # moveit2.remove_collision_mesh(id="Floor_Left")
            # moveit2.remove_collision_mesh(id="Floor_Center")
            # moveit2.remove_collision_mesh(id="Floor_Right")

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
