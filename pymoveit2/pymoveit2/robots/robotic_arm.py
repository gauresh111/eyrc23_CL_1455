from typing import List

MOVE_GROUP_ARM: str = "arm"
MOVE_GROUP_GRIPPER: str = "gripper"

def joint_names() -> List[str]:

    return ["joint_1",
            "joint_2",
            "joint_3",
            "joint_4",]

def base_link_name() -> str:
    return "base_link"


def end_effector_name() -> str:
    return "gripper_eef"
