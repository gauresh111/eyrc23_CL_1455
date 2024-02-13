import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path


def load_file(file_path):
    """Load the contents of a file into a string"""
    try:
        with open(file_path, "r") as file:
            return file.read()
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(file_path):
    """Load a yaml file into a dictionary"""
    try:
        with open(file_path, "r") as file:
            return yaml.safe_load(file)
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def run_xacro(xacro_file):
    """Run xacro and output a file in the same directory with the same name, w/o a .xacro suffix"""
    urdf_file, ext = os.path.splitext(xacro_file)
    if ext != ".xacro":
        raise RuntimeError(
            f"Input file to xacro must have a .xacro extension, got {xacro_file}"
        )
    os.system(f"xacro {xacro_file} -o {urdf_file}")
    return urdf_file


def generate_launch_description():
    moveit_config_folder_name = "ur5_moveit"

    xacro_file = get_package_file(moveit_config_folder_name, "config/ur5.urdf.xacro")
    urdf_file = run_xacro(xacro_file)
    robot_description_arm = load_file(urdf_file)

    srdf_file = get_package_file(moveit_config_folder_name, "config/ur5.srdf")
    kinematics_file = get_package_file(
        moveit_config_folder_name, "config/kinematics.yaml"
    )
    ompl_config_file = get_package_file(
        moveit_config_folder_name, "config/ompl_planning.yaml"
    )
    moveit_controllers_file = get_package_file(
        moveit_config_folder_name, "config/moveit_controllers.yaml"
    )
    moveit_servo_file = get_package_file(
        moveit_config_folder_name, "config/ur_servo.yaml"
    )
    ros_controllers_file = get_package_file("ur5_moveit", "config/ros_controllers.yaml")

    robot_description_semantic = load_file(srdf_file)
    kinematics_config = load_yaml(kinematics_file)
    ompl_config = load_yaml(ompl_config_file)

    moveit_controllers = {
        "moveit_simple_controller_manager": load_yaml(moveit_controllers_file),
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    planning_scene_monitor_config = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Command-line arguments
    db_arg = DeclareLaunchArgument(
        "db", default_value="False", description="Database flag"
    )

    # moveit_config = (
    #     MoveItConfigsBuilder("moveit_resources_fanuc")
    #     .robot_description(file_path="config/fanuc.urdf.xacro")
    #     .robot_description_semantic(file_path="config/fanuc.srdf")
    #     .trajectory_execution(file_path="config/moveit_controllers.yaml")
    #     .to_moveit_configs()
    # )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_arm,
                "robot_description_semantic": robot_description_semantic,
                "robot_description_kinematics": kinematics_config,
                "ompl": ompl_config,
                "planning_pipelines": ["ompl"],
            },
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor_config,
            {"use_sim_time": True},
        ],
    )

    # RViz
    # rviz_base = os.path.join(
    #     get_package_share_directory("moveit_resources_fanuc_moveit_config"), "launch"
    # )
    # rviz_full_config = os.path.join(rviz_base, "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        # arguments=["-d", rviz_full_config],
        parameters=[
            {
                "robot_description": robot_description_arm,
                "robot_description_semantic": robot_description_semantic,
                "robot_description_kinematics": kinematics_config,
            }
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{'robot_description': robot_description_arm}],
        remappings=[('robot_description', 'robot_description_ur5')]
    )

    # ros2_control using FakeSystem as hardware
    # ros2_controllers_path = os.path.join(
    #     get_package_share_directory("moveit_resources_fanuc_moveit_config"),
    #     "config",
    #     "ros2_controllers.yaml",
    # )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters= [
            {'robot_description': robot_description_arm},
            ros_controllers_file
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Warehouse mongodb server
    db_config = LaunchConfiguration("db")
    mongodb_server_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=[
            {"warehouse_port": 33829},
            {"warehouse_host": "localhost"},
            {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
        ],
        output="screen",
        condition=IfCondition(db_config),
    )

    return LaunchDescription(
        [
            db_arg,
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
            mongodb_server_node,
            joint_state_broadcaster_spawner,
            joint_trajectory_controller_spawner,
        ]
    )
