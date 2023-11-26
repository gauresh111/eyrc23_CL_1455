import os
import yaml
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction
import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, ExecuteProcess


def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def load_file(file_path):
    """Load the contents of a file into a string"""
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(file_path):
    """Load a yaml file into a dictionary"""
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def run_xacro(xacro_file):
    """Run xacro and output a file in the same directory with the same name, w/o a .xacro suffix"""
    urdf_file, ext = os.path.splitext(xacro_file)
    if ext != '.xacro':
        raise RuntimeError(f'Input file to xacro must have a .xacro extension, got {xacro_file}')
    os.system(f'xacro {xacro_file} -o {urdf_file}')
    return urdf_file
def generate_launch_description():
    # ld = LaunchDescription()
    # start_perception = Node(
    #     package="mani_stack",
    #     executable="task1ab-perception.py"#"task1ab-perception.py",
    # )
    # ld.add_action(start_perception)
    # return ld
    start_perception = Node(
    package='mani_stack',
    executable='task1ab-perception.py',
 
    )
    start_docking = Node(
       package='ebot_docking',
    executable='ebot_docking_boilerplate.py',

    )
    start_navigation = Node(
    package='ebot_docking',
    executable='task2b.py',
    )
 
    return LaunchDescription([
    start_perception,
     start_docking,
     start_navigation
    ])