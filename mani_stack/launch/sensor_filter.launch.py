
from launch.actions import ExecuteProcess
from launch import LaunchDescription
def generate_launch_description():

    
    
    Start_perceiption = ExecuteProcess( 
                                       cmd=[[
            'ros2 run mani_stack perception.py',
        ]],
        shell=True
    )
    Start_Imu = ExecuteProcess(
        cmd=[[
            'ros2 run mani_stack duplicateImu.py',
        ]],
        shell=True
    )
    Start_ultraFilter = ExecuteProcess(
        cmd=[[
            'ros2 run mani_stack ultraFilter.py',
        ]],
        shell=True
    )
    
    return LaunchDescription([
    Start_perceiption,
    Start_Imu,
    Start_ultraFilter
    ])