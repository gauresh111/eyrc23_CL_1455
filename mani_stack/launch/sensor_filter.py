
from launch.actions import ExecuteProcess
from launch import LaunchDescription
def generate_launch_description():

    start_docking = ExecuteProcess(
        cmd=[[
            'ros2 run ebot_docking docking_Hardware_boilerplate.py',
        ]],
        shell=True
    )
    start_yaml_controller = ExecuteProcess(
        cmd=[[
            'ros2 run ebot_docking ebot_nav2_yaml.py',
        ]],
        shell=True
    )
    start_nav2 = ExecuteProcess(
        cmd=[[
            'ros2 run ebot_nav2 nav2_cmd.py',
        ]],
        shell=True
    )
    # Start_exitNav = ExecuteProcess(
    #     cmd=[[
    #         'ros2 run mani_stack exitNav2.py',
    #     ]],
    #     shell=True
    # )
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
    # Start_arm_manipulation = ExecuteProcess(
    #     cmd=[[
    #         'ros2 run mani_stack manipulation.py',
    #     ]],
    #     shell=True
    # )
    Start_duplication = ExecuteProcess(
        cmd=[[
            'ros2 run ebot_docking duplicate.py',
        ]],
        shell=True
    )
    return LaunchDescription([
    Start_perceiption,
    Start_Imu,
    Start_ultraFilter
    ])