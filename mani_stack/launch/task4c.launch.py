
from launch.actions import ExecuteProcess
from launch import LaunchDescription
def generate_launch_description():

    start_docking = ExecuteProcess(
        cmd=[[
            'ros2 run ebot_docking docking_Simulation_boilerplate.py',
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
    # Start_arm_manipulation = ExecuteProcess(
    #     cmd=[[
    #         'ros2 run mani_stack manipulation.py',
    #     ]],
    #     shell=True
    # )
    return LaunchDescription([
    start_docking,
    start_yaml_controller,
    start_nav2,
    Start_perceiption,
    # Start_arm_manipulation
    ])