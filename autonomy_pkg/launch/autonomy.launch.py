import os
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():

    ld = LaunchDescription()

    terminal_command = 'gnome-terminal -- bash -c '

    ros_runs = [
        '"ros2 run autonomy_pkg autonomy_ptp_planner_node"',
        '"ros2 run autonomy_pkg autonomy_text_interface_node"',

    ]
    for run in ros_runs:
        ld.add_action(
            ExecuteProcess(
                cmd=[terminal_command + run],
                shell=True,
            )
        )

    return ld