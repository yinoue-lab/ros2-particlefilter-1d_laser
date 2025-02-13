from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
import subprocess

package_name = 'simulation_launcher'

def generate_launch_description():
    subprocess.run(["blackbox_create"]) 

    ld = LaunchDescription()
    
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('aws_robomaker_small_warehouse_world'), 'launch', 'no_roof_small_warehouse.launch.py')
            ),
            launch_arguments={'headless': 'True'}.items()
        )
    )
    
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(package_name), 'launch', 'spawn_robot.launch.py')
            ),
            launch_arguments={
                'x_pose': '0',
                'y_pose': '0',
                'yaw_pose': '0',
            }.items()
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(package_name), 'launch', 'robot_state_publisher.launch.py')
            )
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('sim_lcmcl'), 'launch', 'debug_rviz.launch.py')
            )
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('lcmcl'), 'launch', 'lcmcl.launch.py')
            )
        )
    )

    return ld