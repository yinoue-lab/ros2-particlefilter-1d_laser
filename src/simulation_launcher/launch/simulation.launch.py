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
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')
    
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
            )
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(package_name), 'launch', 'robot_state_publisher.launch.py')
            )
        )
    )

    # ld.add_action(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             os.path.join(get_package_share_directory('aws-robomaker-small-warehouse-world'), 'launch', 'no_roof_small_warehouse.launch.launch.py')
    #         )
    #     )
    # )

    return ld