from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
import os
import datetime

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
import subprocess

package_name = 'simulation_launcher'

def generate_launch_description():
    subprocess.run(["blackbox_archive"]) 
    subprocess.run(["blackbox_create"]) 
    
    base_dir = "/tmp/blackbox_log"
    current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_file_name = "simulation_record_" + current_time
    bag_output_dir = os.path.join(base_dir, bag_file_name)
    
    # 記録したいトピックのリスト
    topics_to_record = [
        "/waffle_1d/back_laser_range",
        "/waffle_1d/cmd_vel",
        "/waffle_1d/front_laser_range",
        "/waffle_1d/gazebo_position",
        "/waffle_1d/imu",
        "/waffle_1d/l_laser_range",
        "/waffle_1d/lb_laser_range",
        "/waffle_1d/lf_laser_range",
        "/waffle_1d/odom",
        "/waffle_1d/r_laser_range",
        "/waffle_1d/rb_laser_range",
        "/waffle_1d/rf_laser_range",
    ]

    headless = LaunchConfiguration('headless')

    ld = LaunchDescription()

    ld.add_action(
        SetParameter(name='use_sim_time', value=True)
    )

    ld.add_action(
        ExecuteProcess(
            cmd=[
                "ros2", "bag", "record",
                "--storage", "mcap",
                "-o", bag_output_dir
            ] + topics_to_record,
            output="screen"
        )
    )
    
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('aws_robomaker_small_warehouse_world'), 'launch', 'no_roof_small_warehouse.launch.py')
            ),
            launch_arguments={'headless': 'False'}.items()
        )
    )
    
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(package_name), 'launch', 'spawn_robot.launch.py')
            ),
            launch_arguments={
                'only_broadcaster': 'False',
                'x_pose': '0',
                'y_pose': '0',
                'yaw_pose': '0',
            }.items()
        )
    )

    ld.add_action(ExecuteProcess(
        cmd=[
            'xterm', '-e',
            'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
            '--ros-args', '--remap', '/cmd_vel:=/waffle_1d/cmd_vel'
        ],
        output='screen'
    ))

    return ld