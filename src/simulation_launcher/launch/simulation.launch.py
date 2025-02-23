from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
import os
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
    
    headless = LaunchConfiguration('headless')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)'))

    ld.add_action(
        SetParameter(name='use_sim_time', value=True)
    )
    
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('aws_robomaker_small_warehouse_world'), 'launch', 'no_roof_small_warehouse.launch.py')
            ),
            launch_arguments={'headless': headless}.items()
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
            ),
            launch_arguments={
                'publish_topic_pf': 'pf_odom',
                'publish_topic_kf': 'kf_odom',
                'subscribe_topic_odom': '/waffle_1d/odom',
                }.items()
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('odom_evaluator'), 'launch', 'odom_evaluator.launch.py')
            ),
            launch_arguments={
                'true_odom_topic': '/waffle_1d/true_position',
                'est_odom_topic': '/localization/pf_odom',
                'eval_mean_odom_topic': 'pf_eval_odom',
                'node_name': 'pf_evaluator'
                }.items()
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('odom_evaluator'), 'launch', 'odom_evaluator.launch.py')
            ),
            launch_arguments={
                'true_odom_topic': '/waffle_1d/true_position',
                'est_odom_topic': '/localization/kf_odom',
                'eval_mean_odom_topic': 'kf_eval_odom',
                'node_name': 'kf_evaluator'
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