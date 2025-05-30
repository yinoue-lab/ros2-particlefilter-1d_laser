from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
import os
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction, DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
import subprocess

package_name = 'simulation_launcher'

def generate_launch_description():
    subprocess.run(["blackbox_archive"]) 
    subprocess.run(["blackbox_create"]) 
    
    headless = LaunchConfiguration('headless')

    initial_pos_x = LaunchConfiguration('initial_pos_x')
    initial_pos_y = LaunchConfiguration('initial_pos_y')
    initial_pos_yaw = LaunchConfiguration('initial_pos_yaw')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)'))

    ld.add_action(DeclareLaunchArgument(
        'initial_pos_x',
        default_value='0.0',
        description='Initial robot pose (x-axis)'))

    ld.add_action(DeclareLaunchArgument(
        'initial_pos_y',
        default_value='0.0',
        description='Initial robot pose (y-axis)'))

    ld.add_action(DeclareLaunchArgument(
        'initial_pos_yaw',
        default_value='0.0',
        description='Initial robot pose (degree)'))

    ld.add_action(
        SetParameter(name='use_sim_time', value=True)
    )
    
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('simulation_launcher'), 'launch', 'aws_robomaker', 'no_roof_small_warehouse.launch.py')
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
                'only_broadcaster': 'False',
                'x_pose': initial_pos_x,
                'y_pose': initial_pos_y,
                'yaw_pose': initial_pos_yaw,
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
                'initial_pos_x' : initial_pos_x,
                'initial_pos_y' : initial_pos_y,
                'initial_pos_yaw' : initial_pos_yaw,
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