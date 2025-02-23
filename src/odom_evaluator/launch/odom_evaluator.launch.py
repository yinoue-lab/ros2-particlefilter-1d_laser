from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

package_name = 'odom_evaluator'

def generate_launch_description():
    true_odom_topic = LaunchConfiguration('true_odom_topic')
    est_odom_topic = LaunchConfiguration('est_odom_topic')
    eval_mean_odom_topic = LaunchConfiguration('eval_mean_odom_topic')
    node_name = LaunchConfiguration('node_name')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'true_odom_topic',
        default_value='true_odom',
        description='true odom topic name'))
    
    ld.add_action(DeclareLaunchArgument(
        'est_odom_topic',
        default_value='est_odom',
        description='estimation odom result topic name'))
    
    ld.add_action(DeclareLaunchArgument(
        'eval_mean_odom_topic',
        default_value='eval_mean_odom',
        description='result of evaluation odom topic name'))

    ld.add_action(DeclareLaunchArgument(
        'node_name',
        default_value='odom_evaluator',
        description='node name'))

    ld.add_action(Node(
        package=package_name,
        executable="odom_evaluator",
        name=node_name,
        namespace='',
        remappings=[
                ('true_odom', true_odom_topic),
                ('est_odom', est_odom_topic),
                ('eval_mean_odom', eval_mean_odom_topic)
            ]
    ))

    return ld