import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOT_MODEL = "turtlebot3_waffle_1d_laser"
package_name = 'simulation_launcher'

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory(package_name),
        'urdf', ROBOT_MODEL, 'robot.urdf'
    )

    # Launch configuration variables for simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    yaw_pose = LaunchConfiguration('yaw_pose', default='0.0')  # YAW角（ラジアン単位）

    # Declare the launch arguments
    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='初期x座標を指定します'
    )
    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='初期y座標を指定します'
    )
    declare_yaw_pose_cmd = DeclareLaunchArgument(
        'yaw_pose', default_value='0.0',
        description='初期yaw角（ラジアン単位）を指定します'
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-entity', 'my_robot',
            '-file', urdf_file,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
            '-Y', yaw_pose  # yaw角を指定する引数
        ]
    )

    return LaunchDescription([
        declare_x_pose_cmd,
        declare_y_pose_cmd,
        declare_yaw_pose_cmd,
        spawn_entity
    ])
