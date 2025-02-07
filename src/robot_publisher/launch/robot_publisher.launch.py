from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory('robot_publisher'))
    xacro_file = os.path.join(pkg_path,
                            'xacro',
                            'robot.xacro')
    robot_description = {'robot_description':  xacro.process_file(xacro_file).toxml()}

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace="r2",
            parameters=[robot_description],
            respawn = True
        ),
    ])
