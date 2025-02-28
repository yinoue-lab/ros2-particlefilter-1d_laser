import os
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOT_MODEL = "turtlebot3_waffle_1d_laser"
package_name = 'simulation_launcher'

def generate_launch_description():

    declare_only_broadcaster_cmd = DeclareLaunchArgument(
        'only_broadcaster',
        default_value='False',
        description='Switch enable spawn.py.'
    )

    # x, y, yawのoffset（初期位置）のLaunch引数を宣言
    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='初期x座標 / x offset'
    )
    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='初期y座標 / y offset'
    )
    declare_yaw_pose_cmd = DeclareLaunchArgument(
        'yaw_pose',
        default_value='0.0',
        description='初期yaw角（ラジアン単位） / yaw offset'
    )

    # map_offset
    declare_map_offset_x_cmd = DeclareLaunchArgument(
        'map_offset_x',
        default_value='3.0',
        description='mapのx座標オフセット'
    )
    declare_map_offset_y_cmd = DeclareLaunchArgument(
        'map_offset_y',
        default_value='2.2',
        description='mapのy座標オフセット'
    )
    declare_map_offset_yaw_cmd = DeclareLaunchArgument(
        'map_offset_yaw',
        default_value='-87.7',
        description='mapのyaw座標オフセット'
    )

    def launch_setup(context, *args, **kwargs):
        only_broadcaster_val = LaunchConfiguration('only_broadcaster').perform(context)

        # Launch引数から値を取得
        x_offset_val = LaunchConfiguration('x_pose').perform(context)
        y_offset_val = LaunchConfiguration('y_pose').perform(context)
        yaw_offset_val = LaunchConfiguration('yaw_pose').perform(context)

        # map_offset
        map_offset_x_val = LaunchConfiguration('map_offset_x').perform(context)
        map_offset_y_val = LaunchConfiguration('map_offset_y').perform(context)
        map_offset_yaw_val = LaunchConfiguration('map_offset_yaw').perform(context)

        x_offset_float = float(x_offset_val) + float(map_offset_x_val)
        y_offset_float = float(y_offset_val) + float(map_offset_y_val)
        yaw_offset_float = float(yaw_offset_val) + (float(map_offset_yaw_val) * math.pi / 180.0)

        urdf_file = os.path.join(
            get_package_share_directory(package_name),
            'urdf', ROBOT_MODEL, 'robot.urdf'
        )

        ld_node_list = []

        if only_broadcaster_val == 'False':
            ld_node_list.append(Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_robot',
                output='screen',
                arguments=[
                    '-entity', 'waffle_1d',
                    '-robot_namespace', 'waffle_1d',
                    '-file', urdf_file,
                    '-reference_frame', 'map',
                    '-x', str(x_offset_float),
                    '-y', str(y_offset_float),
                    '-z', '0.01',
                    '-Y', str(yaw_offset_float)  # yaw角を指定する引数
                ]
            ))

        ld_node_list.append(Node(
            package='sim_lcmcl',
            executable='tf_broadcaster',
            name='tf_broadcaster',
            parameters=[{
                'odom_topic': '/waffle_1d/gazebo_position',
                'offset_topic': '/waffle_1d/true_position',
                'x_offset': -x_offset_float,
                'y_offset': -y_offset_float,
                'yaw_offset': -yaw_offset_float,
            }]
        ))

        return ld_node_list

    return LaunchDescription([
        declare_x_pose_cmd,
        declare_y_pose_cmd,
        declare_yaw_pose_cmd,
        declare_map_offset_x_cmd,
        declare_map_offset_y_cmd,
        declare_map_offset_yaw_cmd,
        OpaqueFunction(function=launch_setup)
    ])
