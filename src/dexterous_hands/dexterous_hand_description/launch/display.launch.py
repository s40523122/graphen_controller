import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('dexterous_hand_description')
    xacro_file = os.path.join(pkg, 'urdf', 'dual_hands.urdf.xacro')

    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([
        # robot_state_publisher：解析 URDF 並發布 TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # joint_state_publisher_gui：提供滑桿控制關節角度
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        # rviz2：視覺化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg, 'rviz', 'dual_hands.rviz')],
        ),
    ])