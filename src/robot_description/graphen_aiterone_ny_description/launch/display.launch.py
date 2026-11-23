"""檢視整機模型：robot_state_publisher + joint_state_publisher_gui + RViz。"""

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('graphen_aiterone_ny_description')
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'graphen_aiterone_ny.urdf.xacro'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'display.rviz'])

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
