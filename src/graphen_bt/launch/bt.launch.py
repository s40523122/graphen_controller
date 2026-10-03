import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('graphen_bt')

    tree_file_arg = DeclareLaunchArgument(
        'tree_file',
        default_value=os.path.join(pkg_share, 'trees', 'demo.xml'),
        description='Behavior tree XML file to execute')
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'bt_runner.yaml'),
        description='Parameter file for bt_runner')

    bt_runner = Node(
        package='graphen_bt',
        executable='bt_runner',
        name='bt_runner',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'tree_file': LaunchConfiguration('tree_file')},
        ])

    return LaunchDescription([
        tree_file_arg,
        params_file_arg,
        bt_runner,
    ])
