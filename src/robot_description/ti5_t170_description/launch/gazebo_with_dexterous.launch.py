from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    ti5_pkg_share = get_package_share_directory('ti5_t170_description')
    hand_pkg_share = get_package_share_directory('dexterous_hand_description')

    # ti5 mesh: model://meshes/... → share/ti5_t170_description/meshes/...
    # hand mesh: model://dexterous_hand_description/meshes/... → share/dexterous_hand_description/meshes/...
    gz_resource_path = ti5_pkg_share
    hand_share_root = os.path.dirname(hand_pkg_share)
    if hand_share_root not in gz_resource_path:
        gz_resource_path = f'{gz_resource_path}:{hand_share_root}'
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing_gz_path:
        gz_resource_path = f'{gz_resource_path}:{existing_gz_path}'

    xacro_file = os.path.join(
        ti5_pkg_share,
        'urdf',
        'ti5_with_dexterous_hand_real.urdf.xacro',
    )

    pose = LaunchConfiguration('pose')
    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_file,
            ' use_dexterous_hand:=true',
            ' use_gazebo:=true',
            ' pose:=', pose,
        ]),
        value_type=str,
    )

    gz_sim_share = get_package_share_directory('ros_gz_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([

        DeclareLaunchArgument(
            'pose',
            default_value='q',
            description='Arm kinematic zero: t (T-pose) or q (Q-pose)',
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true',
        ),

        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=gz_resource_path,
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': '-v 4 empty.sdf',
            }.items(),
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'ti5_with_dexterous_hands',
                '-topic', '/robot_description',
                '-x', '0',
                '-y', '0',
                '-z', '1.3',
            ],
            output='screen',
        ),
    ])
