from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    pkg_share = get_package_share_directory('ti5_t170_description')

    xacro_file = os.path.join(
        pkg_share,
        'urdf',
        'T170-V2.1-A0.urdf.xacro'
    )

    pose = LaunchConfiguration('pose')

    # 展開 xacro（Gazebo 一定要開 ros2_control）；pose:=t|q 見 urdf/pose/
    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_file,
            ' use_ros2_control:=true',
            ' use_gazebo:=true',
            ' pose:=', pose,
        ]),
        value_type=str,
    )

    gz_sim_share = get_package_share_directory('ros_gz_sim')

    return LaunchDescription([

        DeclareLaunchArgument(
            'pose',
            default_value='t',
            description='Arm kinematic zero: t (T-pose) or q (Q-pose)',
        ),

        # 讓 Gazebo 找得到 mesh
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=pkg_share
        ),

        # 啟動 Gazebo Harmonic
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': '-v 4 empty.sdf'
            }.items()
        ),

        # robot_state_publisher（一定要先起）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description
            }]
        ),

        # 把機器人丟進 Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'HU_D04_01',
                '-topic', '/robot_description',
                '-x', '0',
                '-y', '0',
                '-z', '1.0'
            ],
            output='screen'
        ),
    ])
