from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import os

def generate_launch_description():
    pkg_name = 'ti5_170a_graphen'

    pkg_path = get_package_share_directory(pkg_name)

    xacro_file = os.path.join(
        pkg_path,
        'urdf',
        'robot.urdf.xacro'
    )
    
    rviz_config = os.path.join(
        pkg_path, 
        'rviz', 
        't170_v2_1_a0_11_14.rviz'
        )

    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={
            'package_name': pkg_name,
            #'use_gazebo': 'false'
        }
    )

    robot_description = robot_description_config.toxml()

    return LaunchDescription([

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]
        )
    ])
