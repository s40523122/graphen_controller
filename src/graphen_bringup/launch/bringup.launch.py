import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():

    description_share = get_package_share_directory('graphen_description')
    bringup_share = get_package_share_directory('graphen_bringup')

    xacro_file = os.path.join(description_share, 'urdf', 'robot.urdf.xacro')
    controllers_yaml_file = os.path.join(bringup_share, 'config', 'controllers.yaml')

    robot_description_raw = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': robot_description_raw}

    # 1. State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 2. Controller Manager
    node_controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_yaml_file],
        output='screen'
    )

    # 3. Joint State Broadcaster
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # 3-1. 模擬 sensor 資料寫入 (imu/raw_data)
    spawn_sensor_forwarder = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['sensor_forwarder'],
        output='screen'
    )

    # 4. 下游 Arm
    spawn_arm_group_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_group_controller'],
        output='screen'
    )

    # 5. 下游 Gripper
    spawn_gripper_group_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_group_controller'],
        output='screen'
    )

    # 6. 上游 Switch Group
    spawn_switch_group_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['switch_group_controller'],
        output='screen'
    )

    # ===== 嚴格順序控制：確保廣播器與所有下游 Controller 完全就緒後，才啟動頂層 SwitchGroupController =====

    # Broadcaster 完成 -> 啟動 arm
    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_arm_group_controller]
        )
    )

    # Arm 完成 -> 啟動 gripper
    delay_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_arm_group_controller,
            on_exit=[spawn_gripper_group_controller]
        )
    )

    # Gripper 完成 -> 啟動頂層 switch_group_controller
    delay_switch_group_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_gripper_group_controller,
            on_exit=[spawn_switch_group_controller]
        )
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_controller_manager,
        spawn_joint_state_broadcaster,
        spawn_sensor_forwarder,
        delay_arm_controller,
        delay_gripper_controller,
        delay_switch_group_controller
    ])
