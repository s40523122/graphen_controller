import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    description_share = get_package_share_directory('graphen_description')
    bringup_share = get_package_share_directory('graphen_bringup')

    xacro_file = os.path.join(description_share, 'urdf', 'robot.urdf.xacro')
    controllers_yaml_file = os.path.join(bringup_share, 'config', 'controllers.yaml')
    # 實機 IMU 時疊加的設定（imu_sensor_broadcaster、switch_group_controller 的 sensor）
    imu_yaml_file = os.path.join(bringup_share, 'config', 'controllers_imu16460.yaml')

    use_mock_imu = LaunchConfiguration('use_mock_imu')
    imu_port = LaunchConfiguration('imu_port')
    imu_baudrate = LaunchConfiguration('imu_baudrate')

    declare_args = [
        DeclareLaunchArgument(
            'use_mock_imu', default_value='true',
            description='true: mock imu/raw_data (sensor_forwarder); '
                        'false: real FSS-IMU16460-C via imu16460_hardware'),
        DeclareLaunchArgument(
            'imu_port', default_value='/dev/ttyTHS1',
            description='Serial port of the real IMU (use_mock_imu:=false)'),
        DeclareLaunchArgument(
            'imu_baudrate', default_value='921600',
            description='Baudrate of the real IMU (use_mock_imu:=false)'),
    ]

    robot_description_raw = Command([
        'xacro ', xacro_file,
        ' use_mock_imu:=', use_mock_imu,
        ' imu_port:=', imu_port,
        ' imu_baudrate:=', imu_baudrate,
    ])
    robot_description = {'robot_description': robot_description_raw}

    # 1. State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 2. Controller Manager
    #    實機 IMU 時多載入 imu_yaml_file；參數檔無法用 condition 切換，所以在 OpaqueFunction 中組合
    def create_controller_manager(context):
        parameters = [robot_description, controllers_yaml_file]
        if not IfCondition(use_mock_imu).evaluate(context):
            parameters.append(imu_yaml_file)
        return [Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=parameters,
            output='screen'
        )]

    node_controller_manager = OpaqueFunction(function=create_controller_manager)

    # 3. Joint State Broadcaster
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # 3-1. 模擬 sensor 資料寫入 (imu/raw_data)，只在 mock IMU 時啟動
    spawn_sensor_forwarder = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['sensor_forwarder'],
        output='screen',
        condition=IfCondition(use_mock_imu)
    )

    # 3-2. 實機 IMU 發布 sensor_msgs/Imu（/imu_sensor_broadcaster/imu），只在實機 IMU 時啟動
    spawn_imu_sensor_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster'],
        output='screen',
        condition=UnlessCondition(use_mock_imu)
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

    return LaunchDescription(declare_args + [
        node_robot_state_publisher,
        node_controller_manager,
        spawn_joint_state_broadcaster,
        spawn_sensor_forwarder,
        spawn_imu_sensor_broadcaster,
        delay_arm_controller,
        delay_gripper_controller,
        delay_switch_group_controller
    ])
