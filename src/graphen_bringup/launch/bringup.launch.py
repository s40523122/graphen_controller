import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler,
    SetLaunchConfiguration)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    description_share = get_package_share_directory('ti5_170a_graphen')
    bringup_share = get_package_share_directory('graphen_bringup')

    xacro_file = os.path.join(description_share, 'urdf', 'ti5_170a_graphen.urdf.xacro')
    config_dir = os.path.join(bringup_share, 'config')
    # 與硬體無關的設定（模式、switch 接線）；硬體相關的設定依 hardware:= 疊加 hardware/*.yaml、imu/*.yaml
    controllers_yaml_file = os.path.join(config_dir, 'controllers.yaml')
    # 實機本體（ti5_hardware）使用的 CAN 設定與初始位置
    can_config_file = os.path.join(bringup_share, 'config', 'can_driver_config.json')
    real_initial_positions_file = os.path.join(
        bringup_share, 'config', 'real_initial_positions.yaml')

    hardware = LaunchConfiguration('hardware')
    hw_read_timeout_ms = LaunchConfiguration('hw_read_timeout_ms')
    max_command_velocity = LaunchConfiguration('max_command_velocity')
    max_missed_cycles = LaunchConfiguration('max_missed_cycles')
    use_hands = LaunchConfiguration('use_hands')
    left_hand_port = LaunchConfiguration('left_hand_port')
    right_hand_port = LaunchConfiguration('right_hand_port')
    hand_baudrate = LaunchConfiguration('hand_baudrate')
    use_mock_imu = LaunchConfiguration('use_mock_imu')
    imu_port = LaunchConfiguration('imu_port')
    imu_baudrate = LaunchConfiguration('imu_baudrate')
    mujoco_fixed_base = LaunchConfiguration('mujoco_fixed_base')
    mujoco_model = LaunchConfiguration('mujoco_model')
    mujoco_viewer_port = LaunchConfiguration('mujoco_viewer_port')
    mujoco_robot = LaunchConfiguration('mujoco_robot')
    scenario_params = LaunchConfiguration('scenario_params')
    # 由 resolve_mujoco_model 依 mujoco_robot 設定
    mujoco_missing_joints = LaunchConfiguration('mujoco_missing_joints')
    mujoco_initial_base_height = LaunchConfiguration('mujoco_initial_base_height')

    declare_args = [
        DeclareLaunchArgument(
            'hardware', default_value='mock',
            choices=['mock', 'mujoco', 'mujoco_pd', 'real', 'real_pd'],
            description='mock: mock_components/GenericSystem + PositionController; '
                        'mujoco: MuJoCo position actuators + PositionController; '
                        'mujoco_pd: MuJoCo torque actuators + PdController; '
                        'real: T170 joints via ti5_hardware (CAN position mode) + PositionController; '
                        'real_pd: T170 joints via ti5_hardware (CAN current mode) + PdController'),
        DeclareLaunchArgument(
            'hw_read_timeout_ms', default_value='',
            description='CAN read timeout of ti5_hardware (hardware:=real / real_pd); '
                        'empty selects 20 for real and 2 for real_pd (must fit in the 5 ms cycle)'),
        DeclareLaunchArgument(
            'max_missed_cycles', default_value='10',
            description='hardware:=real_pd only. Consecutive cycles with missing CAN feedback '
                        'tolerated before all motors fall back to position hold '
                        '(default 10 gives ~50 ms transition margin after entering current mode)'),
        DeclareLaunchArgument(
            'max_command_velocity', default_value='0.5',
            description='Max command velocity [rad/s] of ti5_hardware, also limits the move '
                        'to real_initial_positions at startup (0 disables)'),
        DeclareLaunchArgument(
            'use_hands', default_value='false',
            description='Enable dexterous hands ros2_control (real only with hardware:=real / real_pd)'),
        DeclareLaunchArgument(
            'left_hand_port', default_value='/dev/ttyUSB1',
            description='Serial port of the real left hand'),
        DeclareLaunchArgument(
            'right_hand_port', default_value='/dev/ttyUSB0',
            description='Serial port of the real right hand'),
        DeclareLaunchArgument(
            'hand_baudrate', default_value='115200',
            description='Baudrate of the real hands'),
        DeclareLaunchArgument(
            'use_mock_imu', default_value='true',
            description='hardware:=mock / real / real_pd only. true: mock imu_sensor/* written via '
                        'sensor_forwarder; false: real FSS-IMU16460-C via imu16460_hardware'),
        DeclareLaunchArgument(
            'imu_port', default_value='/dev/ttyTHS1',
            description='Serial port of the real IMU (use_mock_imu:=false)'),
        DeclareLaunchArgument(
            'imu_baudrate', default_value='921600',
            description='Baudrate of the real IMU (use_mock_imu:=false)'),
        DeclareLaunchArgument(
            'mujoco_fixed_base', default_value='true',
            description='true: base fixed in the air (like hanging); false: free-standing. '
                        'fb70 selects t170_fixed.xml; models with a free joint (t170a) are pinned'),
        DeclareLaunchArgument(
            'mujoco_robot', default_value='fb70',
            choices=['fb70', 't170a', 'ti5_t170'],
            description='hardware:=mujoco / mujoco_pd only. fb70: graphen_mujoco/models '
                        '(FullBodyT70 export); t170a: t170a_description/mjcf/t170a.xml '
                        '(motor actuators, hardware:=mujoco_pd only, no neck joints); '
                        'ti5_t170: ti5_t170_description/mjcf/ti5_t170.xml '
                        '(T170-V2.1-A0 T-pose, motor actuators, hardware:=mujoco_pd only)'),
        DeclareLaunchArgument(
            'mujoco_model', default_value='',
            description='MJCF path; empty selects the model of mujoco_robot'),
        DeclareLaunchArgument(
            'scenario_params', default_value='',
            description='Parameter file loaded after all others; used by scenario launch files '
                        '(e.g. sim_rl.launch.py) to override the mode settings'),
        DeclareLaunchArgument(
            'mujoco_viewer_port', default_value='9870',
            description='UDP port of graphen_mujoco/tools/mujoco_viewer.py (0 disables)'),
    ]

    def is_mujoco(context):
        return hardware.perform(context).startswith('mujoco')

    # 依 mujoco_robot 選擇模型與 MujocoSystem 的設定。只在 MuJoCo 時查詢模型套件，
    # 沒有建置 graphen_mujoco / 模型套件時其他硬體仍可使用
    # 力矩馬達模型（只能用 PD）：(套件, MJCF 相對路徑, 缺少的 joint 處理, 浮動基座初始高度)。
    # 初始高度為訓練初始姿態（微彎膝）下腳底貼地的高度
    motor_models = {
        # 模型沒有頸部，三軸改為虛擬關節
        't170a': ('t170a_description', 'mjcf/t170a.xml', 'virtual', '0.894'),
        # 腳底貼地為 1.065 m（ti5_t170_description/scripts/make_mjcf.py），留 5 mm 落地
        'ti5_t170': ('ti5_t170_description', 'mjcf/ti5_t170.xml', 'error', '1.07'),
    }

    def resolve_mujoco_model(context):
        actions = [SetLaunchConfiguration('mujoco_missing_joints', 'error'),
                   SetLaunchConfiguration('mujoco_initial_base_height', '1.07')]
        if not is_mujoco(context):
            return actions
        robot = mujoco_robot.perform(context)
        if robot in motor_models:
            if hardware.perform(context) != 'mujoco_pd':
                raise RuntimeError(f'mujoco_robot:={robot} needs hardware:=mujoco_pd '
                                   '(the model has motor actuators)')
            package, relpath, missing_joints, base_height = motor_models[robot]
            actions = [SetLaunchConfiguration('mujoco_missing_joints', missing_joints),
                       SetLaunchConfiguration('mujoco_initial_base_height', base_height)]
            path = os.path.join(get_package_share_directory(package), relpath)
        else:
            name = 't170_fixed.xml' if IfCondition(mujoco_fixed_base).evaluate(context) \
                else 't170_free.xml'
            path = os.path.join(get_package_share_directory('graphen_mujoco'), 'models', name)
        if not mujoco_model.perform(context):
            actions.append(SetLaunchConfiguration('mujoco_model', path))
        return actions

    set_mujoco_model = OpaqueFunction(function=resolve_mujoco_model)

    # ti5_hardware 的 CAN 讀取逾時：位置模式（100 Hz）沿用 20 ms；電流模式在 200 Hz 迴圈內讀取，
    # 必須遠小於 5 ms 的週期。使用者有指定時不覆寫
    def resolve_hw_read_timeout(context):
        if hw_read_timeout_ms.perform(context):
            return []
        timeout = '2' if hardware.perform(context) == 'real_pd' else '20'
        return [SetLaunchConfiguration('hw_read_timeout_ms', timeout)]

    set_hw_read_timeout = OpaqueFunction(function=resolve_hw_read_timeout)

    robot_description_raw = Command([
        'xacro ', xacro_file,
        ' hardware:=', hardware,
        ' can_config_path:=', can_config_file,
        ' real_initial_positions_file:=', real_initial_positions_file,
        ' hw_read_timeout_ms:=', hw_read_timeout_ms,
        ' max_command_velocity:=', max_command_velocity,
        ' max_missed_cycles:=', max_missed_cycles,
        ' use_hands:=', use_hands,
        ' left_hand_port:=', left_hand_port,
        ' right_hand_port:=', right_hand_port,
        ' hand_baudrate:=', hand_baudrate,
        ' use_mock_imu:=', use_mock_imu,
        ' imu_port:=', imu_port,
        ' imu_baudrate:=', imu_baudrate,
        ' mujoco_model:="', mujoco_model, '"',
        ' mujoco_viewer_port:=', mujoco_viewer_port,
        ' mujoco_fixed_base:=', mujoco_fixed_base,
        ' mujoco_missing_joints:=', mujoco_missing_joints,
        ' mujoco_initial_base_height:=', mujoco_initial_base_height,
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
    #    共用設定 →（mujoco_pd / real_pd）hardware/pd_common.yaml → hardware/<hardware>.yaml
    #    →（mujoco_pd）hardware/mujoco_pd_<mujoco_robot>.yaml →（mock / real / real_pd）imu/<mock|imu16460>.yaml
    #    →（情境 launch）scenario_params，後載入的覆蓋先載入的。
    #    參數檔無法用 condition 切換，所以在 OpaqueFunction 中組合
    def create_controller_manager(context):
        hw = hardware.perform(context)
        files = [controllers_yaml_file]
        if hw in ('mujoco_pd', 'real_pd'):
            # PD 子群組與 switch 的速度前饋 / kp / kd 接線，模擬與實機共用
            files.append(os.path.join(config_dir, 'hardware', 'pd_common.yaml'))
        files.append(os.path.join(config_dir, 'hardware', f'{hw}.yaml'))
        if hw == 'mujoco_pd':
            # 預設增益依模型：顯式 PD 的增益要配合模型的慣量，否則會振盪
            files.append(os.path.join(
                config_dir, 'hardware', f'mujoco_pd_{mujoco_robot.perform(context)}.yaml'))
        if not is_mujoco(context):
            imu = 'mock' if IfCondition(use_mock_imu).evaluate(context) else 'imu16460'
            files.append(os.path.join(config_dir, 'imu', f'{imu}.yaml'))
        # 情境 launch 的覆寫（例如一開始就進入 RL 模式），不改本機 controllers.yaml
        if scenario_params.perform(context):
            files.append(scenario_params.perform(context))
        def shown(path):
            return os.path.relpath(path, config_dir) if path.startswith(config_dir) else path
        return [LogInfo(msg='controller parameter files: ' + ', '.join(
            shown(f) for f in files)), Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description] + files,
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

    # 3-1. 模擬 IMU 資料寫入 (imu_sensor/*)，只在 mock IMU 時啟動（MuJoCo 時 IMU 由 MujocoSystem 提供）
    mock_imu_active = PythonExpression(
        ["'", use_mock_imu, "'.lower() == 'true' and not '", hardware, "'.startswith('mujoco')"])
    spawn_sensor_forwarder = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['sensor_forwarder'],
        output='screen',
        condition=IfCondition(mock_imu_active)
    )

    # 3-2. 實機或 MuJoCo 的 IMU 發布 sensor_msgs/Imu（/imu_sensor_broadcaster/imu）
    spawn_imu_sensor_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster'],
        output='screen',
        condition=UnlessCondition(mock_imu_active)
    )

    # 4. 下游子群組（32 軸，依部位分組），依序啟動
    sub_group_controllers = [
        'left_arm_group_controller',
        'right_arm_group_controller',
        'waist_group_controller',
        'head_group_controller',
        'left_leg_group_controller',
        'right_leg_group_controller',
    ]
    spawn_sub_group_controllers = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name],
            output='screen'
        )
        for name in sub_group_controllers
    ]

    # 5. 上游 Switch Group
    spawn_switch_group_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['switch_group_controller'],
        output='screen'
    )

    # ===== 嚴格順序控制：確保廣播器與所有下游 Controller 完全就緒後，才啟動頂層 SwitchGroupController =====
    # Broadcaster -> 子群組 1 -> ... -> 子群組 N -> switch_group_controller
    spawn_chain = [spawn_joint_state_broadcaster] + spawn_sub_group_controllers + [
        spawn_switch_group_controller]
    delayed_spawners = [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=previous,
                on_exit=[following]
            )
        )
        for previous, following in zip(spawn_chain[:-1], spawn_chain[1:])
    ]

    return LaunchDescription(declare_args + [
        set_mujoco_model,  # 必須在使用 robot_description 的 node 之前
        set_hw_read_timeout,
        node_robot_state_publisher,
        node_controller_manager,
        spawn_joint_state_broadcaster,
        spawn_sensor_forwarder,
        spawn_imu_sensor_broadcaster,
    ] + delayed_spawners)
