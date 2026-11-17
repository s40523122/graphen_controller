import os
from collections import namedtuple

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler,
    SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    description_share = get_package_share_directory('ti5_170a_graphen')
    bringup_share = get_package_share_directory('graphen_bringup')

    xacro_file = os.path.join(description_share, 'urdf', 'ti5_170a_graphen.urdf.xacro')
    config_dir = os.path.join(bringup_share, 'config')
    # 與硬體無關的設定（模式、switch 接線）；控制方式依 hardware:= 疊加 hardware/<position|pd>_common.yaml、
    # IMU 疊加 imu/*.yaml，PD 增益依 robot:= 取自模型套件
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
    robot = LaunchConfiguration('robot')
    scenario_params = LaunchConfiguration('scenario_params')
    # 由 resolve_mujoco_model 依 robot 設定
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
                        'fb70 selects t170_fixed.xml; models with a free joint (ti5_t170) are pinned'),
        DeclareLaunchArgument(
            'robot', default_value='ti5_170a',
            choices=['ti5_170a', 'ti5_t170', 'fb70'],
            description='Robot model, selects the URDF body, the MJCF (MuJoCo) and the PD gains '
                        '(mujoco_pd / real_pd), for every hardware. ti5_t170: '
                        'ti5_t170_description (T170-V2.1-A0, motor actuators); ti5_170a: '
                        'ti5_170a_graphen (own URDF, generated MJCF with motor actuators); fb70: '
                        'fb70_description (FullBodyT70 export, position servos, URDF body of '
                        'ti5_170a_graphen). hardware:=mujoco needs fb70'),
        DeclareLaunchArgument(
            'mujoco_model', default_value='',
            description='MJCF path; empty selects the model of robot'),
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

    # 各模型（robot:=）的資訊。模型套件提供 MJCF（MuJoCo）與 config/joint_pd_gains.yaml（PD 增益）；
    # URDF 本體由 ti5_170a_graphen.urdf.xacro 依 robot 選擇。只在需要時查詢模型套件
    #   mjcf：MJCF 相對路徑，None 時依 mujoco_fixed_base 選 mjcf/t170_fixed.xml / t170_free.xml
    #   servo：True 為位置伺服（mujoco / mujoco_pd 都可用），False 為力矩馬達（只能 mujoco_pd）
    #   missing_joints / base_height：MujocoSystem 的設定；初始高度為訓練初始姿態下腳底貼地的高度
    Robot = namedtuple('Robot', 'package mjcf servo missing_joints base_height')
    robots = {
        # 腳底貼地為 1.065 m（ti5_t170_description/scripts/make_mjcf.py），留 5 mm 落地
        'ti5_t170': Robot('ti5_t170_description', 'mjcf/ti5_t170.xml', False, 'error', '1.07'),
        # ti5_170a_graphen/scripts/make_mjcf.py 印出的高度（直膝下腳底最低點 + 5 mm），重新產生 MJCF 後同步
        'ti5_170a': Robot('ti5_170a_graphen', 'mjcf/ti5_170a.xml', False, 'error', '0.899'),
        'fb70': Robot('fb70_description', None, True, 'error', '1.07'),
    }

    def resolve_mujoco_model(context):
        model = robots[robot.perform(context)]
        actions = [SetLaunchConfiguration('mujoco_missing_joints', model.missing_joints),
                   SetLaunchConfiguration('mujoco_initial_base_height', model.base_height)]
        if not is_mujoco(context):
            return actions
        if not model.servo and hardware.perform(context) != 'mujoco_pd':
            raise RuntimeError(f'robot:={robot.perform(context)} needs hardware:=mujoco_pd '
                               '(the model has motor actuators); use robot:=fb70 for hardware:=mujoco')
        relpath = model.mjcf or os.path.join(
            'mjcf', 't170_fixed.xml' if IfCondition(mujoco_fixed_base).evaluate(context)
            else 't170_free.xml')
        if not mujoco_model.perform(context):
            actions.append(SetLaunchConfiguration(
                'mujoco_model', os.path.join(get_package_share_directory(model.package), relpath)))
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
        ' robot:=', robot,
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
    #    共用設定 → hardware/<position|pd>_common.yaml → imu/common.yaml → imu/<mujoco|mock|imu16460>.yaml
    #    →（mujoco_pd / real_pd）<robot 套件>/config/joint_pd_gains.yaml →（情境 launch）scenario_params，
    #    後載入的覆蓋先載入的。參數檔無法用 condition 切換，所以在 OpaqueFunction 中組合
    def create_controller_manager(context):
        hw = hardware.perform(context)
        robot_name = robot.perform(context)
        pd = hw in ('mujoco_pd', 'real_pd')
        # 子群組型別與接線只取決於控制方式（位置 / PD），與硬體、模型無關
        files = [controllers_yaml_file, os.path.join(
            config_dir, 'hardware', 'pd_common.yaml' if pd else 'position_common.yaml')]
        if is_mujoco(context):
            imu = 'mujoco'
        else:
            imu = 'mock' if IfCondition(use_mock_imu).evaluate(context) else 'imu16460'
        files += [os.path.join(config_dir, 'imu', 'common.yaml'),
                  os.path.join(config_dir, 'imu', f'{imu}.yaml')]
        if pd:
            # 預設增益屬於模型：顯式 PD 的增益要配合模型的慣量，否則會振盪
            files.append(os.path.join(get_package_share_directory(robots[robot_name].package),
                                      'config', 'joint_pd_gains.yaml'))
        # 情境 launch 的覆寫（例如一開始就進入 RL 模式），不改本機 controllers.yaml
        if scenario_params.perform(context):
            files.append(scenario_params.perform(context))
        def shown(path):
            return os.path.relpath(path, config_dir) if path.startswith(config_dir) else path
        return [LogInfo(msg=f'robot: {robot_name} (URDF body, MJCF, PD gains)'),
                LogInfo(msg='controller parameter files: ' + ', '.join(
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

    # 3-2. IMU 發布 sensor_msgs/Imu（/imu_sensor_broadcaster/imu），所有 IMU 來源都啟動；
    #       mock IMU 時與 sensor_forwarder 並存（forwarder 寫入、broadcaster 讀取 state）
    spawn_imu_sensor_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster'],
        output='screen'
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
