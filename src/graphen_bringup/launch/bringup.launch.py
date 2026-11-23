import os
import tempfile
from collections import namedtuple

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler,
    SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    description_share = get_package_share_directory('ti5_170a_graphen')
    bringup_share = get_package_share_directory('graphen_bringup')

    xacro_file = os.path.join(description_share, 'urdf', 'ti5_170a_graphen.urdf.xacro')
    default_srdf_file = os.path.join(description_share, 'config', 'ti5_170a_graphen.srdf')
    config_dir = os.path.join(bringup_share, 'config')
    # 與硬體無關的設定（模式、switch 接線）；控制方式依 hardware:= 疊加 hardware/<position|pd>_common.yaml、
    # IMU 疊加 imu/*.yaml，PD 增益依 robot:= 取自模型套件
    controllers_yaml_file = os.path.join(config_dir, 'controllers.yaml')
    # 實機本體（ti5_hardware）使用的 CAN 設定與初始位置。CAN 配線依 robot 而定（Robot.can_config），
    # 模型沒有自己的配線時用這份
    default_can_config_file = os.path.join(bringup_share, 'config', 'can_driver_config.json')
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
    srdf_file = LaunchConfiguration('srdf_file')
    arm_ik_config = LaunchConfiguration('arm_ik_config')
    arm_ik_server = LaunchConfiguration('arm_ik_server')
    # 由 resolve_can_config 依 robot 設定
    can_config_file = LaunchConfiguration('can_config_path')
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
            choices=['ti5_170a', 'ti5_t170', 'fb70', 'aiterone_ny'],
            description='Robot model, selects the URDF body, the MJCF (MuJoCo), the PD gains '
                        '(mujoco_pd / real_pd) and the CAN wiring (real / real_pd), for every '
                        'hardware. ti5_t170: ti5_t170_description (T170-V2.1-A0, motor actuators); '
                        'ti5_170a: ti5_170a_graphen (own URDF, generated MJCF with motor actuators); '
                        'fb70: fb70_description (FullBodyT70 export, position servos, URDF body of '
                        'ti5_170a_graphen); aiterone_ny: graphen_aiterone_ny_description (NY machine, '
                        'generated MJCF with position servos, own CAN wiring). '
                        'hardware:=mujoco needs fb70 or aiterone_ny'),
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
        DeclareLaunchArgument(
            'arm_ik_config', default_value='auto',
            description='graphen_arm_ik config (arm IK, collision checking) shared by arm_ik_server, '
                        'the choreography path collision check (choreo.arm_ik_config) and the arm '
                        'servo modes. auto: <robot package>/config/arm_ik.yaml when the robot has a '
                        'verified one; none disables all of them'),
        DeclareLaunchArgument(
            'arm_ik_server', default_value='true',
            description='Start graphen_arm_ik arm_ik_server (~/solve, ~/forward) when arm_ik_config '
                        'is set'),
        DeclareLaunchArgument(
            'srdf_file', default_value=default_srdf_file,
            description='SRDF published (latched) on /robot_description_semantic for external '
                        'programs; empty disables'),
    ]

    def is_mujoco(context):
        return hardware.perform(context).startswith('mujoco')

    # 各模型（robot:=）的資訊。模型套件提供 MJCF（MuJoCo）與 config/joint_pd_gains.yaml（PD 增益）；
    # URDF 本體由 ti5_170a_graphen.urdf.xacro 依 robot 選擇。只在需要時查詢模型套件
    #   mjcf：MJCF 相對路徑，None 時依 mujoco_fixed_base 選 mjcf/t170_fixed.xml / t170_free.xml
    #   servo：True 為位置伺服（mujoco / mujoco_pd 都可用），False 為力矩馬達（只能 mujoco_pd）
    #   missing_joints / base_height：MujocoSystem 的設定；初始高度為直膝（ros2_control 的初始姿態，
    #   所有關節 0）下腳底最低點 + 5 mm。以屈膝的訓練姿態計算會讓直膝的腳踩進地面，
    #   吊掛時每一步重設基座、接觸力持續灌入能量，全身的 roll 關節會來回震盪
    #   arm_ik：arm_ik_config:=auto 時是否使用模型套件的 config/arm_ik.yaml（graphen_arm_ik）
    #   can_config：hardware:=real / real_pd 的 CAN 配線，模型套件內的相對路徑；None 為 graphen_bringup 的
    #   config/can_driver_config.json。配線屬於機台，只有自己接線方式的機台才在模型套件提供
    Robot = namedtuple('Robot', 'package mjcf servo missing_joints base_height arm_ik can_config',
                       defaults=(None,))
    robots = {
        # 直膝下腳底最低點為 0.9954 m，留 5 mm。MJCF 的 keyframe 用 0.985（屈膝的訓練姿態），兩者不同
        'ti5_t170': Robot('ti5_t170_description', 'mjcf/ti5_t170.xml', False, 'error', '1.000', True),
        # ti5_170a_graphen/scripts/make_mjcf.py 印出的高度（直膝下腳底最低點 + 5 mm），重新產生 MJCF 後同步
        'ti5_170a': Robot('ti5_170a_graphen', 'mjcf/ti5_170a.xml', False, 'error', '0.899', True),
        # TODO(verify-on-hw): fb70 的 MJCF mesh 指向 Orin 的 /devtest，開發機無法載入；
        # 在 Orin 上以 arm_ik_check 驗證 arm_ik.yaml 之後再改成 True
        'fb70': Robot('fb70_description', None, True, 'error', '1.07', False),
        # graphen_aiterone_ny_description/scripts/make_mjcf.py 印出的高度（直膝下腳底最低點 + 5 mm），重新產生 MJCF 後同步
        # TODO(verify-on-hw): NY 的 CAN 配線（can0–can5，每條腿 / 手臂一條通道）沿用 NY 工作區的設定
        # 位置伺服的增益由 make_mjcf.py 依頻寬計算（--servo-bandwidth / --servo-damping-ratio）
        'aiterone_ny': Robot('graphen_aiterone_ny_description', 'mjcf/graphen_aiterone_ny.xml', True,
                             'error', '0.899', True, 'config/can_driver_config.json'),
    }

    def resolve_can_config(context):
        model = robots[robot.perform(context)]
        path = (os.path.join(get_package_share_directory(model.package), model.can_config)
                if model.can_config else default_can_config_file)
        return [SetLaunchConfiguration('can_config_path', path)]

    set_can_config = OpaqueFunction(function=resolve_can_config)

    # arm_ik_config：auto 依 robot 選擇，none 停用。命令列的 arm_ik_config:='' 會收到字面上的兩個引號，
    # 一併視為停用（否則會被當成路徑，連帶讓 arm_servo 模式與整個 controller configure 失敗）
    def resolve_arm_ik_config(context):
        value = arm_ik_config.perform(context).strip()
        if value in ('', 'none', "''", '""'):
            return ''
        if value != 'auto':
            return value
        model = robots[robot.perform(context)]
        if not model.arm_ik:
            return ''
        return os.path.join(get_package_share_directory(model.package), 'config', 'arm_ik.yaml')

    # 把 arm_ik_config 帶入 switch_group_controller：choreo.arm_ik_config（編舞的路徑碰撞檢查），
    # 以及所有 plugin 為 graphen_arm_ik/ArmServoAlgorithm 的模式（<mode>.arm_ik_config）。
    # 模式從已組合的參數檔中找出，不寫死模式名稱。回傳產生的參數檔（暫存檔），沒有要帶入的值時為 None
    def arm_ik_parameter_file(path, files):
        merged = {}
        for f in files:
            with open(f) as stream:
                content = yaml.safe_load(stream) or {}
            params = (content.get('switch_group_controller') or {}).get('ros__parameters') or {}
            for key, value in params.items():
                if isinstance(value, dict) and isinstance(merged.get(key), dict):
                    merged[key] = {**merged[key], **value}
                else:
                    merged[key] = value
        injected = {'choreo': {'arm_ik_config': path}}
        for mode in merged.get('modes') or []:
            settings = merged.get(mode)
            if isinstance(settings, dict) and settings.get('plugin') == 'graphen_arm_ik/ArmServoAlgorithm':
                injected[mode] = {'arm_ik_config': path}
        out = tempfile.NamedTemporaryFile(
            'w', prefix='graphen_arm_ik_', suffix='.yaml', delete=False)
        with out:
            yaml.safe_dump({'switch_group_controller': {'ros__parameters': injected}}, out)
        return out.name, sorted(m for m in injected if m != 'choreo')

    def resolve_mujoco_model(context):
        model = robots[robot.perform(context)]
        actions = [SetLaunchConfiguration('mujoco_missing_joints', model.missing_joints),
                   SetLaunchConfiguration('mujoco_initial_base_height', model.base_height)]
        if not is_mujoco(context):
            return actions
        if not model.servo and hardware.perform(context) != 'mujoco_pd':
            raise RuntimeError(f'robot:={robot.perform(context)} needs hardware:=mujoco_pd '
                               '(the model has motor actuators); use robot:=fb70 or aiterone_ny for '
                               'hardware:=mujoco')
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
    # 明確指定為字串：否則 launch_ros 會把 URDF 當 YAML 解析，註解中出現「: 」就會失敗
    robot_description = {'robot_description': ParameterValue(robot_description_raw, value_type=str)}

    # 1. State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 1-1. SRDF（joint 分組等語意資訊）給外部程式訂閱；controller 不使用。
    #       QoS 與 /robot_description 相同（transient_local），晚加入的 subscriber 也收得到
    node_srdf_publisher = Node(
        package='graphen_bringup',
        executable='file_publisher',
        name='srdf_publisher',
        output='screen',
        parameters=[{'file_path': srdf_file, 'topic': 'robot_description_semantic'}],
        condition=IfCondition(PythonExpression(["'", srdf_file, "' != ''"]))
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
        # 手臂 IK 設定（依 robot），放在情境覆寫之前：情境仍然可以改掉它
        arm_ik_path = resolve_arm_ik_config(context)
        arm_ik_messages = []
        if arm_ik_path:
            arm_ik_file, servo_modes = arm_ik_parameter_file(
                arm_ik_path,
                files + ([scenario_params.perform(context)] if scenario_params.perform(context) else []))
            files.append(arm_ik_file)
            arm_ik_messages.append(LogInfo(
                msg=f'arm_ik_config: {arm_ik_path} (choreo collision check, servo modes: '
                    f'{", ".join(servo_modes) or "none"})'))
        else:
            arm_ik_messages.append(LogInfo(
                msg='arm_ik_config: disabled (no choreography collision check; arm servo modes '
                    'cannot be entered)'))
        # 情境 launch 的覆寫（例如一開始就進入 RL 模式），不改本機 controllers.yaml
        if scenario_params.perform(context):
            files.append(scenario_params.perform(context))
        def shown(path):
            return os.path.relpath(path, config_dir) if path.startswith(config_dir) else path
        return arm_ik_messages + [LogInfo(msg=f'robot: {robot_name} (URDF body, MJCF, PD gains, CAN wiring)'),
                LogInfo(msg='can_config: ' + can_config_file.perform(context)),
                LogInfo(msg='controller parameter files: ' + ', '.join(
            shown(f) for f in files)), Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description] + files,
            output='screen'
        )]

    node_controller_manager = OpaqueFunction(function=create_controller_manager)

    # 2-1. 手臂 IK service（~/solve、~/forward），給 viewer 的示教與外部 UI；只做計算，不送指令
    def create_arm_ik_server(context):
        path = resolve_arm_ik_config(context)
        if not path or not IfCondition(arm_ik_server).evaluate(context):
            return []
        return [Node(
            package='graphen_arm_ik',
            executable='arm_ik_server',
            output='screen',
            parameters=[{'arm_ik_config': path}],
        )]

    node_arm_ik_server = OpaqueFunction(function=create_arm_ik_server)

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
        set_can_config,
        set_hw_read_timeout,
        node_robot_state_publisher,
        node_srdf_publisher,
        node_controller_manager,
        node_arm_ik_server,
        spawn_joint_state_broadcaster,
        spawn_sensor_forwarder,
        spawn_imu_sensor_broadcaster,
    ] + delayed_spawners)
