"""實機 PD 控制 + RL policy（Ti5FlatPolicy）：啟動時直接進入指定模式。

hardware:=real_pd（Ti5Hardware CAN 電流模式 + 主機端 PdController 200 Hz）。
policy 以 ~/cmd_vel vx/vy/wz 為指令，無指令時原地站立。
entry_speed 控制先插補到 default_joint_pos 再讓 policy 接管的速度（0 = 不插補）。

  ros2 launch graphen_bringup real_pd_tc_flat.launch.py
  ros2 launch graphen_bringup real_pd_tc_flat.launch.py entry_speed:=0.0

注意：首次實機測試請先吊掛並執行 real_pd_test_hip_y.launch.py 確認電流方向正確，
      再以此 launch 啟動；real_pd.yaml 的 TODO(verify-on-hw) 項目亦需逐一確認。
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_launch = os.path.join(
        get_package_share_directory('graphen_bringup'), 'launch', 'bringup.launch.py')

    mode = LaunchConfiguration('mode')
    entry_speed = LaunchConfiguration('entry_speed')
    use_mock_imu = LaunchConfiguration('use_mock_imu')
    imu_port = LaunchConfiguration('imu_port')
    imu_baudrate = LaunchConfiguration('imu_baudrate')
    max_missed_cycles = LaunchConfiguration('max_missed_cycles')
    max_command_velocity = LaunchConfiguration('max_command_velocity')

    declare_args = [
        DeclareLaunchArgument(
            'mode', default_value='rl_flat',
            description='Mode entered at startup (must be defined in controllers.yaml)'),
        DeclareLaunchArgument(
            'entry_speed', default_value='0.5',
            description='Speed [rad/s] to interpolate to default_joint_pos before policy starts '
                        '(0 = policy takes over immediately from current position)'),
        DeclareLaunchArgument(
            'use_mock_imu', default_value='false',
            description='true: mock IMU via sensor_forwarder; false: real FSS-IMU16460-C'),
        DeclareLaunchArgument(
            'imu_port', default_value='/dev/ttyTHS1',
            description='Serial port of the real IMU (use_mock_imu:=false)'),
        DeclareLaunchArgument(
            'imu_baudrate', default_value='921600',
            description='Baudrate of the real IMU (use_mock_imu:=false)'),
        DeclareLaunchArgument(
            'max_missed_cycles', default_value='10',
            description='Consecutive cycles with missing CAN feedback tolerated '
                        'before all motors fall back to position hold '
                        '(default 10 gives ~50 ms transition margin after entering current mode)'),
        DeclareLaunchArgument(
            'max_command_velocity', default_value='0.3',
            description='Max command velocity [rad/s] of ti5_hardware '
                        '(lower than bringup default 0.5 for safety)'),
    ]

    def include_bringup(context):
        mode_name = mode.perform(context)
        entry_speed_val = entry_speed.perform(context)
        scenario = tempfile.NamedTemporaryFile(
            'w', prefix='real_pd_tc_flat_', suffix='.yaml', delete=False)
        scenario.write(
            'switch_group_controller:\n'
            '  ros__parameters:\n'
            f'    default_mode: {mode_name}\n'
            f'    {mode_name}:\n'
            f'      entry_speed: {entry_speed_val}\n')
        scenario.close()
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                'hardware': 'real_pd',
                'use_mock_imu': use_mock_imu.perform(context),
                'imu_port': imu_port.perform(context),
                'imu_baudrate': imu_baudrate.perform(context),
                'max_missed_cycles': max_missed_cycles.perform(context),
                'max_command_velocity': max_command_velocity.perform(context),
                'scenario_params': scenario.name,
            }.items())]

    return LaunchDescription(declare_args + [OpaqueFunction(function=include_bringup)])
