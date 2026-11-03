"""實機 PD 控制電流驗證：僅移動 L/R HIP_Y，觀察 CAN 電流是否正確。

hardware:=real_pd，進入 test_hip_y 模式（SineWaveAlgorithm，amplitude=0.1 rad，
frequency=0.2 Hz，每秒 log 一次）。HIP_Y 為旋轉軸，吊掛下移動對機構最安全。

用途：在跑 rl_flat policy 之前，先驗證 Ti5Hardware 的力矩→電流換算（KT、減速比、
電流方向）是否符合預期，防止增益或換算錯誤導致爆走。

  ros2 launch graphen_bringup real_pd_test_hip_y.launch.py

觀察指令（另開終端）：
  ros2 topic echo /dynamic_joint_states --field interface_values
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

    use_mock_imu = LaunchConfiguration('use_mock_imu')
    imu_port = LaunchConfiguration('imu_port')
    imu_baudrate = LaunchConfiguration('imu_baudrate')
    max_missed_cycles = LaunchConfiguration('max_missed_cycles')
    max_command_velocity = LaunchConfiguration('max_command_velocity')

    declare_args = [
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
            'max_missed_cycles', default_value='2',
            description='Consecutive cycles with missing CAN feedback tolerated '
                        'before all motors fall back to position hold'),
        DeclareLaunchArgument(
            'max_command_velocity', default_value='0.3',
            description='Max command velocity [rad/s] of ti5_hardware'),
    ]

    def include_bringup(context):
        scenario = tempfile.NamedTemporaryFile(
            'w', prefix='real_pd_test_hip_y_', suffix='.yaml', delete=False)
        scenario.write(
            'switch_group_controller:\n'
            '  ros__parameters:\n'
            '    default_mode: test_hip_y\n')
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
