"""MuJoCo + 外部關節軌跡（GR00T / VLA）：模擬一開始就進入 trajectory 模式。

trajectory 模式（graphen_ros_bridge/JointTrajectoryFollower）訂閱 trajectory_msgs/JointTrajectory，
經 PD 追蹤。這個 launch 以 bringup.launch.py 啟動 MuJoCo，並以情境參數把 default_mode 設成 mode，
不必再送 switch_mode。本機的 controllers.yaml 不需要修改，但 mode 必須已經定義在其中
（範本 controllers.yaml.example 的 trajectory）。

GR00T 的 topic 在另一組 RMW 設定下（預設 domain 35、Fast DDS、UDPv4、不限本機）。這個 launch 只讓
自己啟動的程序使用這組設定：BT、ros2 CLI 等要和它互通時，必須在同樣的環境變數下執行。
不沿用呼叫端的 FASTRTPS_DEFAULT_PROFILES_FILE（那份 XML 通常只存在於發布端的機器）。
FASTDDS_BUILTIN_TRANSPORTS=UDPv4（預設）在 WSL 開發機上會讓 controller_manager 的 service 沒有回應、
spawner 起不來；不需要跨 container / 機器時以 fastdds_builtin_transports:=none 不設定。

預設吊掛（mujoco_fixed_base:=true），沒有平衡能力時不會倒下；同時開啟 MuJoCo 視窗（mujoco_gui:=false 關閉）。

  ros2 launch graphen_bringup sim_vla.launch.py
  ros2 launch graphen_bringup sim_vla.launch.py mujoco_fixed_base:=false
  ros2 launch graphen_bringup sim_vla.launch.py ros_domain_id:=0 mujoco_gui:=false
  ros2 launch graphen_bringup sim_vla.launch.py fastdds_builtin_transports:=none   # WSL 開發機
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable,
    UnsetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_launch = os.path.join(
        get_package_share_directory('graphen_bringup'), 'launch', 'bringup.launch.py')

    mode = LaunchConfiguration('mode')
    hardware = LaunchConfiguration('hardware')
    robot = LaunchConfiguration('robot')
    mujoco_fixed_base = LaunchConfiguration('mujoco_fixed_base')
    mujoco_viewer_port = LaunchConfiguration('mujoco_viewer_port')
    mujoco_gui = LaunchConfiguration('mujoco_gui')
    mujoco_viewer_python = LaunchConfiguration('mujoco_viewer_python')

    declare_args = [
        DeclareLaunchArgument(
            'mode', default_value='trajectory',
            description='Mode entered at startup (must be defined in controllers.yaml); '
                        'trajectory follows the JointTrajectory topic of that mode'),
        DeclareLaunchArgument(
            'hardware', default_value='mujoco_pd', choices=['mujoco', 'mujoco_pd'],
            description='MuJoCo control: position actuators or torque actuators + PdController'),
        DeclareLaunchArgument(
            'robot', default_value='ti5_t170',
            choices=['ti5_170a', 'ti5_t170', 'fb70', 'aiterone_ny'],
            description='Simulated model (ti5_t170 / ti5_170a need hardware:=mujoco_pd)'),
        DeclareLaunchArgument(
            'mujoco_fixed_base', default_value='true',
            description='Pin the floating base so the robot hangs instead of falling'),
        DeclareLaunchArgument(
            'mujoco_viewer_port', default_value='9870',
            description='UDP port of graphen_mujoco/tools/mujoco_viewer.py (0 disables the window)'),
        DeclareLaunchArgument(
            'mujoco_gui', default_value='true',
            description='Also start the MuJoCo window on the simulated MJCF'),
        DeclareLaunchArgument(
            'mujoco_viewer_python', default_value='',
            description='Python interpreter for the MuJoCo window; empty keeps the script default'),
        # 發布端（GR00T）的 RMW 設定；只套用到這個 launch 啟動的程序
        DeclareLaunchArgument(
            'ros_domain_id', default_value='35', description='ROS_DOMAIN_ID of the publisher'),
        DeclareLaunchArgument(
            'ros_localhost_only', default_value='0',
            description='ROS_LOCALHOST_ONLY; 1 cannot discover a publisher on another machine'),
        DeclareLaunchArgument(
            'rmw_implementation', default_value='rmw_fastrtps_cpp',
            description='RMW_IMPLEMENTATION of the publisher'),
        DeclareLaunchArgument(
            'fastdds_builtin_transports', default_value='UDPv4',
            description='FASTDDS_BUILTIN_TRANSPORTS (UDPv4: no shared memory, works across '
                        'containers / machines). none or empty leaves it unset (needed on WSL, '
                        'where UDPv4 makes the controller_manager services hang)'),
    ]

    # none / 空字串時不設定。命令列的 :='' 會收到字面上的兩個引號，一併視為不設定
    def set_builtin_transports(context):
        value = LaunchConfiguration('fastdds_builtin_transports').perform(context).strip()
        if value in ('', 'none', "''", '""'):
            return [UnsetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS')]
        return [SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', value)]

    def include_bringup(context):
        # 情境參數：一開始就進入 mode
        scenario = tempfile.NamedTemporaryFile(
            'w', prefix='sim_vla_', suffix='.yaml', delete=False)
        scenario.write(
            'switch_group_controller:\n'
            '  ros__parameters:\n'
            f'    default_mode: {mode.perform(context)}\n')
        scenario.close()
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                'hardware': hardware.perform(context),
                'robot': robot.perform(context),
                'mujoco_fixed_base': mujoco_fixed_base.perform(context),
                'mujoco_viewer_port': mujoco_viewer_port.perform(context),
                'mujoco_gui': mujoco_gui.perform(context),
                'mujoco_viewer_python': mujoco_viewer_python.perform(context),
                'scenario_params': scenario.name,
            }.items())]

    return LaunchDescription(declare_args + [
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', LaunchConfiguration('ros_localhost_only')),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', LaunchConfiguration('rmw_implementation')),
        OpaqueFunction(function=set_builtin_transports),
        UnsetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE'),
        OpaqueFunction(function=include_bringup),
    ])
