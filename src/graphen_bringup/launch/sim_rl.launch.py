"""MuJoCo 自由站立 + RL policy：模擬一開始就由 policy 接手。

自由站立時，PD 只保持姿態、沒有平衡能力，啟動後幾秒內就會倒下，之後才切到 RL 模式已經來不及
（policy 沒有學過從倒地起身）。這個 launch 以 bringup.launch.py 啟動 MuJoCo，並在最後疊加一份
情境參數：default_mode 設為 mode、該模式的 entry_speed 設為 0（不先插補到預設姿態，直接推論）。
本機的 controllers.yaml 不需要修改，但 mode 必須已經定義在其中（modes 與該模式的參數）。

  ros2 launch graphen_bringup sim_rl.launch.py                       # t170a + rl_flat
  ros2 launch graphen_bringup sim_rl.launch.py mode:=rl_walk mujoco_robot:=fb70
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
    hardware = LaunchConfiguration('hardware')
    mujoco_robot = LaunchConfiguration('mujoco_robot')
    mujoco_viewer_port = LaunchConfiguration('mujoco_viewer_port')

    declare_args = [
        DeclareLaunchArgument(
            'mode', default_value='rl_flat',
            description='Mode entered at startup (must be defined in controllers.yaml)'),
        DeclareLaunchArgument(
            'hardware', default_value='mujoco_pd', choices=['mujoco', 'mujoco_pd'],
            description='MuJoCo control: position actuators or torque actuators + PdController'),
        DeclareLaunchArgument(
            'mujoco_robot', default_value='t170a', choices=['fb70', 't170a'],
            description='Simulated model (t170a needs hardware:=mujoco_pd)'),
        DeclareLaunchArgument(
            'mujoco_viewer_port', default_value='9870',
            description='UDP port of graphen_mujoco/tools/mujoco_viewer.py (0 disables)'),
    ]

    def include_bringup(context):
        mode_name = mode.perform(context)
        # 情境參數：一開始就進入 mode，且不先插補到預設姿態（entry_speed 只有 RL policy 模式會讀取）
        scenario = tempfile.NamedTemporaryFile(
            'w', prefix='sim_rl_', suffix='.yaml', delete=False)
        scenario.write(
            'switch_group_controller:\n'
            '  ros__parameters:\n'
            f'    default_mode: {mode_name}\n'
            f'    {mode_name}:\n'
            '      entry_speed: 0.0\n')
        scenario.close()
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                'hardware': hardware.perform(context),
                'mujoco_robot': mujoco_robot.perform(context),
                'mujoco_fixed_base': 'false',
                'mujoco_viewer_port': mujoco_viewer_port.perform(context),
                'scenario_params': scenario.name,
            }.items())]

    return LaunchDescription(declare_args + [OpaqueFunction(function=include_bringup)])
