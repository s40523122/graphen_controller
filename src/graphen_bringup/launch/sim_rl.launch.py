"""MuJoCo 自由站立 + RL policy：模擬一開始就由 policy 接手。

自由站立時，PD 只保持姿態、沒有平衡能力，啟動後幾秒內就會倒下，之後才切到 RL 模式已經來不及
（policy 沒有學過從倒地起身）。這個 launch 以 bringup.launch.py 啟動 MuJoCo，並在最後疊加一份
情境參數：default_policy 設為 policy（configure 時就載入，activate 時直接進入）、
該 policy 的 entry_speed 覆寫為 0（不先插補到預設姿態，直接推論）。
本機的 controllers.yaml 不需要修改；policy 必須在 policy 庫中（<名稱>.policy.yaml，
預設為 graphen_rl_policy/policy），而且 controllers.yaml 要設定 policy.library_path。

  ros2 launch graphen_bringup sim_rl.launch.py                       # ti5_170a + rl_flat
  ros2 launch graphen_bringup sim_rl.launch.py policy:=rl_walk robot:=fb70
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

    policy = LaunchConfiguration('policy')
    hardware = LaunchConfiguration('hardware')
    robot = LaunchConfiguration('robot')
    mujoco_viewer_port = LaunchConfiguration('mujoco_viewer_port')

    declare_args = [
        DeclareLaunchArgument(
            'policy', default_value='rl_flat',
            description='Policy run from startup: <name>.policy.yaml in the policy library '
                        '(policy.library_path, graphen_rl_policy/policy by default)'),
        DeclareLaunchArgument(
            'hardware', default_value='mujoco_pd', choices=['mujoco', 'mujoco_pd'],
            description='MuJoCo control: position actuators or torque actuators + PdController'),
        DeclareLaunchArgument(
            'robot', default_value='ti5_170a',
            choices=['ti5_170a', 'ti5_t170', 'fb70', 'aiterone_ny'],
            description='Simulated model; must match the model the policy was trained on '
                        '(ti5_t170 / ti5_170a need hardware:=mujoco_pd)'),
        DeclareLaunchArgument(
            'mujoco_viewer_port', default_value='9870',
            description='UDP port of graphen_mujoco/tools/mujoco_viewer.py (0 disables)'),
    ]

    def include_bringup(context):
        policy_name = policy.perform(context)
        # 情境參數：一開始就執行 policy，且不先插補到預設姿態（<policy>.entry_speed 覆寫設定檔的值）
        scenario = tempfile.NamedTemporaryFile(
            'w', prefix='sim_rl_', suffix='.yaml', delete=False)
        scenario.write(
            'switch_group_controller:\n'
            '  ros__parameters:\n'
            f'    default_policy: {policy_name}\n'
            f'    {policy_name}:\n'
            '      entry_speed: 0.0\n')
        scenario.close()
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                'hardware': hardware.perform(context),
                'robot': robot.perform(context),
                'mujoco_fixed_base': 'false',
                'mujoco_viewer_port': mujoco_viewer_port.perform(context),
                'scenario_params': scenario.name,
            }.items())]

    return LaunchDescription(declare_args + [OpaqueFunction(function=include_bringup)])
