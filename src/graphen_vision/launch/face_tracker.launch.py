"""人臉跟隨的視覺端：face_tracker（可選擇一起啟動 RealSense）。

  ros2 launch graphen_vision face_tracker.launch.py
  ros2 launch graphen_vision face_tracker.launch.py start_camera:=true    # 一起啟動 realsense2_camera
  ros2 launch graphen_vision face_tracker.launch.py source:=device device:=/dev/video4
      # 直接讀 V4L2 裝置（webcam，或 RealSense 的彩色串流當 webcam，不需要 SDK）

輸出 /face_tracker/target（geometry_msgs/PointStamped，像素），由 graphen_bt 的 face_track.xml 使用。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _camera(context):
    if LaunchConfiguration('start_camera').perform(context).lower() not in ('true', '1'):
        return []
    # realsense2_camera 只在需要時尋找：開發機沒有安裝時，start_camera:=false 仍可使用
    rs_share = get_package_share_directory('realsense2_camera')
    # TODO(verify-on-hw): 參數名稱依 realsense-ros 版本而不同（4.54 以後為 rgb_camera.color_profile，
    # 較舊為 rgb_camera.profile），以機器人端安裝的版本確認
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rs_share, 'launch', 'rs_launch.py')),
        launch_arguments={
            'enable_depth': 'false',
            'rgb_camera.color_profile': LaunchConfiguration('color_profile').perform(context),
        }.items())]


def _tracker(context):
    # source / device 有給時覆寫設定檔（空字串 = 用設定檔的值）
    overrides = {}
    for key in ('source', 'device'):
        value = LaunchConfiguration(key).perform(context)
        if value:
            overrides[key] = value
    return [Node(
        package='graphen_vision',
        executable='face_tracker',
        name='face_tracker',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), overrides],
    )]


def generate_launch_description():
    pkg_share = get_package_share_directory('graphen_vision')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(pkg_share, 'config', 'face_tracker.yaml'),
            description='face_tracker parameters'),
        DeclareLaunchArgument(
            'start_camera', default_value='false',
            description='Also start realsense2_camera (rs_launch.py)'),
        DeclareLaunchArgument(
            'color_profile', default_value='640x480x30',
            description='RealSense color profile (width x height x fps); must match '
                        'gp_face_image_width / gp_face_image_height of bt_runner'),
        DeclareLaunchArgument(
            'source', default_value='',
            description='topic or device; empty = the value in params_file'),
        DeclareLaunchArgument(
            'device', default_value='',
            description='V4L2 device for source:=device (e.g. /dev/video4); empty = params_file'),
        OpaqueFunction(function=_camera),
        OpaqueFunction(function=_tracker),
    ])
