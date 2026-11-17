#!/usr/bin/env python3
"""
把一個文字檔的內容發布成 std_msgs/String（latched），例如 SRDF → /robot_description_semantic.

  ros2 run graphen_bringup file_publisher --ros-args -p file_path:=<檔案> -p topic:=<topic>

QoS 與 robot_state_publisher 的 /robot_description 相同（reliable + transient_local，depth 1），
晚加入的 subscriber 也能收到。只在啟動時讀一次檔案；檔案改變後要重啟這個 node。
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class FilePublisher(Node):

    def __init__(self):
        super().__init__('file_publisher')
        file_path = self.declare_parameter('file_path', '').value
        topic = self.declare_parameter('topic', '').value
        if not file_path or not topic:
            raise ValueError("parameters 'file_path' and 'topic' must both be set")

        with open(file_path, encoding='utf-8') as f:
            content = f.read()

        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # 必須保留 publisher 的參考：transient_local 的訊息由 publisher 保存
        self._publisher = self.create_publisher(String, topic, qos)
        self._publisher.publish(String(data=content))
        self.get_logger().info(
            f"Published '{file_path}' ({len(content)} bytes) on "
            f"'{self._publisher.topic_name}' (transient_local)")


def main():
    rclpy.init()
    try:
        node = FilePublisher()
    except (OSError, ValueError) as e:
        print(f'file_publisher: {e}', file=sys.stderr)
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
