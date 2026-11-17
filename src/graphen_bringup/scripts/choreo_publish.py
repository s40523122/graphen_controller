#!/usr/bin/env python3
"""
舞曲庫工具：把編舞腳本發佈到機器人端的舞曲庫，或列出、讀回已發佈的內容.

  choreo_publish <名稱> <腳本檔>      發佈（同名覆寫）；發佈前 controller 會完整檢查
  choreo_publish --list               列出已發佈的編舞與內容的 sha256
  choreo_publish --get <名稱>         把已發佈的內容印到 stdout

需要 bringup 已經啟動，而且 controllers.yaml 有 choreo 區塊（choreo.plugin）。
發佈只寫入舞曲庫的檔案，不會讓機器人動作。回傳值：0 成功、1 失敗、2 用法錯誤。
"""

import argparse
import hashlib
import sys

from graphen_interfaces.srv import GetChoreo, ListChoreos, PublishChoreo
import rclpy
from rclpy.node import Node

# 等待 controller 的 service 出現的時間（秒）
SERVICE_TIMEOUT = 5.0
# 等待回應的時間（秒）
CALL_TIMEOUT = 10.0


def call(node: Node, service_type, name: str, request):
    """呼叫 service 並等待回應；service 不存在或逾時時回傳 None."""
    client = node.create_client(service_type, name)
    if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
        print(f'Service {name} is not available: is bringup running, and is '
              "'choreo.plugin' set in controllers.yaml?", file=sys.stderr)
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=CALL_TIMEOUT)
    if not future.done():
        print(f'Service {name} did not answer in {CALL_TIMEOUT:.0f} s.', file=sys.stderr)
        return None
    return future.result()


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Publish, list or read back choreographies in the robot-side library.')
    parser.add_argument('name', nargs='?', help='choreography name ([a-z0-9_-], 1-64 characters)')
    parser.add_argument('file', nargs='?', help='choreography script (*.choreo.yaml) to publish')
    parser.add_argument('--list', action='store_true', help='list the published choreographies')
    parser.add_argument('--get', metavar='NAME', help='print a published choreography')
    parser.add_argument(
        '--controller', default='/switch_group_controller',
        help='controller node name (default: /switch_group_controller)')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    publishing = args.name is not None and args.file is not None
    if sum([publishing, args.list, args.get is not None]) != 1 or (
            args.name is not None and args.file is None):
        parser.print_usage(sys.stderr)
        return 2

    rclpy.init()
    node = Node('choreo_publish')
    try:
        if args.list:
            response = call(node, ListChoreos, f'{args.controller}/list_choreos',
                            ListChoreos.Request())
            if response is None:
                return 1
            if not response.names:
                print('No published choreographies.')
            for name, sha256 in zip(response.names, response.sha256):
                print(f'{sha256}  {name}')
            return 0

        if args.get is not None:
            response = call(node, GetChoreo, f'{args.controller}/get_choreo',
                            GetChoreo.Request(name=args.get))
            if response is None:
                return 1
            if not response.success:
                print(f'Error: {response.message}', file=sys.stderr)
                return 1
            sys.stdout.write(response.content)
            return 0

        try:
            with open(args.file, encoding='utf-8') as file:
                content = file.read()
        except OSError as error:
            print(f'Error: cannot read {args.file}: {error}', file=sys.stderr)
            return 1
        response = call(node, PublishChoreo, f'{args.controller}/publish_choreo',
                        PublishChoreo.Request(name=args.name, content=content))
        if response is None:
            return 1
        if not response.success:
            print(f'Not published: {response.message}', file=sys.stderr)
            return 1
        # controller 回傳的 hash 應該和本機內容一致；不一致表示傳輸或編碼有問題
        local = hashlib.sha256(content.encode('utf-8')).hexdigest()
        print(f"Published '{args.name}' (sha256 {response.sha256}).")
        if local != response.sha256:
            print('Warning: the published sha256 differs from the local file '
                  f'({local}).', file=sys.stderr)
            return 1
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
