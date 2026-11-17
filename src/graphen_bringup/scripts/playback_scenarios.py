#!/usr/bin/env python3
"""
播放介面的情境回歸：對著已啟動的 bringup 送出一連串指令，比對 controller 的回應.

涵蓋四個播放入口（switch_mode / play_choreo / play_bvh / run_policy）、舞曲庫的 service、
以及它們之間的切換規則（on_same、不可中斷、stop 搶占、互相取代）。每一項印出 PASS / FAIL，
全部通過時回傳 0。不是測試框架：只是把手動測試的步驟寫成可以重跑的腳本。

**會讓機器人動作**（頸部 yaw 小幅轉動、切換系統模式、播放 BVH、執行 policy）：
只在 mock 或 MuJoCo 上執行，不要對實機使用。建議用獨立的 ROS_DOMAIN_ID，並用 scenario_params
把 choreo.library_path 指到暫存目錄（這個腳本會發佈名稱以 scn_ 開頭的測試舞曲，舞曲庫沒有刪除介面）。

  ros2 run graphen_bringup playback_scenarios                 # 全部（沒有啟用的類型自動略過）
  ros2 run graphen_bringup playback_scenarios --only choreo   # 只跑一個區段
  ros2 run graphen_bringup playback_scenarios --bvh walk --policy rl_flat

需要的設定：系統模式 stop（hold）與 --target-mode 指定的 target 模式（預設 forward）；
編舞區段會動 --joint（預設 NECK_Y），choreo.max_steps 與速度上限要容得下約 0.4 rad/s。
"""

import argparse
import hashlib
import sys
import time

from action_msgs.msg import GoalStatus
from graphen_interfaces.action import PlayBvh, PlayChoreo, RunPolicy, SwitchMode
from graphen_interfaces.msg import PlaybackFeedback, PlaybackOptions
from graphen_interfaces.srv import GetChoreo, ListChoreos, Preload, PublishChoreo
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

IGNORE = PlaybackOptions.ON_SAME_IGNORE
RESTART = PlaybackOptions.ON_SAME_RESTART
RESTART_IF_FINISHED = PlaybackOptions.ON_SAME_RESTART_IF_FINISHED

# 等待 action server / service 出現的時間（秒）
DISCOVERY_TIMEOUT = 5.0
# 一般指令（不等待播完）的回應期限（秒）；第一次載入 BVH / policy 也要在這之內
RESPONSE_TIMEOUT = 15.0
# 讓 controller 處理完上一個指令再送下一個（秒）
SETTLE = 0.3

STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
}


def choreo_script(joint: str, target: float, duration: float, interruptible: bool = True,
                  pose_extra: str = '') -> str:
    """只動一個 joint 的編舞：從目前位置在 duration 秒內走到 target."""
    return (
        'version: 1\n'
        f'interruptible: {"true" if interruptible else "false"}\n'
        'poses:\n'
        f'  p: {{{joint}: {target}{pose_extra}}}\n'
        'tracks:\n'
        '  t:\n'
        f'    joints: [{joint}]\n'
        '    keyframes:\n'
        f'      - {{t: {duration}, pose: p}}\n')


class Outcome:
    """一次 goal 的結果."""

    def __init__(self):
        self.accepted = False
        self.status = None      # GoalStatus
        self.result = None      # PlaybackResult
        self.stages = []        # 收到的 feedback stage
        self.elapsed = 0.0      # 最後一次 feedback 的 elapsed
        self.duration = 0.0     # 送出到結束的時間（秒）
        self.timed_out = False

    @property
    def status_name(self) -> str:
        if not self.accepted:
            return 'REJECTED'
        if self.timed_out:
            return 'TIMEOUT'
        return STATUS_NAMES.get(self.status, str(self.status))

    @property
    def message(self) -> str:
        return self.result.message if self.result is not None else ''


class Pending:
    """已送出、還沒結束的 goal（用來測試等待中被取代或取消）."""

    def __init__(self, runner, outcome, goal_handle, started):
        self.runner = runner
        self.outcome = outcome
        self.goal_handle = goal_handle
        self.started = started

    def cancel(self):
        future = self.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self.runner.node, future, timeout_sec=RESPONSE_TIMEOUT)

    def wait(self, timeout: float) -> Outcome:
        future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.runner.node, future, timeout_sec=timeout)
        self.outcome.duration = time.monotonic() - self.started
        if not future.done():
            self.outcome.timed_out = True
            return self.outcome
        self.outcome.status = future.result().status
        self.outcome.result = future.result().result.result
        return self.outcome


class Runner:
    def __init__(self, node: Node, controller: str):
        self.node = node
        self.controller = controller
        self.clients = {
            'switch_mode': ActionClient(node, SwitchMode, f'{controller}/switch_mode'),
            'play_choreo': ActionClient(node, PlayChoreo, f'{controller}/play_choreo'),
            'play_bvh': ActionClient(node, PlayBvh, f'{controller}/play_bvh'),
            'run_policy': ActionClient(node, RunPolicy, f'{controller}/run_policy'),
        }
        self.goal_types = {
            'switch_mode': SwitchMode.Goal, 'play_choreo': PlayChoreo.Goal,
            'play_bvh': PlayBvh.Goal, 'run_policy': RunPolicy.Goal,
        }
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_publisher = node.create_publisher(String, f'{controller}/mode', qos)
        self.passed = 0
        self.failed = []
        self.skipped = []

    # ── 回報 ──
    def check(self, name: str, ok: bool, detail: str = ''):
        if ok:
            self.passed += 1
            print(f'  PASS  {name}')
        else:
            self.failed.append(name)
            print(f'  FAIL  {name}' + (f'  ({detail})' if detail else ''))

    def expect(self, name: str, outcome: Outcome, status: str, message: str = '',
               mode: str = None, item: str = None):
        """比對 goal 的狀態、message（包含即可）與 result 的 mode / item."""
        problems = []
        if outcome.status_name != status:
            problems.append(f'status {outcome.status_name}, expected {status}')
        if message and message not in outcome.message:
            problems.append(f"message '{outcome.message}', expected to contain '{message}'")
        if outcome.result is not None:
            if mode is not None and outcome.result.mode != mode:
                problems.append(f"mode '{outcome.result.mode}', expected '{mode}'")
            if item is not None and outcome.result.item != item:
                problems.append(f"item '{outcome.result.item}', expected '{item}'")
        self.check(name, not problems, '; '.join(problems))

    def section(self, title: str):
        print(f'\n[{title}]')

    def skip(self, title: str, reason: str):
        self.skipped.append(title)
        print(f'\n[{title}] skipped: {reason}')

    # ── 送出指令 ──
    def available(self, action: str) -> bool:
        return self.clients[action].wait_for_server(timeout_sec=DISCOVERY_TIMEOUT)

    def send(self, action: str, on_same: int = IGNORE, wait: bool = False, **fields):
        """送出 goal，回傳 Pending；被 reject 時回傳 Outcome（accepted 為 False）."""
        goal = self.goal_types[action]()
        for key, value in fields.items():
            setattr(goal, key, value)
        goal.options.on_same = on_same
        goal.options.wait_until_finished = wait

        outcome = Outcome()

        def on_feedback(message):
            outcome.stages.append(message.feedback.feedback.stage)
            outcome.elapsed = message.feedback.feedback.elapsed

        started = time.monotonic()
        future = self.clients[action].send_goal_async(goal, feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=RESPONSE_TIMEOUT)
        if not future.done() or not future.result().accepted:
            outcome.duration = time.monotonic() - started
            return outcome
        outcome.accepted = True
        return Pending(self, outcome, future.result(), started)

    def play(self, action: str, on_same: int = IGNORE, wait: bool = False,
             timeout: float = RESPONSE_TIMEOUT, **fields) -> Outcome:
        """送出 goal 並等到結束."""
        pending = self.send(action, on_same, wait, **fields)
        if isinstance(pending, Outcome):
            return pending
        return pending.wait(timeout)

    def call(self, service_type, name: str, request):
        client = self.node.create_client(service_type, f'{self.controller}/{name}')
        if not client.wait_for_service(timeout_sec=DISCOVERY_TIMEOUT):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=RESPONSE_TIMEOUT)
        return future.result() if future.done() else None

    def pause(self, seconds: float = SETTLE):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def stop(self):
        """回到 stop（hold），讓下一個區段從乾淨的狀態開始."""
        self.play('switch_mode', mode='stop', on_same=RESTART)
        self.pause()


def run_switch_mode(r: Runner, target_mode: str):
    r.section('switch_mode: system modes')
    r.expect('unknown mode is rejected', r.play('switch_mode', mode='no_such_mode'), 'REJECTED')
    r.expect('invalid on_same is rejected', r.play('switch_mode', mode='stop', on_same=9),
             'REJECTED')
    r.expect('reserved type name is not a system mode', r.play('switch_mode', mode='choreo'),
             'REJECTED')

    r.expect('switch to stop', r.play('switch_mode', mode='stop', on_same=RESTART),
             'SUCCEEDED', 'Switched', mode='stop', item='')
    r.expect('same mode, ignore: not restarted', r.play('switch_mode', mode='stop'),
             'SUCCEEDED', 'Already playing this')
    r.expect('same mode, restart: restarted', r.play('switch_mode', mode='stop', on_same=RESTART),
             'SUCCEEDED', 'Switched')
    r.pause()
    r.expect('hold finishes immediately when waited for',
             r.play('switch_mode', mode='stop', on_same=RESTART, wait=True), 'SUCCEEDED',
             'Finished')

    outcome = r.play('switch_mode', mode=target_mode, wait=True, timeout=60.0)
    r.expect(f'{target_mode}: wait until the target is reached', outcome, 'SUCCEEDED',
             'Finished', mode=target_mode)
    r.check('feedback reports the playing stage with elapsed time',
            PlaybackFeedback.STAGE_PLAYING in outcome.stages or outcome.duration < 0.2,
            f'stages {outcome.stages}')
    r.expect('finished mode, restart_if_finished: restarted',
             r.play('switch_mode', mode=target_mode, on_same=RESTART_IF_FINISHED),
             'SUCCEEDED', 'Switched')

    # ~/mode topic 沒有回應：用之後的 ignore 指令確認已經切過去
    r.mode_publisher.publish(String(data='stop'))
    r.pause(1.0)
    r.expect('~/mode topic switches the system mode', r.play('switch_mode', mode='stop'),
             'SUCCEEDED', 'Already playing this', mode='stop')


def run_choreo(r: Runner, joint: str, target_mode: str):
    if not r.available('play_choreo'):
        r.skip('play_choreo', "action not available ('choreo.plugin' not set?)")
        return
    r.section('choreo: library services')
    script_a = choreo_script(joint, 0.2, 1.5)
    script_b = choreo_script(joint, -0.2, 1.5)
    script_lock = choreo_script(joint, 0.15, 3.0, interruptible=False)

    def publish(name, content):
        return r.call(PublishChoreo, 'publish_choreo', PublishChoreo.Request(
            name=name, content=content))

    response = publish('../scn_x', script_a)
    r.check('publish: invalid name is refused',
            response is not None and not response.success and 'name' in response.message,
            getattr(response, 'message', 'no response'))
    response = publish('scn_bad', 'version: 1\ntracks: [')
    r.check('publish: YAML syntax error is refused',
            response is not None and not response.success, getattr(response, 'message', ''))
    response = publish('scn_bad', choreo_script(joint, 0.1, 1.5, pose_extra=', NO_SUCH_JOINT: 0.0'))
    r.check('publish: unknown joint in a pose is refused',
            response is not None and not response.success and 'NO_SUCH_JOINT' in response.message,
            getattr(response, 'message', ''))
    # 兩個 keyframe 之間過快：發佈時就能檢查出來
    too_fast = (
        'version: 1\n'
        'tracks:\n'
        '  t:\n'
        f'    joints: [{joint}]\n'
        '    keyframes:\n'
        f'      - {{t: 1.0, positions: {{{joint}: 0.0}}}}\n'
        f'      - {{t: 1.05, positions: {{{joint}: 0.5}}}}\n')
    response = publish('scn_bad', too_fast)
    r.check('publish: a segment faster than the limits is refused',
            response is not None and not response.success and 'cannot be played' in
            response.message, getattr(response, 'message', ''))

    response = publish('scn_a', script_a)
    sha_a = hashlib.sha256(script_a.encode()).hexdigest()
    r.check('publish: valid choreography returns its sha256',
            response is not None and response.success and response.sha256 == sha_a,
            getattr(response, 'message', 'no response'))
    listing = r.call(ListChoreos, 'list_choreos', ListChoreos.Request())
    entries = dict(zip(listing.names, listing.sha256)) if listing is not None else {}
    r.check('list: contains the published choreography with the same sha256',
            entries.get('scn_a') == sha_a and 'scn_bad' not in entries, str(entries))
    response = r.call(GetChoreo, 'get_choreo', GetChoreo.Request(name='scn_a'))
    r.check('get: returns the published content',
            response is not None and response.success and response.content == script_a)
    response = r.call(GetChoreo, 'get_choreo', GetChoreo.Request(name='scn_missing'))
    r.check('get: unknown name fails', response is not None and not response.success)

    r.section('choreo: playback')
    r.stop()
    r.expect('unpublished name is aborted without switching',
             r.play('play_choreo', name='scn_missing'), 'ABORTED', 'not published', mode='stop')
    r.expect('name and content together are rejected',
             r.play('play_choreo', name='scn_a', content=script_a), 'REJECTED')
    r.expect('neither name nor content is rejected', r.play('play_choreo'), 'REJECTED')
    r.expect('invalid preview content is aborted without switching',
             r.play('play_choreo', content='version: 1\ntracks: ['), 'ABORTED', mode='stop')

    # 從目前位置走到第一個 keyframe 的那一段取決於當下的位置，進入時才檢查：
    # goal 會被接受，但動作不執行（保持），等待中的 goal 以失敗結束
    r.expect('a first segment that is too fast fails on entry and holds',
             r.play('play_choreo', content=choreo_script(joint, 0.5, 0.05), wait=True,
                    timeout=5.0),
             'ABORTED', 'failed', mode='choreo', item='(preview)')
    r.stop()

    outcome = r.play('play_choreo', name='scn_a', wait=True, timeout=10.0)
    r.expect('published choreography plays to the end', outcome, 'SUCCEEDED', 'Finished',
             mode='choreo', item='scn_a')
    r.check('it takes the scripted time (1.5 s)', 1.4 <= outcome.duration <= 2.5,
            f'{outcome.duration:.2f} s')
    r.expect('finished, ignore: not restarted', r.play('play_choreo', name='scn_a'),
             'SUCCEEDED', 'Already playing this')
    r.expect('finished, restart_if_finished: restarted',
             r.play('play_choreo', name='scn_a', on_same=RESTART_IF_FINISHED),
             'SUCCEEDED', 'Switched')

    # 播放中重新發佈：正在播的那一次不受影響；之後同名視為不同內容
    r.play('play_choreo', content=choreo_script(joint, 0.0, 1.0), wait=True, timeout=10.0)
    pending = r.send('play_choreo', name='scn_a', wait=True)
    r.pause(0.5)
    r.expect('playing, restart_if_finished: not restarted',
             r.play('play_choreo', name='scn_a', on_same=RESTART_IF_FINISHED),
             'SUCCEEDED', 'Already playing this')
    response = publish('scn_a', script_b)
    r.check('republish while it is playing', response is not None and response.success)
    outcome = pending.wait(10.0)
    r.expect('the run in progress is not affected by republishing', outcome, 'SUCCEEDED',
             'Finished')
    r.expect('same name with new content counts as different: switched',
             r.play('play_choreo', name='scn_a', wait=True, timeout=10.0), 'SUCCEEDED',
             'Finished', item='scn_a')

    outcome = r.play('play_choreo', content=choreo_script(joint, 0.0, 1.5), wait=True,
                     timeout=10.0)
    r.expect('preview by content plays to the end', outcome, 'SUCCEEDED', 'Finished',
             mode='choreo', item='(preview)')

    r.section('choreo: interruptible set by the script')
    response = publish('scn_lock', script_lock)
    r.check('publish the non-interruptible choreography',
            response is not None and response.success, getattr(response, 'message', ''))
    publish('scn_a', script_a)
    pending = r.send('play_choreo', name='scn_lock', wait=True)
    r.pause(0.5)
    r.expect('system mode is refused while it plays',
             r.play('switch_mode', mode=target_mode), 'ABORTED', 'not interruptible',
             mode='choreo', item='scn_lock')
    r.expect('another choreography is refused while it plays',
             r.play('play_choreo', name='scn_a'), 'ABORTED', 'not interruptible')
    r.expect('restarting it is refused while it plays',
             r.play('play_choreo', name='scn_lock', on_same=RESTART), 'ABORTED',
             'not interruptible')
    r.mode_publisher.publish(String(data=target_mode))
    r.pause(0.5)
    r.expect('~/mode topic is refused too (still playing the choreography)',
             r.play('play_choreo', name='scn_lock'), 'SUCCEEDED', 'Already playing this',
             item='scn_lock')
    r.expect('stop (preempt) still cuts in', r.play('switch_mode', mode='stop'), 'SUCCEEDED',
             'Switched', mode='stop')
    r.expect('the waiting goal is told it was preempted', pending.wait(5.0), 'ABORTED',
             'Preempted by [ stop ]')

    outcome = r.play('play_choreo', name='scn_lock', wait=True, timeout=10.0)
    r.expect('non-interruptible choreography plays to the end', outcome, 'SUCCEEDED', 'Finished')
    r.expect('after it finished, switching is allowed again',
             r.play('play_choreo', name='scn_a'), 'SUCCEEDED', 'Switched', item='scn_a')

    r.section('choreo: cancel only stops waiting')
    r.play('play_choreo', content=choreo_script(joint, 0.0, 1.0), wait=True, timeout=10.0)
    pending = r.send('play_choreo', name='scn_a', on_same=RESTART, wait=True)
    r.pause(0.4)
    pending.cancel()
    r.expect('canceled goal ends as canceled', pending.wait(5.0), 'CANCELED', 'Stopped waiting',
             mode='choreo', item='scn_a')
    r.expect('the motion keeps running after the cancel',
             r.play('play_choreo', name='scn_a', wait=True, timeout=10.0), 'SUCCEEDED',
             'Finished')
    r.play('play_choreo', content=choreo_script(joint, 0.0, 1.0), wait=True, timeout=10.0)
    r.stop()


def run_bvh(r: Runner, bvh: str):
    if not r.available('play_bvh'):
        r.skip('play_bvh', "action not available ('bvh.library_path' not set?)")
        return
    r.section('bvh')
    r.stop()
    r.expect('unknown file is aborted without switching',
             r.play('play_bvh', name='scn_missing'), 'ABORTED', 'not found', mode='stop')
    r.expect('invalid name is rejected', r.play('play_bvh', name='../x'), 'REJECTED')
    r.expect('negative time_scale is rejected',
             r.play('play_bvh', name=bvh, time_scale=-1.0), 'REJECTED')

    response = r.call(Preload, 'preload', Preload.Request(mode='bvh', items=['scn_missing']))
    r.check('preload: unknown file reports the failure',
            response is not None and not response.success and response.messages[0] != '')
    response = r.call(Preload, 'preload', Preload.Request(mode='choreo', items=['x']))
    r.check('preload: unsupported mode reports the failure',
            response is not None and not response.success)
    response = r.call(Preload, 'preload', Preload.Request(mode='bvh', items=[bvh]))
    r.check('preload: loads the file', response is not None and response.success,
            str(getattr(response, 'messages', 'no response')))

    outcome = r.play('play_bvh', name=bvh, loop=True, time_scale=1.0)
    r.expect('play looping', outcome, 'SUCCEEDED', 'Switched', mode='bvh', item=bvh)
    r.check('no loading stage after preload',
            PlaybackFeedback.STAGE_LOADING not in outcome.stages, f'stages {outcome.stages}')
    r.expect('same file and options, ignore: not restarted',
             r.play('play_bvh', name=bvh, loop=True, time_scale=1.0), 'SUCCEEDED',
             'Already playing this')
    r.expect('time_scale 0 means 1.0: still the same playback',
             r.play('play_bvh', name=bvh, loop=True, time_scale=0.0), 'SUCCEEDED',
             'Already playing this')
    r.expect('different options count as a different playback: switched',
             r.play('play_bvh', name=bvh, loop=False, time_scale=1.0), 'SUCCEEDED', 'Switched')
    r.stop()


def run_policy(r: Runner, policy: str):
    if not r.available('run_policy'):
        r.skip('run_policy', "action not available ('policy.library_path' not set?)")
        return
    r.section('policy')
    r.stop()
    r.expect('unknown policy is aborted without switching',
             r.play('run_policy', name='scn_missing'), 'ABORTED', 'not found', mode='stop')
    r.expect('invalid name is rejected', r.play('run_policy', name='Bad Name'), 'REJECTED')

    response = r.call(Preload, 'preload', Preload.Request(mode='policy', items=[policy]))
    r.check('preload: loads the policy', response is not None and response.success,
            str(getattr(response, 'messages', 'no response')))
    outcome = r.play('run_policy', name=policy)
    r.expect('run the policy', outcome, 'SUCCEEDED', 'Switched', mode='policy', item=policy)
    r.check('no loading stage after preload',
            PlaybackFeedback.STAGE_LOADING not in outcome.stages, f'stages {outcome.stages}')
    r.expect('same policy, ignore: not restarted', r.play('run_policy', name=policy),
             'SUCCEEDED', 'Already playing this')
    r.expect('stop takes over from the policy', r.play('switch_mode', mode='stop'),
             'SUCCEEDED', 'Switched', mode='stop')
    r.pause()


def run_cross(r: Runner, joint: str, bvh: str):
    if not r.available('play_choreo') or not r.available('play_bvh'):
        r.skip('cross', 'needs both play_choreo and play_bvh')
        return
    r.section('cross: entries preempt each other')
    r.stop()
    pending = r.send('play_choreo', content=choreo_script(joint, 0.1, 3.0), wait=True)
    r.pause(0.5)
    r.expect('BVH takes over from a choreography',
             r.play('play_bvh', name=bvh, loop=True, time_scale=1.0), 'SUCCEEDED', 'Switched',
             mode='bvh', item=bvh)
    r.expect('the waiting choreography goal is told what preempted it', pending.wait(5.0),
             'ABORTED', f'Preempted by [ bvh:{bvh} ]')
    r.expect('a choreography takes over from the BVH',
             r.play('play_choreo', content=choreo_script(joint, 0.0, 2.0), wait=True,
                    timeout=15.0),
             'SUCCEEDED', 'Finished', mode='choreo')
    r.stop()


SECTIONS = ('switch_mode', 'choreo', 'bvh', 'policy', 'cross')


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Regression scenarios for the playback actions. Moves the robot: '
                    'run it against mock or MuJoCo only.')
    parser.add_argument('--only', help='comma-separated sections: ' + ', '.join(SECTIONS))
    parser.add_argument('--controller', default='/switch_group_controller')
    parser.add_argument('--target-mode', default='forward',
                        help='a target-type system mode (default: forward)')
    parser.add_argument('--joint', default='NECK_Y',
                        help='joint moved by the choreography scenarios (default: NECK_Y)')
    parser.add_argument('--bvh', default='t170wk_walk1_subject1',
                        help='BVH file (without .bvh) in bvh.library_path')
    parser.add_argument('--policy', default='rl_flat', help='policy in policy.library_path')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    sections = SECTIONS if not args.only else tuple(args.only.split(','))
    unknown = [s for s in sections if s not in SECTIONS]
    if unknown:
        parser.error(f'unknown section(s): {", ".join(unknown)}')

    rclpy.init()
    node = Node('playback_scenarios')
    runner = Runner(node, args.controller)
    try:
        if not runner.available('switch_mode'):
            print(f'{args.controller}/switch_mode is not available: is bringup running '
                  '(same ROS_DOMAIN_ID)?', file=sys.stderr)
            return 1
        if 'switch_mode' in sections:
            run_switch_mode(runner, args.target_mode)
        if 'choreo' in sections:
            run_choreo(runner, args.joint, args.target_mode)
        if 'bvh' in sections:
            run_bvh(runner, args.bvh)
        if 'policy' in sections:
            run_policy(runner, args.policy)
        if 'cross' in sections:
            run_cross(runner, args.joint, args.bvh)
        runner.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(f'\n{runner.passed} passed, {len(runner.failed)} failed'
          + (f', skipped sections: {", ".join(runner.skipped)}' if runner.skipped else ''))
    for name in runner.failed:
        print(f'  FAILED: {name}')
    return 1 if runner.failed else 0


if __name__ == '__main__':
    sys.exit(main())
