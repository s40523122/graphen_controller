#!/usr/bin/env python3
"""
播放介面的情境回歸：對著已啟動的 bringup 送出一連串指令，比對 controller 的回應.

涵蓋四個播放入口（switch_mode / play_choreo / play_bvh / run_policy）、舞曲庫的 service、
它們之間的切換規則（on_same、不可中斷、stop 搶占、互相取代），以及影子試播（dry_run、shadow_token）
與 ~/playback_state。每一項印出 PASS / FAIL，
全部通過時回傳 0。不是測試框架：只是把手動測試的步驟寫成可以重跑的腳本。

**會讓機器人動作**（頸部 yaw 小幅轉動、切換系統模式、播放 BVH、執行 policy）：
只在 mock 或 MuJoCo 上執行，不要對實機使用。建議用獨立的 ROS_DOMAIN_ID，並用 scenario_params
把 choreo.library_path 指到暫存目錄（這個腳本會發佈名稱以 scn_ 開頭的測試舞曲，舞曲庫沒有刪除介面）。

  ros2 run graphen_bringup playback_scenarios                 # 全部（沒有啟用的類型自動略過）
  ros2 run graphen_bringup playback_scenarios --only choreo   # 只跑一個區段
  ros2 run graphen_bringup playback_scenarios --bvh walk --policy rl_flat

需要的設定：系統模式 stop（hold）、--teach-mode 指定的 hold 模式（預設 teach，shadow_playback: true，shadow 區段）
與 --target-mode 指定的 target 模式（預設 forward）；
編舞區段會動 --joint（預設 NECK_Y），choreo.max_steps 與速度上限要容得下約 0.4 rad/s。
arm 區段（手臂 IK）需要 arm_ik_server、arm_ik.yaml 的 right 手臂與 --servo-mode（預設 arm_servo）；
伺服會移動右手臂，手臂子群組要 output_enabled: true，否則追蹤的項目會 FAIL。
"""

import argparse
import copy
import hashlib
import math
import sys
import time

from action_msgs.msg import GoalStatus
from graphen_interfaces.action import PlayBvh, PlayChoreo, RunPolicy, SwitchMode
from graphen_interfaces.msg import (ArmPoseTarget, PlaybackFeedback, PlaybackOptions,
                                    PlaybackState)
from graphen_interfaces.srv import (ArmForward, GetChoreo, ListChoreos, Preload, PublishChoreo,
                                    SolveArmIk, TcpJog)
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
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

    def send(self, action: str, on_same: int = IGNORE, wait: bool = False,
             dry_run: bool = False, shadow_token: str = '', **fields):
        """送出 goal，回傳 Pending；被 reject 時回傳 Outcome（accepted 為 False）."""
        goal = self.goal_types[action]()
        for key, value in fields.items():
            setattr(goal, key, value)
        goal.options.on_same = on_same
        goal.options.wait_until_finished = wait
        goal.options.dry_run = dry_run
        goal.options.shadow_token = shadow_token

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
             timeout: float = RESPONSE_TIMEOUT, dry_run: bool = False, shadow_token: str = '',
             **fields) -> Outcome:
        """送出 goal 並等到結束."""
        pending = self.send(action, on_same, wait, dry_run, shadow_token, **fields)
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


# 右手臂從胸前橫向掃過身體的編舞：兩個 keyframe 本身都不碰撞，中間的路徑會碰到髖部 / 腰部
# （ti5_170a 的模型；以 arm_ik_server 搜尋得到）
CROSSING_CHOREO = (
    'version: 1\n'
    'tracks:\n'
    '  right_arm:\n'
    '    joints: [R_SHOULDER_P, R_SHOULDER_R, R_SHOULDER_Y, R_ELBOW_Y, R_WRIST_P, R_WRIST_Y, '
    'R_WRIST_R]\n'
    '    keyframes:\n'
    '      - {t: 2.0, positions: {R_SHOULDER_P: -1.198, R_SHOULDER_R: 1.378, R_SHOULDER_Y: 1.064, '
    'R_ELBOW_Y: 1.578, R_WRIST_P: 0.0, R_WRIST_Y: 0.0, R_WRIST_R: 0.0}}\n'
    '      - {t: 4.0, positions: {R_SHOULDER_P: -1.394, R_SHOULDER_R: 1.378, R_SHOULDER_Y: -1.064, '
    'R_ELBOW_Y: 1.578, R_WRIST_P: 0.0, R_WRIST_Y: 0.0, R_WRIST_R: 0.0}}\n')


# arm 區段的起始姿態：右手在胸前、手肘彎曲（ti5_170a 不碰撞、離限位有餘裕），
# 讓每次執行都從同一個姿態開始（上一次執行可能把手臂停在軀幹旁邊）
ARM_START_CHOREO = (
    'version: 1\n'
    'tracks:\n'
    '  right_arm:\n'
    '    joints: [R_SHOULDER_P, R_SHOULDER_R, R_SHOULDER_Y, R_ELBOW_Y, R_WRIST_P, R_WRIST_Y, '
    'R_WRIST_R]\n'
    '    keyframes:\n'
    '      - {t: 8.0, positions: {R_SHOULDER_P: 0.6, R_SHOULDER_R: 1.2, R_SHOULDER_Y: 0.0, '
    'R_ELBOW_Y: 0.8, R_WRIST_P: 0.0, R_WRIST_Y: 0.0, R_WRIST_R: 0.0}}\n')


def quat_mul(a, b):
    """四元數乘法，(x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_rotate(q, v):
    """以四元數 q（x, y, z, w）旋轉向量 v."""
    x, y, z, _ = quat_mul(quat_mul(q, (v[0], v[1], v[2], 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return (x, y, z)


def arm_choreo(joint_names, positions, duration: float) -> str:
    """一個 keyframe 的右手臂編舞（tcp_jog 的結果交給 PlayChoreo 的方式）."""
    values = ', '.join(f'{name}: {value:.6f}' for name, value in zip(joint_names, positions))
    return ('version: 1\n'
            'tracks:\n'
            '  right_arm:\n'
            f'    joints: [{", ".join(joint_names)}]\n'
            '    keyframes:\n'
            f'      - {{t: {duration}, positions: {{{values}}}}}\n')


def run_tcp_jog(r: Runner, service, forward):
    """~/tcp_jog：以基準姿態沿一個軸單步，結果交給 PlayChoreo 播放."""
    r.section('arm: tcp_jog')
    current = forward()
    if current is None or not current.success:
        r.check('tcp_jog: forward of the current pose', False, getattr(current, 'message', 'no response'))
        return
    reference = list(current.positions)
    names = list(current.joint_names)
    base = current.pose.pose
    base_p = (base.position.x, base.position.y, base.position.z)
    base_q = (base.orientation.x, base.orientation.y, base.orientation.z, base.orientation.w)

    def jog(axis, delta, frame=TcpJog.Request.FRAME_TOOL, ref=None):
        return service(TcpJog, 'tcp_jog', TcpJog.Request(
            arm='right', axis=axis, delta=delta, frame=frame,
            reference=reference if ref is None else ref))

    def pose_of(positions):
        """positions 的末端位姿（forward 帶 state，不依賴機器人目前的姿態）."""
        response = service(ArmForward, 'forward', ArmForward.Request(
            arm='right', state=JointState(name=names, position=list(positions))))
        pose = response.pose.pose
        return ((pose.position.x, pose.position.y, pose.position.z),
                (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                response.swivel)

    def describe(response):
        return getattr(response, 'message', 'no response')

    # 手腕座標的平移：位移沿手腕目前的 x 軸，姿態與手肘不變
    step = 0.02
    response = jog(TcpJog.Request.AXIS_X, step)
    ok = response is not None and response.success and response.max_joint_delta > 0.0
    detail = describe(response)
    if ok:
        p, q, swivel = pose_of(response.positions)
        expected = quat_rotate(base_q, (step, 0.0, 0.0))
        error = math.dist([p[i] - base_p[i] for i in range(3)], expected)
        ok = error < 1e-3 and abs(math.remainder(swivel - current.swivel, math.tau)) < 1e-2
        detail = f'displacement error {error * 1e3:.2f} mm, swivel {current.swivel:.4f} -> {swivel:.4f}'
    r.check('tcp_jog: X 2 cm in the tool frame moves along the wrist x axis, elbow kept', ok, detail)

    # 軀幹座標的平移
    response = jog(TcpJog.Request.AXIS_Z, step, frame=TcpJog.Request.FRAME_ROOT)
    ok = response is not None and response.success
    detail = describe(response)
    if ok:
        p, _, _ = pose_of(response.positions)
        error = math.dist([p[i] - base_p[i] for i in range(3)], (0.0, 0.0, step))
        ok = error < 1e-3
        detail = f'displacement error {error * 1e3:.2f} mm'
    r.check('tcp_jog: Z 2 cm in the root frame moves straight up', ok, detail)

    # 手腕座標的旋轉：位置不變，相對旋轉 = 繞手腕 x 軸 angle
    angle = 0.1
    response = jog(TcpJog.Request.AXIS_ROLL, angle)
    ok = response is not None and response.success
    detail = describe(response)
    if ok:
        p, q, _ = pose_of(response.positions)
        # 誤差 = 實際的相對旋轉與預期旋轉之間的角度
        relative = quat_mul((-base_q[0], -base_q[1], -base_q[2], base_q[3]), q)
        remaining = quat_mul((-math.sin(angle / 2), 0.0, 0.0, math.cos(angle / 2)), relative)
        rotation_error = 2.0 * math.acos(min(1.0, abs(remaining[3])))
        moved = math.dist(p, base_p)
        ok = moved < 1e-3 and rotation_error < 1e-2
        detail = f'tip moved {moved * 1e3:.2f} mm, rotation error {rotation_error:.4f} rad'
    r.check('tcp_jog: ROLL 0.1 rad turns about the wrist x axis in place', ok, detail)

    # 手肘：末端不動、swivel 改變（其中一個方向可能被限位擋住）
    messages = []
    ok = False
    for delta in (0.2, -0.2):
        response = jog(TcpJog.Request.AXIS_ELBOW, delta)
        if response is None or not response.success:
            messages.append(describe(response))
            continue
        p, _, swivel = pose_of(response.positions)
        moved = math.dist(p, base_p)
        turned = math.remainder(swivel - current.swivel, math.tau)
        messages.append(f'tip moved {moved * 1e3:.2f} mm, swivel changed {turned:.4f}')
        if moved < 1e-3 and abs(turned - delta) < 1e-2:
            ok = True
            break
    r.check('tcp_jog: ELBOW 0.2 rad keeps the tip (either direction)', ok, '; '.join(messages))

    response = jog(TcpJog.Request.AXIS_X, 2.0)
    r.check('tcp_jog: an unreachable step fails',
            response is not None and not response.success and len(response.positions) == 7,
            describe(response))
    response = jog(9, 0.01)
    r.check('tcp_jog: an unknown axis is refused',
            response is not None and not response.success and 'axis' in response.message,
            describe(response))
    response = jog(TcpJog.Request.AXIS_X, 0.01, ref=[0.0, 0.0])
    r.check('tcp_jog: a reference without 7 values is refused',
            response is not None and not response.success and 'reference' in response.message,
            describe(response))

    # UI 的流程：連續兩步（第二步以第一步的結果為基準），每一步以 PlayChoreo 播放
    target = None
    ok = True
    detail = ''
    ref = reference
    for axis, delta in ((TcpJog.Request.AXIS_Z, 0.02), (TcpJog.Request.AXIS_PITCH, 0.1)):
        response = jog(axis, delta, ref=ref)
        if response is None or not response.success:
            ok, detail = False, describe(response)
            break
        outcome = r.play('play_choreo', on_same=RESTART, wait=True, timeout=15.0,
                         content=arm_choreo(response.joint_names, response.positions, 1.0))
        if outcome.status_name != 'SUCCEEDED':
            ok, detail = False, f'{outcome.status_name}: {outcome.message}'
            break
        ref = list(response.positions)
        target = response.target.pose
    if ok:
        r.pause(0.5)
        reached = forward()
        p = reached.pose.pose.position
        error = math.dist((p.x, p.y, p.z), (target.position.x, target.position.y, target.position.z))
        ok = error < 2e-3
        detail = f'remaining error {error * 1e3:.1f} mm (arm group output disabled?)'
    r.check('tcp_jog: two steps played with PlayChoreo reach the last target', ok, detail)


def run_arm(r: Runner, server: str, servo_mode: str):
    """手臂 IK：arm_ik_server、編舞的路徑碰撞檢查、伺服模式."""
    def service(service_type, name, request):
        client = r.node.create_client(service_type, f'{server}/{name}')
        if not client.wait_for_service(timeout_sec=DISCOVERY_TIMEOUT):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(r.node, future, timeout_sec=RESPONSE_TIMEOUT)
        return future.result() if future.done() else None

    def forward():
        return service(ArmForward, 'forward', ArmForward.Request(arm='right'))

    if forward() is None:
        r.skip('arm', f'{server} not available (arm_ik_config disabled?)')
        return
    r.section('arm: IK service')
    r.expect('move the right arm to the start pose (choreography)',
             r.play('play_choreo', on_same=RESTART, wait=True, timeout=30.0,
                    content=ARM_START_CHOREO), 'SUCCEEDED', 'Finished')
    r.pause(0.5)
    current = forward()
    r.check('forward: current pose of the right arm', current.success and
            current.pose.header.frame_id != '', current.message)
    frame = current.pose.header.frame_id

    def solve_request(pose, frame_id=None, swivel=math.nan):
        request = SolveArmIk.Request(arm='right', swivel=swivel)
        request.target.header.frame_id = frame if frame_id is None else frame_id
        request.target.pose = pose
        return request

    moved = copy.deepcopy(current.pose.pose)
    moved.position.x += 0.03
    response = service(SolveArmIk, 'solve', solve_request(moved))
    r.check('solve: 3 cm forward of the current pose',
            response is not None and response.success and len(response.positions) == 7 and
            response.position_error < 1e-3, getattr(response, 'message', 'no response'))
    if response is not None and response.success and math.isfinite(response.swivel):
        # 其中一個方向可能被限位擋住（手臂下垂時）：任一方向做到即可
        messages = []
        for delta in (0.2, -0.2):
            shifted = service(SolveArmIk, 'solve', solve_request(moved, swivel=response.swivel + delta))
            ok = shifted is not None and shifted.success and shifted.position_error < 1e-3
            messages.append(getattr(shifted, 'message', 'no response'))
            if ok:
                break
        r.check('solve: same tip with the elbow swiveled by 0.2 rad (either direction)', ok,
                '; '.join(messages))
    response = service(SolveArmIk, 'solve', solve_request(moved, frame_id='camera_link'))
    r.check('solve: wrong frame_id is refused',
            response is not None and not response.success and 'frame_id' in response.message,
            getattr(response, 'message', 'no response'))
    inside = copy.deepcopy(current.pose.pose)
    inside.position.x, inside.position.y, inside.position.z = 0.05, 0.0, 0.1
    response = service(SolveArmIk, 'solve', solve_request(inside))
    r.check('solve: a target inside the torso fails and reports collisions',
            response is not None and not response.success and len(response.collisions) > 0,
            getattr(response, 'message', 'no response'))

    run_tcp_jog(r, service, forward)

    # 編舞的路徑碰撞檢查（controller 有 choreo.arm_ik_config 時）
    params = r.call(GetParameters, 'get_parameters',
                    GetParameters.Request(names=['choreo.arm_ik_config']))
    collision_enabled = params is not None and params.values and params.values[0].string_value
    if collision_enabled and r.available('play_choreo'):
        r.section('arm: choreography path collision')
        response = r.call(PublishChoreo, 'publish_choreo',
                          PublishChoreo.Request(name='scn_crossing', content=CROSSING_CHOREO))
        r.check('publish: a path through the body is refused with the time and bodies',
                response is not None and not response.success and 'collision at t=' in
                response.message, getattr(response, 'message', 'no response'))
        r.expect('play: the same content is aborted without switching',
                 r.play('play_choreo', content=CROSSING_CHOREO), 'ABORTED', 'collision at t=')
    else:
        r.skip('arm: choreography path collision', "'choreo.arm_ik_config' is not set")

    outcome = r.play('switch_mode', mode=servo_mode, on_same=RESTART)
    if outcome.status_name == 'REJECTED':
        r.skip('arm: servo', f"system mode '{servo_mode}' not configured")
        return
    r.section('arm: servo')
    r.expect(f'switch to {servo_mode}', outcome, 'SUCCEEDED', 'Switched', mode=servo_mode)
    publisher = r.node.create_publisher(ArmPoseTarget, f'{r.controller}/pose_targets', 10)
    start = forward()

    def stream(pose, seconds, rate=30.0):
        msg = ArmPoseTarget(arm='right', pose=pose, swivel=math.nan)
        msg.header.frame_id = frame
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            publisher.publish(msg)
            r.pause(1.0 / rate)

    def distance(a, b):
        return math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))

    goal = copy.deepcopy(start.pose.pose)
    goal.position.x += 0.05
    goal.position.z += 0.05
    stream(goal, 4.0)
    # 停止發布後，目標在 pose_targets.timeout 之內仍然有效
    r.pause(1.0)
    reached = forward()
    error = distance(reached.pose.pose.position, goal.position)
    r.check('servo: the wrist follows a target 7 cm away', error < 0.005,
            f'remaining error {error * 1e3:.1f} mm (arm group output disabled?)')
    r.pause(1.0)
    held = forward()
    drift = max(abs(a - b) for a, b in zip(held.positions, reached.positions))
    r.check('servo: holds after the target times out', drift < 1e-3, f'moved {drift:.4f} rad')

    inside = copy.deepcopy(start.pose.pose)
    inside.position.x, inside.position.y, inside.position.z = 0.05, 0.0, 0.1
    stream(inside, 4.0)
    stopped = forward()
    r.check('servo: an arm driven into the torso stops before colliding',
            not stopped.collisions and distance(stopped.pose.pose.position, inside.position) > 0.05,
            f'collisions {list(stopped.collisions)}')
    r.node.destroy_publisher(publisher)
    r.stop()


def joint_value(outcome: Outcome, joint: str):
    """result 的 state 快照中 <joint>/position 的值；沒有時為 None."""
    result = outcome.result
    name = f'{joint}/position'
    if result is None or name not in result.interface_names:
        return None
    return result.values[list(result.interface_names).index(name)]


def run_shadow(r: Runner, joint: str, teach_mode: str):
    if not r.available('play_choreo'):
        r.skip('shadow', "play_choreo not available ('choreo.plugin' not set?)")
        return
    r.section('shadow: playback_state and dry run')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    states = []
    shadow = []
    state_sub = r.node.create_subscription(
        PlaybackState, f'{r.controller}/playback_state', states.append, qos)
    shadow_sub = r.node.create_subscription(
        JointState, f'{r.controller}/shadow/joint_states', shadow.append, 10)
    try:
        outcome = r.play('switch_mode', mode=teach_mode, on_same=RESTART)
        r.expect(f'switch to {teach_mode}', outcome, 'SUCCEEDED', mode=teach_mode)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not (states and states[-1].mode == teach_mode):
            r.pause(0.1)
        r.check('playback_state follows the mode', bool(states) and states[-1].mode == teach_mode,
                f'last {states[-1].mode if states else None!r}')
        base = joint_value(outcome, joint)
        if base is None:
            r.check(f'{joint}/position in the state snapshot', False, 'not found')
            return
        target = base + 0.2 if base < 0.5 else base - 0.2
        script = choreo_script(joint, round(target, 4), 1.5)

        # 試播：不切換、不動，回傳 token，軌跡發布到 ~/shadow/joint_states
        shadow.clear()
        dry = r.play('play_choreo', content=script, dry_run=True)
        r.expect('dry run passes without switching', dry, 'SUCCEEDED', 'Dry run finished',
                 mode=teach_mode)
        token = dry.result.shadow_token if dry.result is not None else ''
        r.check('dry run returns a shadow_token', bool(token))
        r.pause(2.5)
        after = joint_value(r.play('switch_mode', mode=teach_mode), joint)
        r.check('dry run does not move the robot',
                after is not None and abs(after - base) < 1e-6, f'{base} -> {after}')
        last = None
        if shadow and joint in shadow[-1].name:
            last = shadow[-1].position[list(shadow[-1].name).index(joint)]
        r.check('dry run trajectory is published and reaches the target',
                last is not None and abs(last - target) < 1e-3,
                f'{len(shadow)} messages, last {last}, target {target}')

        # 試播失敗：第一個 keyframe 在 t: 0 但離目前位置太遠（on_enter 的檢查）
        far = (
            'version: 1\n'
            'tracks:\n'
            '  t:\n'
            f'    joints: [{joint}]\n'
            '    keyframes:\n'
            f'      - {{t: 0.0, positions: {{{joint}: {round(target, 4)}}}}}\n'
            f'      - {{t: 1.0, positions: {{{joint}: {round(target, 4)}}}}}\n')
        failed = r.play('play_choreo', content=far, dry_run=True)
        r.expect('dry run reports a failure on entry', failed, 'ABORTED', 'Dry run failed',
                 mode=teach_mode)
        r.check('failed dry run returns no token',
                failed.result is None or not failed.result.shadow_token)

        # 不支援試播的 action
        outcome = r.play('switch_mode', mode=teach_mode, dry_run=True)
        r.expect('switch_mode rejects dry_run', outcome, 'REJECTED')
        outcome = r.play('switch_mode', mode=teach_mode, shadow_token=token)
        r.expect('switch_mode rejects shadow_token', outcome, 'REJECTED')

        # teach（shadow_playback: true）中的 PlayChoreo 一律只做影子試播，帶 token 也一樣
        outcome = r.play('play_choreo', content=script)
        r.expect('play in the teach mode is a shadow playback', outcome, 'SUCCEEDED',
                 'Shadow playback in mode', mode=teach_mode)
        r.check('shadow playback returns a shadow_token',
                outcome.result is not None and bool(outcome.result.shadow_token))
        outcome = r.play('play_choreo', content=script, shadow_token=token, wait=True)
        r.expect('a token does not play for real in the teach mode', outcome, 'SUCCEEDED',
                 'Shadow playback in mode', mode=teach_mode)
        after = joint_value(r.play('switch_mode', mode=teach_mode), joint)
        r.check('shadow playback does not move the robot',
                after is not None and abs(after - base) < 1e-6, f'{base} -> {after}')

        # 憑證檢查：先切到 stop（hold，同樣不動，起點不變）
        outcome = r.play('switch_mode', mode='stop', on_same=RESTART)
        r.expect('leave the teach mode (stop)', outcome, 'SUCCEEDED', mode='stop')
        outcome = r.play('play_choreo', content=script, shadow_token='bogus')
        r.expect('unknown shadow_token is refused', outcome, 'ABORTED', 'unknown or expired',
                 mode='stop')
        other = choreo_script(joint, round(target, 4), 2.0)
        outcome = r.play('play_choreo', content=other, shadow_token=token)
        r.expect('shadow_token of other content is refused', outcome, 'ABORTED',
                 'content differs', mode='stop')
        outcome = r.play('play_choreo', content=script, shadow_token=token, wait=True,
                         timeout=10.0)
        r.expect('valid shadow_token plays', outcome, 'SUCCEEDED', 'Finished', mode='choreo')
        # 「播完」以命令位置判斷；量測值稍後才跟上（WSL 時鐘回跳時 RT 迴圈會暫停約 1 s），等一下再讀
        r.pause(1.5)
        reached = joint_value(r.play('switch_mode', mode='stop', on_same=RESTART), joint)
        r.check('played to the dry run target',
                reached is not None and abs(reached - target) < 1e-3, f'{reached} vs {target}')
        r.pause()
        outcome = r.play('play_choreo', content=script, shadow_token=token)
        r.expect('shadow_token is refused after the robot moved', outcome, 'ABORTED',
                 'moved since the dry run', mode='stop')
    finally:
        r.node.destroy_subscription(state_sub)
        r.node.destroy_subscription(shadow_sub)
        r.stop()


SECTIONS = ('switch_mode', 'choreo', 'shadow', 'bvh', 'policy', 'cross', 'arm')


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Regression scenarios for the playback actions. Moves the robot: '
                    'run it against mock or MuJoCo only.')
    parser.add_argument('--only', help='comma-separated sections: ' + ', '.join(SECTIONS))
    parser.add_argument('--controller', default='/switch_group_controller')
    parser.add_argument('--target-mode', default='forward',
                        help='a target-type system mode (default: forward)')
    parser.add_argument('--teach-mode', default='teach',
                        help='a hold-type system mode with shadow_playback: true, used by the shadow '
                             'section (default: teach)')
    parser.add_argument('--joint', default='NECK_Y',
                        help='joint moved by the choreography scenarios (default: NECK_Y)')
    parser.add_argument('--bvh', default='t170wk_walk1_subject1',
                        help='BVH file (without .bvh) in bvh.library_path')
    parser.add_argument('--policy', default='rl_flat', help='policy in policy.library_path')
    parser.add_argument('--arm-ik-server', default='/arm_ik_server')
    parser.add_argument('--servo-mode', default='arm_servo',
                        help='system mode using graphen_arm_ik/ArmServoAlgorithm')
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
        if 'shadow' in sections:
            run_shadow(runner, args.joint, args.teach_mode)
        if 'bvh' in sections:
            run_bvh(runner, args.bvh)
        if 'policy' in sections:
            run_policy(runner, args.policy)
        if 'cross' in sections:
            run_cross(runner, args.joint, args.bvh)
        if 'arm' in sections:
            run_arm(runner, args.arm_ik_server, args.servo_mode)
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
