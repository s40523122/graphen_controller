import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from example_interfaces.action import Fibonacci
from std_msgs.msg import Int32


class WebControlToActionNode(Node):

    def __init__(self):
        super().__init__('web_control_to_action_node')

        # 1. 建立 Action Client (發送目標給 /gui_play_set)
        self._action_client = ActionClient(
            self,
            Fibonacci,
            '/gui_play_set'
        )

        # 2. 建立 Topic Subscriber (訂閱 /ti5_manual_web_control)
        self._subscription = self.create_subscription(
            Int32,
            '/ti5_manual_web_control',
            self.topic_callback,
            10
        )
        
        self.get_logger().info('WebControlToAction Node initialized. Ready for topics...')

    def topic_callback(self, msg: Int32):
        order_value = msg.data
        self.get_logger().info(f'Received topic value: {order_value}')

        # 確認 Action Server 是否在線上
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Action server /gui_play_set not available!')
            return

        # 建立 Goal
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order_value

        self.get_logger().info(f'Sending action goal order: {order_value}...')

        # 非同步發送 Goal 並綁定 Response Callback
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by server.')
            return

        self.get_logger().info('Goal accepted by server! Waiting for result...')
        
        # 綁定 Result Callback
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        # 接收執行過程中的 Feedback（如不需要可保持空白）
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback sequence: {feedback.sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Action finished! Result sequence: {result.sequence}')


def main(args=None):
    rclpy.init(args=args)
    node = WebControlToActionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()