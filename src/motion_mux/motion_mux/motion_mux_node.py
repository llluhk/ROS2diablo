import rclpy
from rclpy.node import Node
from motion_msgs.msg import MotionCtrl  # 替换为你的消息类型
from rclpy.time import Time

class MotionMux(Node):
    def __init__(self):
        super().__init__('motion_mux')

        # 设置优先级和时间阈值
        self.escape_active_time = 2  # escape 话题的优先级有效时间（秒）

        # 记录最近一次 escape 消息的时间
        self.last_escape_time = None

        # 订阅 teleop 和 escape 话题
        self.sub_teleop = self.create_subscription(
            MotionCtrl, '/diablo/MotionCmd_joystick', self.teleop_callback, 10)

        self.sub_escape = self.create_subscription(
            MotionCtrl, '/diablo/MotionCmd_escape', self.escape_callback, 10)

        # 创建发布者
        self.pub_cmd = self.create_publisher(MotionCtrl, '/diablo/MotionCmd', 10)

        # 创建定时器，定期检查是否恢复 teleop
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def escape_callback(self, msg):
        """处理 /diablo/MotionCmd_escape 消息"""
        self.last_escape_time = self.get_clock().now()  # 记录 escape 话题的时间
        self.pub_cmd.publish(msg)  # 立即发布 escape 命令

    def teleop_callback(self, msg):
        """处理 /diablo/MotionCmd_teleop 消息"""
        # 只有当 escape 话题的优先级超时后才发布 teleop 消息
        if self.last_escape_time is None:
            self.pub_cmd.publish(msg)
        else:
            now = self.get_clock().now()
            time_diff = (now - self.last_escape_time).nanoseconds / 1e9  # 转换为秒
            if time_diff > self.escape_active_time:  # 超时，恢复 teleop 控制
                self.pub_cmd.publish(msg)

    def timer_callback(self):
        """定期检查 escape 话题是否超时"""
        if self.last_escape_time is not None:
            now = self.get_clock().now()
            time_diff = (now - self.last_escape_time).nanoseconds / 1e9  # 转换为秒
            if time_diff > self.escape_active_time:
                self.last_escape_time = None  # 超时后重置 escape

def main(args=None):
    rclpy.init(args=args)
    node = MotionMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
