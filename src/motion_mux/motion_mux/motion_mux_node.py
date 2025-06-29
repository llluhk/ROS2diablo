import rclpy
from rclpy.node import Node
from motion_msgs.msg import MotionCtrl

class MotionMux(Node):
    def __init__(self):
        super().__init__('motion_mux')

        # Active times for priorities (in seconds)
        self.escape_active_time = 2.0
        self.experiment_active_time = 1.0

        self.last_escape_time = None
        self.last_experiment_time = None

        # Subscriptions
        self.sub_escape = self.create_subscription(
            MotionCtrl, '/diablo/MotionCmd_escape', self.escape_callback, 10)
        
        self.sub_experiment = self.create_subscription(
            MotionCtrl, '/diablo/MotionCmd_experiment', self.experiment_callback, 10)
        
        self.sub_teleop = self.create_subscription(
            MotionCtrl, '/diablo/MotionCmd_joystick', self.teleop_callback, 10)

        # Publisher
        self.pub_cmd = self.create_publisher(MotionCtrl, '/diablo/MotionCmd', 10)

        # Timer to reset state
        self.timer = self.create_timer(0.1, self.timer_callback)

    def escape_callback(self, msg):
        self.last_escape_time = self.get_clock().now()
        self.pub_cmd.publish(msg)

    def experiment_callback(self, msg):
        now = self.get_clock().now()

        # Allow only if escape is not active
        if self.last_escape_time is None or \
           (now - self.last_escape_time).nanoseconds / 1e9 > self.escape_active_time:
            self.last_experiment_time = now
            self.pub_cmd.publish(msg)

    def teleop_callback(self, msg):
        now = self.get_clock().now()

        # Only publish if both escape and experiment are inactive
        escape_ok = self.last_escape_time is None or \
                    (now - self.last_escape_time).nanoseconds / 1e9 > self.escape_active_time
        experiment_ok = self.last_experiment_time is None or \
                        (now - self.last_experiment_time).nanoseconds / 1e9 > self.experiment_active_time

        if escape_ok and experiment_ok:
            self.pub_cmd.publish(msg)

    def timer_callback(self):
        now = self.get_clock().now()
        # Reset escape if expired
        if self.last_escape_time and \
           (now - self.last_escape_time).nanoseconds / 1e9 > self.escape_active_time:
            self.last_escape_time = None
        # Reset experiment if expired
        if self.last_experiment_time and \
           (now - self.last_experiment_time).nanoseconds / 1e9 > self.experiment_active_time:
            self.last_experiment_time = None

def main(args=None):
    rclpy.init(args=args)
    node = MotionMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
