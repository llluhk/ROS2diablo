import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from motion_msgs.msg import MotionCtrl

class CollisionSimulation(Node):
    def __init__(self):
        super().__init__('collision_simulation')

        self.mctrl_pub = self.create_publisher(MotionCtrl, '/diablo/MotionCmd', 10)
        self.mctrl_msg = MotionCtrl()

        # Record start time
        self.start_time = self.get_clock().now()

        # Initial setup: send a one-time mode command
        self.send_startup_command()

        # Timer to send motion commands every 0.05s
        self.timer = self.create_timer(0.05, self.on_timer)

    def send_startup_command(self):
        startup_msg = MotionCtrl()
        startup_msg.mode_mark = True
        startup_msg.mode.stand_mode = True
        startup_msg.mode.pitch_ctrl_mode = True
        startup_msg.value.up = 1.0
        startup_msg.value.pitch = 0.0

        self.mctrl_pub.publish(startup_msg)
        self.get_logger().info('Sent startup motion mode command.')

    def on_timer(self):
        # Check if 5 seconds have passed since start
        elapsed_time = self.get_clock().now() - self.start_time
        if elapsed_time > Duration(seconds=2.0):
            self.get_logger().info('5 seconds elapsed. Stopping robot and shutting down node.')

            # Stop the robot by setting forward speed to 0
            self.mctrl_msg.value.forward = 0.0
            self.mctrl_pub.publish(self.mctrl_msg)

            # Stop the timer and exit
            self.timer.cancel()
            return

        # Send continuous forward command
        self.mctrl_msg.mode.pitch_ctrl_mode = True
        self.mctrl_msg.value.up = 1.0
        self.mctrl_msg.value.pitch = 0.0
        self.mctrl_msg.value.forward = 0.8 # Move backward

        self.mctrl_pub.publish(self.mctrl_msg)
        self.get_logger().info('Publishing motion command...')

def main(args=None):
    rclpy.init(args=args)
    node = CollisionSimulation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
