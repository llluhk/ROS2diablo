import rclpy
from rclpy.node import Node
from motion_msgs.msg import MotionCtrl
from custom_msgs.msg import Panel

class Motionctrl_diablo(Node):
    def __init__(self):
        super().__init__('motionctrl_diablo')
        self.panel_sub = self.create_subscription(Panel, '/carrierbot/Panel', self.listener_callback_panel, 10)
        # Subscribe to '/carrierbot/Panel' topic, process joystick signals
        self.mctrl_pub = self.create_publisher(MotionCtrl, '/diablo/MotionCmd_joystick', 10)
        self.timer = self.create_timer(0.05, self.on_timer)
        self.cur_forw_vel_panel = 0
        self.cur_angl_vel_panel = 0
        self.joystick=0

        self.max_forw = 0.1
        self.max_ang = 1.0
        self.mctrl_msg = MotionCtrl()


    def listener_callback_panel(self, msg):
        # Get current forward velocity
        self.cur_forw_vel_panel = max(-self.max_forw, min(msg.joystick.x / 100, self.max_forw))
        self.joystick=msg.joystick.x
        # Get current angular velocity
        self.cur_angl_vel_panel = max(-self.max_ang, min(msg.joystick.y / 100, self.max_ang))


    def on_timer(self):
        self.mctrl_msg.mode.stand_mode = True
        self.mctrl_msg.mode.height_ctrl_mode = True
        self.mctrl_msg.value.up = 1.0
        self.mctrl_msg.value.pitch = 0.0
        # Handle forward control if joystick inputs are active
        if self.cur_forw_vel_panel != 0 or self.cur_angl_vel_panel != 0:
            #self.get_logger().info(f"Joystick input: Forward: {self.cur_forw_vel_panel}, Angular: {self.cur_angl_vel_panel}")
            #self.get_logger().info(f"Joystick input: {self.joystick}")

            self.mctrl_msg.value.forward = float(self.cur_forw_vel_panel)
            self.mctrl_msg.value.left = float(self.cur_angl_vel_panel) 

        else:
            self.mctrl_msg.value.forward=0.0
            self.mctrl_msg.value.left=0.0
            self.mctrl_msg.mode_mark = False  # Reset mode if no movement

        # Publish the message after processing
        self.mctrl_pub.publish(self.mctrl_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Motionctrl_diablo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
