import rclpy
from rclpy.node import Node
from motion_msgs.msg import MotionCtrl  
from custom_msgs.msg import Panel, Collisionprediction  

class EscapeBehaviorNode(Node):
    def __init__(self):
        super().__init__('escape_node')

        self.panel_sub = self.create_subscription(Panel, 'carrierbot/Panel', self.joystick_callback, 10)
        self.prediction_sub = self.create_subscription(Collisionprediction, '/collision_prediction', self.callback, 1)  
        self.joystick_sub = self.create_subscription(MotionCtrl, '/diablo/MotionCmd_joystick', self.joystick_cmd_callback, 10)
        self.motion_pub = self.create_publisher(MotionCtrl, '/diablo/MotionCmd_escape', 10)
        
        self.escape_timer = None
        self.escape_enabled = False
        self.prev_down_button = False
        self.last_cmd = MotionCtrl()  # Store last known motion command

    def joystick_cmd_callback(self, msg):
        self.last_cmd = msg  # Update with latest joystick command

    def joystick_callback(self, msg):
        current_down = msg.mainbuttons.downbutton

        if current_down and not self.prev_down_button:
            self.escape_enabled = not self.escape_enabled
            state = "enabled" if self.escape_enabled else "disabled"
            self.get_logger().info(f"Escape behavior toggled: {state}")

        self.prev_down_button = current_down
        
    def callback(self, msg):  
        prediction = msg.prediction
        timestamp = msg.stamp
        self.get_logger().info(f"Collision prediction received: {prediction} at {timestamp.sec}.{timestamp.nanosec:09d}")
        self.execute_escape_behavior(prediction)

    def execute_escape_behavior(self, prediction):
        self.clear_timers()

        if prediction > 0:
            collision_type = {1: "Front", 2: "Left", 3: "Right"}.get(prediction, "Unknown")
            self.get_logger().info(f"⚠️ Collision predicted: {collision_type} (Code: {prediction})")

            if self.escape_enabled:
                self.get_logger().info("Escape behavior triggered: moving backward.")
                self.publish_modified_motion(forward=-0.5)  # Move backward
                self.escape_timer = self.create_timer(0.5, self.stop_motion)  # Stop after 0.5s
            else:
                self.get_logger().info("Escape is disabled. No motion command sent.")

    def stop_motion(self):
        self.clear_timers()
        self.get_logger().info("Stopping motion.")
        self.publish_modified_motion(forward=0.0)

    def publish_modified_motion(self, forward):
        msg = MotionCtrl()
        msg.value.forward = forward
        msg.value.left = 0.0
        msg.mode.stand_mode = self.last_cmd.mode.stand_mode
        msg.mode.height_ctrl_mode = self.last_cmd.mode.height_ctrl_mode
        msg.value.up = self.last_cmd.value.up
        msg.value.pitch = self.last_cmd.value.pitch

        self.motion_pub.publish(msg)
        self.get_logger().info(f"Published modified motion: forward={forward}")

    def clear_timers(self):
        if self.escape_timer is not None:
            self.escape_timer.cancel()
            self.escape_timer = None

def main(args=None):
    rclpy.init(args=args)
    node = EscapeBehaviorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
