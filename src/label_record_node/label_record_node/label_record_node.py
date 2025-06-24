#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from custom_msgs.msg import Panel

class TeleopNode(Node):
    def __init__(self):
        super().__init__("diablo_teleop_node")

        self.label_pub = self.create_publisher(Int32MultiArray, 'collision_label', 10)
        self.subscription = self.create_subscription(Panel, 'carrierbot/Panel', self.joystick_callback, 10)

        self.label_msg = Int32MultiArray()

        # Cooldown time to ignore repeated labels (in seconds)
        self.cooldown = 1.0
        self.last_publish_time = {}

        # Button state memory to detect rising edge
        self.last_button_states = {
            'up': 0,
            'left': 0,
            'right': 0,
        }

    def joystick_callback(self, msg):
        current_states = {
            'up': msg.mainbuttons.upbutton,
            'left': msg.mainbuttons.leftbutton,
            'right': msg.mainbuttons.rightbutton,
        }

        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        label = None
        btn_name = None

        # Detect rising edge and apply cooldown
        for name, value in current_states.items():
            if value == 1 and self.last_button_states[name] == 0:
                temp_label = {'up': 1, 'left': 2, 'right': 3}[name]

                # Check cooldown
                last_time = self.last_publish_time.get(temp_label, 0.0)
                if (now_sec - last_time) >= self.cooldown:
                    label = temp_label
                    btn_name = name
                    break  # Only allow one label at a time

        # Update button states
        self.last_button_states = current_states

        if label is not None:
            sec, nsec = now.seconds_nanoseconds()
            self.label_msg.data = [label, sec, nsec]
            self.label_pub.publish(self.label_msg)
            self.get_logger().info(f"Published Label {label} at {sec}.{nsec:09d}")
            self.last_publish_time[label] = now_sec

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
