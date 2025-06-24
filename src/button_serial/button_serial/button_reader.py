import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
import serial

class ButtonTimestampPublisher(Node):
    def __init__(self):
        super().__init__('button_timestamp_publisher')
        self.publisher_time = self.create_publisher(Header, 'button_press_time', 10)
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.timer = self.create_timer(0.1, self.read_serial)
        self.get_logger().info("ButtonTimestampPublisher node has started")

    def read_serial(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            if line.isdigit() and int(line) == 0:  # Button pressed (0)
                now = self.get_clock().now()
                header_msg = Header()
                header_msg.stamp = now.to_msg()  # converts ROS time to builtin_interfaces/Time
                header_msg.frame_id = 'button_press'  # Optional: identify the frame or source
                self.publisher_time.publish(header_msg)
                self.get_logger().info(f'Published button press header timestamp: {header_msg.stamp.sec}.{header_msg.stamp.nanosec:09d}')

def main(args=None):
    rclpy.init(args=args)
    node = ButtonTimestampPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
