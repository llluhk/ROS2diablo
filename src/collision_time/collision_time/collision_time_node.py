import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from custom_msgs.msg import Collisionprediction  # <-- Updated import
import csv
import os

class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger_node')

        # CSV filenames and fieldnames
        self.button_csv = 'button_press_log.csv'
        self.pred_csv = 'collision_prediction_log.csv'

        self.button_fields = ['sec', 'nanosec']
        self.pred_fields = ['prediction', 'timestamp_sec', 'timestamp_nanosec']

        # Initialize CSV files with headers if not exist
        self._init_csv(self.button_csv, self.button_fields)
        self._init_csv(self.pred_csv, self.pred_fields)

        # Subscribers
        self.button_sub = self.create_subscription(Header, '/button_press_time', self.button_callback, 10)
        self.pred_sub = self.create_subscription(Collisionprediction, '/collision_prediction', self.prediction_callback, 10)

        self.get_logger().info("✅ 正在记录 button_press_time 和 collision_prediction")

    def _init_csv(self, filename, fields):
        if not os.path.isfile(filename):
            with open(filename, mode='w', newline='') as file:
                writer = csv.DictWriter(file, fieldnames=fields)
                writer.writeheader()

    def button_callback(self, msg: Header):
        sec = int(msg.stamp.sec)
        nanosec = int(msg.stamp.nanosec)
        with open(self.button_csv, mode='a', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=self.button_fields)
            writer.writerow({'sec': str(sec), 'nanosec': str(nanosec)})
        self.get_logger().info(f"📥 Button press logged: {sec}.{nanosec:09d}")

    def prediction_callback(self, msg: Collisionprediction):
        prediction = str(msg.prediction)
        timestamp_sec = str(msg.stamp.sec)
        timestamp_nanosec = str(msg.stamp.nanosec)

        row = {
            'prediction': prediction,
            'timestamp_sec': timestamp_sec,
            'timestamp_nanosec': timestamp_nanosec,
        }

        with open(self.pred_csv, mode='a', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=self.pred_fields)
            writer.writerow(row)

        self.get_logger().info(f"📥 Prediction logged: {row}")

def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
