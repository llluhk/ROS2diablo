#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from datetime import datetime
import csv
from pathlib import Path

class LabelCsvSaver(Node):
    def __init__(self):
        super().__init__('label_csv_saver_node')
        self.get_logger().info("LabelCsvSaver node started.")

        self.subscription = self.create_subscription(
            Int32MultiArray,
            'collision_label',
            self.label_callback,
            10
        )

        self.setup_csv()

        # Track last write time (in ROS time)
        self.last_save_time = None

        # Register shutdown hook
        rclpy.get_default_context().on_shutdown(self.cleanup)

    def setup_csv(self):
        directory = Path("/home/pc/BA_data_record/")
        directory.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = directory / f"label_{timestamp}.csv"

        try:
            self.csv_file = self.csv_path.open(mode='a', newline='')
            self.csv_writer = csv.writer(self.csv_file)

            if self.csv_file.tell() == 0:
                self.csv_writer.writerow(["Key", "Timestamp_sec", "Timestamp_nanosec"])
        except Exception as e:
            self.get_logger().error(f"Failed to open CSV file: {e}")
            raise

    def label_callback(self, msg):
        now = self.get_clock().now()

        # If we've saved a label before, check if 2 seconds have passed
        if self.last_save_time is not None:
            time_diff = (now - self.last_save_time).nanoseconds * 1e-9
            if time_diff < 2.0:
                #self.get_logger().info(f"Ignored label due to 2s throttle. Only {time_diff:.2f}s passed.")
                return

        try:
            label, timestamp_sec, timestamp_nanosec = msg.data
            self.csv_writer.writerow([label, timestamp_sec, timestamp_nanosec])
            self.csv_file.flush()

            self.get_logger().info(
                f"Saved Label: {label}, Timestamp: {timestamp_sec}.{timestamp_nanosec:09d}"
            )

            self.last_save_time = now  # Update last save time
        except Exception as e:
            self.get_logger().error(f"Error writing to CSV: {e}")

    def cleanup(self):
        if hasattr(self, "csv_file") and not self.csv_file.closed:
            self.csv_file.close()
            self.get_logger().info(f"CSV file closed: {self.csv_path}")


def main(args=None):
    rclpy.init(args=args)
    node = LabelCsvSaver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Shutting down...")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()
        print("Exit!")


if __name__ == '__main__':
    main()
