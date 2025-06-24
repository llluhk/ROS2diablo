############################# visualization the synchronization algorithm ######################################
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from motion_msgs.msg import LegMotors, MotionCtrlstamp
from custom_msgs.msg import SyncedData

import csv
import os
import threading


class TimestampLoggerNode(Node):
    def __init__(self):
        super().__init__('timestamp_acc_logger_node')

        self.csv_file = 'timestamps_log.csv'
        self.lock = threading.Lock()

        file_exists = os.path.exists(self.csv_file)
        self.file = open(self.csv_file, mode='a', newline='', buffering=1)
        self.writer = csv.writer(self.file)

        if not file_exists:
            self.writer.writerow(['topic', 'sec', 'nanosec', 'linear_acceleration_x'])

        self.create_subscription(Imu, 'synced/imu', self.imu_callback, 10)
        self.create_subscription(MotionCtrlstamp, 'synced/motion_ctrl', self.motion_ctrl_callback, 10)
        self.create_subscription(SyncedData, 'synced_data', self.synced_data_callback, 10)

    def log_timestamp(self, topic_name, sec, nanosec, linear_acc_x=None):
        with self.lock:
            self.writer.writerow([topic_name, sec, nanosec, linear_acc_x])
            self.file.flush()
        self.get_logger().info(f"[{topic_name}] sec: {sec}, nanosec: {nanosec}, acc_x: {linear_acc_x}")

    def imu_callback(self, msg):
        self.log_timestamp('synced/imu', msg.header.stamp.sec, msg.header.stamp.nanosec, msg.linear_acceleration.x)

    def motion_ctrl_callback(self, msg):
        self.log_timestamp('synced/motion_ctrl', msg.header.stamp.sec, msg.header.stamp.nanosec)

    def synced_data_callback(self, msg):
        self.log_timestamp('synced_data', msg.header.stamp.sec, msg.header.stamp.nanosec, msg.imu.linear_acceleration.x)

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TimestampLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
