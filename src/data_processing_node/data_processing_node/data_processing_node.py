import numpy as np
from collections import deque
import rclpy
from rclpy.node import Node
from custom_msgs.msg import SyncedData  # Replace with your actual message type
from std_msgs.msg import Float32MultiArray

class DataProcessingNode(Node):
    def __init__(self):
        super().__init__('data_processing_node')

        # Parameters
        self.n_step = 4
        self.feature_dim = 10  # Total number of features to extract
        self.buffer = deque(maxlen=self.n_step)

        # ROS2 subscriptions and publishers
        self.subscription = self.create_subscription(
            SyncedData, '/synced_data', self.data_callback, 100
        )
        self.publisher = self.create_publisher(Float32MultiArray, '/processed_data', 10)

        self.get_logger().info("DataProcessingNode initialized and waiting for /synced_data...")

    def data_callback(self, msg):
        self.get_logger().info("Received message from /synced_data")

        raw_features = self.extract_features(msg).reshape(1, -1)
        self.get_logger().info(f"Extracted features shape: {raw_features.shape}, features: {raw_features.tolist()}")

        self.buffer.append(raw_features)
        self.get_logger().info(f"Updated buffer size: {len(self.buffer)} / {self.n_step}")

        if len(self.buffer) >= self.n_step:
            window = list(self.buffer)[-self.n_step:]
            combined = np.concatenate(window, axis=1)
            self.get_logger().info(f"Publishing processed window with shape: {combined.shape}")
            self.publish_single_sample(combined)

    def extract_features(self, msg):
        try:
            imu_data = [
                msg.imu.angular_velocity.y,
                msg.imu.angular_velocity.z,
                msg.imu.linear_acceleration.x,
                msg.imu.linear_acceleration.y,
                #msg.imu.linear_acceleration.z
            ]

            motor_data = [
                msg.leg_motors.left_wheel_vel,
                msg.leg_motors.left_wheel_iq,
                msg.leg_motors.right_wheel_vel,
                msg.leg_motors.right_wheel_iq
            ]

            motion_ctrl_data = [
                msg.motion_ctrl.motion_ctrl.value.forward,
                msg.motion_ctrl.motion_ctrl.value.left
            ]

            features = imu_data + motor_data + motion_ctrl_data

            if len(features) != self.feature_dim:
                self.get_logger().error(
                    f"Feature length mismatch: expected {self.feature_dim}, got {len(features)}"
                )

            return np.array(features, dtype=np.float32)

        except Exception as e:
            self.get_logger().error(f"Error extracting features: {str(e)}")
            return np.zeros((self.feature_dim,), dtype=np.float32)

    def publish_single_sample(self, sample):
        msg = Float32MultiArray()
        msg.data = sample.flatten().tolist()
        self.publisher.publish(msg)
        self.get_logger().info(f"Published Float32MultiArray with length {len(msg.data)}")

def main(args=None):
    rclpy.init(args=args)
    node = DataProcessingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
