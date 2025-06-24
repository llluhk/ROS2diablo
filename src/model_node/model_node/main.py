import joblib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray  # Still used for input
from custom_msgs.msg import Collisionprediction
import numpy as np
import time

# Paths (change if deploying to robot)
MODEL_PATH = '/home/pc/diablo_collision_pc_ws/src/model_node/checkpoints/mlp.pkl'
SCALER_PATH = "/home/pc/diablo_collision_pc_ws/src/model_node/checkpoints/scaler.pkl"

class CollisionDetectionNode(Node):
    def __init__(self):
        super().__init__('collision_detection_node')

        try:
            self.model = joblib.load(MODEL_PATH)
            self.scaler = joblib.load(SCALER_PATH)
            self.get_logger().info("模型加载成功（MLP 和归一化器）")
        except Exception as e:
            self.get_logger().error(f"模型加载失败: {e}")
            self.model = None
            self.scaler = None

        # Subscribe to preprocessed sensor data
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/processed_data',
            self.listener_callback,
            10
        )

        # Publish prediction as custom message
        self.publisher = self.create_publisher(
            Collisionprediction,
            '/collision_prediction',
            10
        )

        self.durations = []
        self.start_time = time.time()
        self.get_logger().info("Collision Detection Node 已启动，等待数据...")

    def listener_callback(self, msg):
        if self.model is None or self.scaler is None:
            self.get_logger().error("模型未加载，无法预测！")
            return

        # Convert message to numpy array
        sensor_data = np.array(msg.data)

        if sensor_data.ndim == 1:
            sensor_data = sensor_data.reshape(1, -1)

        expected_features = self.model.n_features_in_
        if sensor_data.shape[1] != expected_features:
            self.get_logger().error(f"⚠️ 数据维度错误！期望 {expected_features} 维，实际 {sensor_data.shape[1]} 维")
            return

        try:
            sensor_data = self.scaler.transform(sensor_data)
        except Exception as e:
            self.get_logger().error(f"归一化失败: {e}")
            return

        try:
            start = time.perf_counter()
            prediction = self.model.predict(sensor_data)[0]
            end = time.perf_counter()

            duration = (end - start) * 1000
            self.durations.append(duration)

            if prediction in [1, 2, 3]:
                now = self.get_clock().now().to_msg()
                prediction_msg = Collisionprediction()
                prediction_msg.prediction = int(prediction)
                prediction_msg.stamp = now

                self.get_logger().warn(
                    f"碰撞检测: 发生碰撞！(类别: {prediction} 时间: {now.sec}.{now.nanosec:09d})"
                )
                self.publisher.publish(prediction_msg)

            if time.time() - self.start_time >= 10:
                avg_time = np.mean(self.durations)
                self.get_logger().info(f"过去 10 秒的平均预测时间: {avg_time:.3f}ms")
                self.durations = []
                self.start_time = time.time()

        except Exception as e:
            self.get_logger().error(f"预测错误: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CollisionDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
