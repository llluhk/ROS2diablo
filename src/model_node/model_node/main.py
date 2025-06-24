import joblib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from custom_msgs.msg import Collisionprediction
import numpy as np
import time

# --- Update paths to your saved cascade models ---
STAGE1_MODEL_PATH = '/home/pc/diablo_collision_pc_ws/src/model_node/checkpoints/clf_stage1.pkl'
STAGE2_MODEL_PATH = '/home/pc/diablo_collision_pc_ws/src/model_node/checkpoints/clf_stage2.pkl'
SCALER_PATH = '/home/pc/diablo_collision_pc_ws/src/model_node/checkpoints/scaler.pkl'

class CollisionDetectionNode(Node):
    def __init__(self):
        super().__init__('collision_detection_node')

        try:
            self.clf_stage1 = joblib.load(STAGE1_MODEL_PATH)
            self.clf_stage2 = joblib.load(STAGE2_MODEL_PATH)
            self.scaler = joblib.load(SCALER_PATH)
            self.get_logger().info("✅ 模型加载成功（Stage 1, Stage 2, Scaler）")
        except Exception as e:
            self.get_logger().error(f"❌ 模型加载失败: {e}")
            self.clf_stage1 = None
            self.clf_stage2 = None
            self.scaler = None

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/processed_data',
            self.listener_callback,
            10
        )

        self.publisher = self.create_publisher(
            Collisionprediction,
            '/collision_prediction',
            10
        )

        self.durations = []
        self.start_time = time.time()
        self.get_logger().info("Collision Detection Node 启动，等待传感器数据...")

    def listener_callback(self, msg):
        if self.clf_stage1 is None or self.clf_stage2 is None or self.scaler is None:
            self.get_logger().error("模型未加载，无法进行预测！")
            return

        sensor_data = np.array(msg.data)
        if sensor_data.ndim == 1:
            sensor_data = sensor_data.reshape(1, -1)

        expected_features = self.clf_stage1.n_features_in_
        if sensor_data.shape[1] != expected_features:
            self.get_logger().error(f"⚠️ 输入特征维度不符！期望: {expected_features}, 实际: {sensor_data.shape[1]}")
            return

        try:
            sensor_data_scaled = self.scaler.transform(sensor_data)
        except Exception as e:
            self.get_logger().error(f"归一化失败: {e}")
            return

        try:
            start = time.perf_counter()
            stage1_pred = self.clf_stage1.predict(sensor_data_scaled)[0]
            end = time.perf_counter()

            duration = (end - start) * 1000
            self.durations.append(duration)

            if stage1_pred == 1:
                # Stage 2: predict collision direction
                direction_pred = self.clf_stage2.predict(sensor_data_scaled)[0]

                now = self.get_clock().now().to_msg()
                prediction_msg = Collisionprediction()
                prediction_msg.prediction = int(direction_pred)
                prediction_msg.stamp = now

                self.get_logger().warn(
                    f"🚨 检测到碰撞！方向: {direction_pred} 时间: {now.sec}.{now.nanosec:09d}"
                )
                self.publisher.publish(prediction_msg)

            if time.time() - self.start_time >= 10:
                avg_time = np.mean(self.durations)
                self.get_logger().info(f"⏱️ 平均预测时间 (过去10秒): {avg_time:.3f}ms")
                self.durations = []
                self.start_time = time.time()

        except Exception as e:
            self.get_logger().error(f"❌ 预测失败: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CollisionDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
