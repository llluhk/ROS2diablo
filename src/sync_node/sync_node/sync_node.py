####################     Description         ###############
# This node subscribes to topics from timestamp_processing_node 
# This node synchronizes the topics using ROS2's Approximate Time Synchronizer.

# ADD dependencies
import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer

# Add interfaces
from sensor_msgs.msg import Imu 
from motion_msgs.msg import LegMotors
from motion_msgs.msg import MotionCtrlstamp
from ception_msgs.msg import IMUEuler
from custom_msgs.msg import SyncedData  
from builtin_interfaces.msg import Time  # Fix timestamp conversion

class TimeSyncNode(Node):

    def __init__(self):
        super().__init__('sync_node')

        # Corrected Subscriber usage
        self.imu_sub = Subscriber(self, Imu, 'synced/imu')
        self.leg_motors_sub = Subscriber(self, LegMotors, 'synced/leg_motors')
        self.motion_ctrl_sub = Subscriber(self, MotionCtrlstamp, 'synced/motion_ctrl')
        self.imu_euler_sub = Subscriber(self, IMUEuler, 'synced/imu_euler')
        # Print subscribed topics


        self.synced_data_publisher = self.create_publisher(SyncedData, 'synced_data', 10)

        queue_size = 10
        max_delay = 0.05
        self.time_sync = ApproximateTimeSynchronizer(
            [self.imu_sub, self.leg_motors_sub, self.motion_ctrl_sub, self.imu_euler_sub],
            queue_size, max_delay
        )

        self.time_sync.registerCallback(self.sync_callback)  # Fixed function name

    def sync_callback(self, imu_msg, motors_msg, motion_ctrl_msg, imu_euler_msg):
        #self.get_logger().info("Received messages from sensors")
        unified_timestamp = self.get_clock().now()

        synced_data = SyncedData()
        synced_data.header.stamp = Time(sec=unified_timestamp.seconds_nanoseconds()[0], 
                                        nanosec=unified_timestamp.seconds_nanoseconds()[1])
        synced_data.imu = imu_msg
        synced_data.leg_motors = motors_msg
        synced_data.imu_euler = imu_euler_msg
        synced_data.motion_ctrl = motion_ctrl_msg

        self.synced_data_publisher.publish(synced_data)
        # Print publish confirmation
        # self.get_logger().info(
            #f"[Published] SyncedData at {synced_data.header.stamp.sec}."
            #f"{synced_data.header.stamp.nanosec:09d}"
        #)

def main(args=None):
    rclpy.init(args=args)
    node = TimeSyncNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
