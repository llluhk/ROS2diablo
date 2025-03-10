#include "rclcpp/rclcpp.hpp"
#include <ctime>
#include "rclcpp/qos.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include "motion_msgs/msg/leg_motors.hpp"
#include "motion_msgs/msg/motion_ctrl.hpp"
#include "ception_msgs/msg/imu_euler.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "custom_msgs/msg/synced_data.hpp"  // 自定义消息类型

class DataSubscriberNode : public rclcpp::Node
{
public:
    DataSubscriberNode()
    : Node("sw_data_processing"), motion_ctrl_received_(false)
    {
        RCLCPP_INFO(this->get_logger(), "DataSubscriberNode constructed");
    }

    void init()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing DataSubscriberNode");

        auto qos = rclcpp::SystemDefaultsQoS();

        imu_subscriber_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(
            this->shared_from_this(), "/diablo/sensor/Imu", qos.get_rmw_qos_profile());

        motors_subscriber_ = std::make_shared<message_filters::Subscriber<motion_msgs::msg::LegMotors>>(
            this->shared_from_this(), "/diablo/sensor/Motors", qos.get_rmw_qos_profile());

        imu_euler_subscriber_ = std::make_shared<message_filters::Subscriber<ception_msgs::msg::IMUEuler>>(
            this->shared_from_this(), "/diablo/sensor/ImuEuler", qos.get_rmw_qos_profile());

        using SyncPolicy = message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Imu,
            motion_msgs::msg::LegMotors,
            ception_msgs::msg::IMUEuler>;

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(100), *imu_subscriber_, *motors_subscriber_, *imu_euler_subscriber_);

        sync_->registerCallback(std::bind(&DataSubscriberNode::syncCallback, this,
                                          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        motion_ctrl_subscriber_ = this->create_subscription<motion_msgs::msg::MotionCtrl>(
            "/diablo/MotionCmd", 10, std::bind(&DataSubscriberNode::motionCtrlCallback, this, std::placeholders::_1));

        synced_data_publisher_ = this->create_publisher<custom_msgs::msg::SyncedData>(
            "/synced_data", 10);

        RCLCPP_INFO(this->get_logger(), "DataSubscriberNode initialized successfully");
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Imu,
        motion_msgs::msg::LegMotors,
        ception_msgs::msg::IMUEuler>;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> imu_subscriber_;
    std::shared_ptr<message_filters::Subscriber<motion_msgs::msg::LegMotors>> motors_subscriber_;
    std::shared_ptr<message_filters::Subscriber<ception_msgs::msg::IMUEuler>> imu_euler_subscriber_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::Subscription<motion_msgs::msg::MotionCtrl>::SharedPtr motion_ctrl_subscriber_;

    rclcpp::Publisher<custom_msgs::msg::SyncedData>::SharedPtr synced_data_publisher_;

    motion_msgs::msg::MotionCtrl last_motion_ctrl_;
    bool motion_ctrl_received_;

    void motionCtrlCallback(const motion_msgs::msg::MotionCtrl::SharedPtr msg)
    {
        last_motion_ctrl_ = *msg;
        motion_ctrl_received_ = true;
        //RCLCPP_INFO(this->get_logger(), "Updated MotionCtrl: forward=%.2f, left=%.2f, up=%.2f, roll=%.2f, pitch=%.2f, leg_split=%.2f",
                    //last_motion_ctrl_.value.forward, last_motion_ctrl_.value.left, last_motion_ctrl_.value.up,
                    //last_motion_ctrl_.value.roll, last_motion_ctrl_.value.pitch, last_motion_ctrl_.value.leg_split);
    }

    void syncCallback(
        const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg,
        const motion_msgs::msg::LegMotors::ConstSharedPtr &motors_msg,
        const ception_msgs::msg::IMUEuler::ConstSharedPtr &imu_euler_msg)
    {
        // 使用 ROS2 系统时间作为统一时间戳
    
        rclcpp::Time unified_timestamp = this->now();

        // 构建 MotionCtrl 默认值（所有字段为 0）
        motion_msgs::msg::MotionCtrl default_motion_ctrl;
        default_motion_ctrl.value.forward = 0.0;
        default_motion_ctrl.value.left = 0.0;
        default_motion_ctrl.value.up = 0.0;
        default_motion_ctrl.value.roll = 0.0;
        default_motion_ctrl.value.pitch = 0.0;
        default_motion_ctrl.value.leg_split = 0.0;

        // 使用接收到的 MotionCtrl 数据（如果有），否则使用默认值
        motion_msgs::msg::MotionCtrl motion_ctrl_data = motion_ctrl_received_ ? last_motion_ctrl_ : default_motion_ctrl;

        // 构建同步后的数据消息
        custom_msgs::msg::SyncedData synced_data;
        synced_data.header.stamp = unified_timestamp; // 赋值统一时间戳
        synced_data.imu = *imu_msg;
        synced_data.leg_motors = *motors_msg;
        synced_data.imu_euler = *imu_euler_msg;
        synced_data.motion_ctrl = motion_ctrl_data;

        // 发布同步后的数据
        synced_data_publisher_->publish(synced_data);

        //RCLCPP_INFO(this->get_logger(), "Published synced data with MotionCtrl: forward=%.2f, left=%.2f, up=%.2f, roll=%.2f, pitch=%.2f, leg_split=%.2f",
                    //motion_ctrl_data.value.forward, motion_ctrl_data.value.left, motion_ctrl_data.value.up,
                    //motion_ctrl_data.value.roll, motion_ctrl_data.value.pitch, motion_ctrl_data.value.leg_split);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DataSubscriberNode>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

