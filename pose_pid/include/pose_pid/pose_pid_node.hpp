#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "utilities/position_pid.hpp"

#include "pose_pid/visibility_control.h"

namespace pose_pid{

class PosePID : public rclcpp::Node {
public:
    POSE_PID_PUBLIC
    explicit PosePID(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    POSE_PID_PUBLIC
    explicit PosePID(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _subscription_stop;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _subscription_restart;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _subscription_autonomous;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr _subscription_target;
    // rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr _subscription_target;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _subscription_selfpose;
    rclcpp::TimerBase::SharedPtr _pub_timer;

    void _subscriber_callback_stop(const std_msgs::msg::Empty::SharedPtr msg);
    void _subscriber_callback_restart(const std_msgs::msg::Empty::SharedPtr msg);
    void _subscriber_callback_autonomous(const std_msgs::msg::Bool::SharedPtr msg);
    void _subscriber_callback_target(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void _subscriber_callback_selfpose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void _publisher_callback();

    void reset();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vel;

    rclcpp::QoS _qos = rclcpp::QoS(10);

    controller::PositionPid pid_linear_x;
    controller::PositionPid pid_linear_y;
    controller::PositionPid pid_angular;

    // 定数
    const int interval_ms;
    const double linear_max_vel;
    const double angular_max_vel;
    const std::vector<double> allowed_area;

    //変数
    bool is_autonomous = false;
    std::shared_ptr<geometry_msgs::msg::PointStamped> target;
    std::shared_ptr<geometry_msgs::msg::PoseStamped> self_pose;

    // 動作モード
    enum class Mode{
        run,
        stay,
        stop
    } mode = Mode::stay;

};

}  // namespace pose_pid
