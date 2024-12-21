#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/float64.hpp>
#include "socketcan_interface_msg/msg/socketcan_if.hpp"

#include "utilities/utils.hpp"
#include "controller/visibility_control.h"

namespace controller{

class Controller : public rclcpp::Node {
public:
    CONTROLLER_PUBLIC
    explicit Controller(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    CONTROLLER_PUBLIC
    explicit Controller(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _subscription_joy;
    rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _subscription_ems;

    void _subscriber_callback_joy(const sensor_msgs::msg::Joy::SharedPtr msg);
    void _subscriber_callback_ems(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_vel;
    // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vel;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_stop;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_restart;
    // rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_emergency;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_autonomous;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_cybergear;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_cybergear_reset;
    rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr publisher_can;

    rclcpp::QoS _qos = rclcpp::QoS(10);

    const double linear_max_vel;
    const double angular_max_vel;

    bool is_autonomous = false;
    bool is_emergency = false;
    bool cybergear_hit = false;

    utils::UpEdge upedge_emergency;
    utils::UpEdge upedge_auto;
    utils::UpEdge upedge_restart;
    utils::UpEdge upedge_cybergear;
    utils::UpEdge upedge_cybergear_reset;

    enum class Axes{
        L_x,
        L_y,
        R_x,
        R_y,
        R_Trigger,
        L_Trigger,
        left_and_right,
        up_and_down
    };
    enum class Buttons{
        A,
        B,
        null0,
        X,
        Y,
        null1,
        L,
        R,
        null2,
        null3,
        View,
        Menu,
        null4,
        L_Stick,
        R_Stick,
        Share
    };

};

}  // namespace controller
