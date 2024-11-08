#include "chassis_driver/chassis_driver_node.hpp"

#include "utilities/data_utils.hpp"
#include "utilities/utils.hpp"
#include <cmath>

using namespace utils;

namespace chassis_driver{

ChassisDriver::ChassisDriver(const rclcpp::NodeOptions& options) : ChassisDriver("", options) {}

ChassisDriver::ChassisDriver(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("chassis_driver_node", name_space, options),
wheel_radius(get_parameter("wheel_radius").as_double()),
attached_direction(get_parameter("attached_direction").as_double()),
linear_max_vel(get_parameter("linear_max.vel").as_double()),
angular_max_vel(get_parameter("angular_max.vel").as_double()),
cos45(std::sqrt(2.0) / 2.0)
{
    _subscription_vel = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        _qos,
        std::bind(&ChassisDriver::_subscriber_callback_vel, this, std::placeholders::_1)
    );
    _subscription_stop = this->create_subscription<std_msgs::msg::Empty>(
        "stop",
        _qos,
        std::bind(&ChassisDriver::_subscriber_callback_stop, this, std::placeholders::_1)
    );
    _subscription_restart = this->create_subscription<std_msgs::msg::Empty>(
        "restart",
        _qos,
        std::bind(&ChassisDriver::_subscriber_callback_restart, this, std::placeholders::_1)
    );

    publisher_can = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);

    RCLCPP_INFO(this->get_logger(), "車輪半径:%f  取り付け距離:%f", wheel_radius, attached_direction);
}

void ChassisDriver::_subscriber_callback_vel(const geometry_msgs::msg::Twist::SharedPtr msg){
    if(mode == Mode::stop) return;
    mode = Mode::cmd;

    this->prime_vel = *msg;
    rotate_vector(msg->linear.x, msg->linear.y, 0.0); // poseが手に入ったらmap座標系での角度を入力

    // 処理に使う一時データ
    const double omega = msg->angular.z;
    const double vx_p = constrain(this->prime_vel.linear.x, -linear_max_vel, linear_max_vel);
    const double vy_p = constrain(this->prime_vel.linear.y, -angular_max_vel, angular_max_vel);
    std::array<double, 4> wheel_vel;

    wheel_vel[0] = (vx_p*cos45 + vy_p*cos45 + attached_direction * omega);
    wheel_vel[1] = (-vx_p*cos45 + vy_p*cos45 - attached_direction * omega);
    wheel_vel[2] = (-vx_p*cos45 - vy_p*cos45 + attached_direction * omega);
    wheel_vel[3] = (vx_p*cos45 - vy_p*cos45 - attached_direction * omega);

    int index = 0;
    for (auto &vel : wheel_vel){
        vel / wheel_radius;
        send_motorvel(index, vel);
        index++;
    }
}

void ChassisDriver::rotate_vector(const double vx, const double vy, const double theta){
    const double cos = std::cos(theta);
    const double sin = std::sin(theta);
    this->prime_vel.linear.x = vx * cos - vy * sin;
    this->prime_vel.linear.y = vx * sin + vy * cos;
}

void ChassisDriver::send_motorvel(const int motor_num, const double vel){
    // 出版
    auto msg_can = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_can->canid = 0x102; // ベースID
    msg_can->canid = (msg_can->canid & 0xF0F) | (motor_num << 4);   // モータ番号によって変更
    msg_can->candlc = 4;

    uint8_t _candata[8];
    float_to_bytes(_candata, static_cast<float>(vel));

    for(int i=0; i<msg_can->candlc; i++) msg_can->candata[i]=_candata[i];
    publisher_can->publish(*msg_can);

    RCLCPP_INFO(this->get_logger(), "canid:%X  vel:%f", msg_can->canid, vel);
}

void ChassisDriver::_subscriber_callback_stop(const std_msgs::msg::Empty::SharedPtr msg){
    mode = Mode::stop;
    RCLCPP_INFO(this->get_logger(), "停止");
}
void ChassisDriver::_subscriber_callback_restart(const std_msgs::msg::Empty::SharedPtr msg){
    mode = Mode::stay;
    RCLCPP_INFO(this->get_logger(), "再稼働");
}

}  // namespace chassis_driver
