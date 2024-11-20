#include "pose_pid/pose_pid_node.hpp"

#include "utilities/utils.hpp"
#include <cmath>

using namespace utils;

namespace pose_pid{

PosePID::PosePID(const rclcpp::NodeOptions& options) : PosePID("", options) {}

PosePID::PosePID(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("pose_pid_node", name_space, options),
interval_ms(get_parameter("interval_ms").as_int()),
pid_linear_x(get_parameter("interval_ms").as_int()),
pid_linear_y(get_parameter("interval_ms").as_int()),
pid_angular(get_parameter("interval_ms").as_int()),
linear_max_vel(get_parameter("linear_max.vel").as_double()),
angular_max_vel(dtor(get_parameter("angular_max.vel").as_double())),
allowed_area(get_parameter("allowed_area").as_double_array())
{
    _subscription_stop = this->create_subscription<std_msgs::msg::Empty>(
        "stop",
        _qos,
        std::bind(&PosePID::_subscriber_callback_stop, this, std::placeholders::_1)
    );
    _subscription_restart = this->create_subscription<std_msgs::msg::Empty>(
        "restart",
        _qos,
        std::bind(&PosePID::_subscriber_callback_restart, this, std::placeholders::_1)
    );
    _subscription_autonomous = this->create_subscription<std_msgs::msg::Bool>(
        "autonomous",
        _qos,
        std::bind(&PosePID::_subscriber_callback_autonomous, this, std::placeholders::_1)
    );
    _subscription_target = this->create_subscription<geometry_msgs::msg::Pose2D>(
        "target_pose",
        _qos,
        std::bind(&PosePID::_subscriber_callback_target, this, std::placeholders::_1)
    );
    _subscription_selfpose = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "self_pose",
        _qos,
        std::bind(&PosePID::_subscriber_callback_selfpose, this, std::placeholders::_1)
    );

    publisher_vel = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", _qos);

    // ゲイン設定
    const auto linear_gain = get_parameter("linear_pidgain").as_double_array();
    const auto angular_gain = get_parameter("angular_pidgain").as_double_array();
    pid_linear_x.gain(linear_gain.at(0), linear_gain.at(1), linear_gain.at(2));
    pid_linear_y.gain(linear_gain.at(0), linear_gain.at(1), linear_gain.at(2));
    pid_angular.gain(angular_gain.at(0), angular_gain.at(1), angular_gain.at(2));

    _pub_timer = this->create_wall_timer(
        std::chrono::milliseconds(interval_ms),
        [this] { if(is_autonomous)_publisher_callback(); }
    );

}

void PosePID::_publisher_callback(){
    if(mode == Mode::stop) return;
    mode = Mode::run;
    if(target == nullptr) return;
    if(self_pose == nullptr) return;

    const double current_yaw = std::atan2(
        2.0 * (self_pose->pose.orientation.w * self_pose->pose.orientation.z),
        1.0 - 2.0 * (self_pose->pose.orientation.z * self_pose->pose.orientation.z));

    auto msg_vel = std::make_shared<geometry_msgs::msg::Twist>();
    msg_vel->linear.x = pid_linear_x.cycle(self_pose->pose.position.x, target->x)*linear_max_vel;
    msg_vel->linear.y = pid_linear_y.cycle(self_pose->pose.position.y, target->y)*linear_max_vel;
    msg_vel->angular.z = pid_angular.cycle(current_yaw, target->theta)*angular_max_vel;

    // 出版
    publisher_vel->publish(*msg_vel);
}

void PosePID::_subscriber_callback_target(const geometry_msgs::msg::Pose2D::SharedPtr msg){
    std::shared_ptr<geometry_msgs::msg::Pose2D> target_ = msg;
    target_->x = constrain(target_->x, allowed_area.at(0), allowed_area.at(1));
    target_->y = constrain(target_->y, allowed_area.at(2), allowed_area.at(3));
    target_->theta = constrain(target_->theta, -pi<double>(), pi<double>());
    this->target = target_;
    RCLCPP_INFO(this->get_logger(), "目標姿勢入力  x:%f  y:%f  theta:%f", target->x, target->y, target->theta);
}
void PosePID::_subscriber_callback_selfpose(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    self_pose = msg;
}
void PosePID::reset(){
    target = nullptr;
    pid_linear_x.reset();
    pid_linear_y.reset();
    pid_angular.reset();
}


/*基幹入力*/
void PosePID::_subscriber_callback_stop(const std_msgs::msg::Empty::SharedPtr msg){
    mode = Mode::stop;

    RCLCPP_INFO(this->get_logger(), "停止");
}
void PosePID::_subscriber_callback_restart(const std_msgs::msg::Empty::SharedPtr msg){
    mode = Mode::stay;
    reset();

    RCLCPP_INFO(this->get_logger(), "再稼働");
}
void PosePID::_subscriber_callback_autonomous(const std_msgs::msg::Bool::SharedPtr msg){
    is_autonomous = msg->data;
    reset();

    RCLCPP_INFO(this->get_logger(), "自動：%d", is_autonomous);
}

}  // namespace pose_pid
