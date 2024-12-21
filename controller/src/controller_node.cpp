#include "controller/controller_node.hpp"
#include "utilities/data_utils.hpp"

using namespace std;
using namespace utils;

namespace controller{

Controller::Controller(const rclcpp::NodeOptions& options) : Controller("", options) {}

Controller::Controller(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("controller_node", name_space, options),
linear_max_vel(get_parameter("linear_max.vel").as_double()),
angular_max_vel(dtor(get_parameter("angular_max.vel").as_double())),
is_autonomous(get_parameter("autonomous_flag").as_bool())
{
    _subscription_joy = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy",
        _qos,
        std::bind(&Controller::_subscriber_callback_joy, this, std::placeholders::_1)
    );
    _subscription_ems = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
        "can_rx_00F",
        _qos,
        std::bind(&Controller::_subscriber_callback_ems, this, std::placeholders::_1)
    );

    publisher_vel = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", _qos);
    publisher_stop = this->create_publisher<std_msgs::msg::Empty>("stop", _qos);
    publisher_restart = this->create_publisher<std_msgs::msg::Empty>("restart", _qos);
    // publisher_emergency = this->create_publisher<std_msgs::msg::Bool>("emergency", _qos);
    publisher_autonomous = this->create_publisher<std_msgs::msg::Bool>("autonomous", _qos);
    publisher_cybergear = this->create_publisher<std_msgs::msg::Float64>("motor/pos", _qos);
    publisher_cybergear_reset = this->create_publisher<std_msgs::msg::Empty>("motor/reset", _qos);
    publisher_can = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);


    // 駆動系に電源が行っている可能性もあるのでリスタートする
    publisher_restart->publish(*std::make_shared<std_msgs::msg::Empty>());
}

void Controller::_subscriber_callback_joy(const sensor_msgs::msg::Joy::SharedPtr msg){

    if(upedge_emergency(msg->buttons[static_cast<int>(Buttons::Share)])){
        is_emergency = !is_emergency;
        auto msg_can = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_can->candlc = 0;

        if(is_emergency){
            msg_can->canid = 0x00;
            RCLCPP_INFO(this->get_logger(), "緊急停止 解除");
        }
        else{
            msg_can->canid = 0x01;
            RCLCPP_INFO(this->get_logger(), "緊急停止 !!");
        }
        // 出版
        publisher_can->publish(*msg_can);
    }
    // 自律か手動か
    if(upedge_auto(msg->buttons[static_cast<int>(Buttons::View)])){
        auto msg_autonomous = std::make_shared<std_msgs::msg::Bool>();
        msg_autonomous->data = is_autonomous = !is_autonomous;
        publisher_autonomous->publish(*msg_autonomous);
        RCLCPP_INFO(this->get_logger(), "自動フラグ : %d", msg_autonomous->data);
    }
    // リスタート
    if(upedge_restart(msg->buttons[static_cast<int>(Buttons::Menu)])){
        publisher_restart->publish(*std::make_shared<std_msgs::msg::Empty>());
        RCLCPP_INFO(this->get_logger(), "再稼働");
    }
    // 打ち返しの手動
    if(upedge_cybergear(msg->buttons[static_cast<int>(Buttons::B)])){
        auto msg = std::make_shared<std_msgs::msg::Float64>();
        if(!cybergear_hit){
            msg->data = dtor(0.0);
            cybergear_hit = true;
            RCLCPP_INFO(this->get_logger(), "射出");
        }else{
            msg->data = dtor(-120.0);
            cybergear_hit = false;
            RCLCPP_INFO(this->get_logger(), "収納");
        }
        publisher_cybergear->publish(*msg);
    }
    // サイバーギアのリセット
    if(upedge_cybergear_reset(msg->buttons[static_cast<int>(Buttons::L_Stick)])){
        publisher_cybergear_reset->publish(*std::make_shared<std_msgs::msg::Empty>());
    }
    // 手動の場合、速度指令値を送る
    if(!is_autonomous){
        auto msg_vel = std::make_shared<geometry_msgs::msg::Twist>();
        msg_vel->linear.x = linear_max_vel * msg->axes[static_cast<int>(Axes::L_y)];
        msg_vel->linear.y = linear_max_vel * msg->axes[static_cast<int>(Axes::L_x)];
        msg_vel->angular.z = angular_max_vel * msg->axes[static_cast<int>(Axes::R_x)];
        publisher_vel->publish(*msg_vel);
    }

}

void Controller::_subscriber_callback_ems(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
    if(msg->candata[0] == 0x01){  //緊急停止ON
        publisher_stop->publish(*std::make_shared<std_msgs::msg::Empty>());
        // is_emergency = true;
    }
    else if(msg->candata[0] == 0x00){ //緊急停止OFF
        // is_emergency = false;
    }
    RCLCPP_INFO(this->get_logger(), "緊急停止フラグ : %d", msg->candata[0]);
}

}  // namespace controller
