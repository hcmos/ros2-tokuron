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
stop_canid(static_cast<uint32_t>(get_parameter("stop_canid").as_int())),
restart_canid(static_cast<uint32_t>(get_parameter("restart_canid").as_int())),
motor_rev_canid(static_cast<uint32_t>(get_parameter("motor_rev_canid").as_int())),
encoder_rev_canid(static_cast<uint32_t>(get_parameter("encoder_rev_canid").as_int())),
velocity_canid(static_cast<uint32_t>(get_parameter("velocity_canid").as_int())),
pidgain_canid(static_cast<uint32_t>(get_parameter("pidgain_canid").as_int())),
linear_max_vel(get_parameter("linear_max.vel").as_double()),
angular_max_vel(dtor(get_parameter("angular_max.vel").as_double())),
sqrt2over2(std::sqrt(2.0) / 2.0)
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

    // モータ・エンコーダの正逆設定
    auto motor_rev = get_parameter("motor_rev").as_integer_array();
    auto encoder_rev = get_parameter("encoder_rev").as_integer_array();
    int index = 0;
    for(auto rev : motor_rev){
        if(rev){
            send_motor_rev(index, static_cast<bool>(rev));
        }
        index++;
    }
    index = 0;
    for(auto rev : encoder_rev){
        if(rev){
            send_encoder_rev(index, static_cast<bool>(rev));
        }
        index++;
    }

    // PIDゲインの設定
    send_pidgain(0, get_parameter("pidgain0").as_double_array());
    send_pidgain(1, get_parameter("pidgain1").as_double_array());
    send_pidgain(2, get_parameter("pidgain2").as_double_array());
    send_pidgain(3, get_parameter("pidgain3").as_double_array());

    RCLCPP_INFO(this->get_logger(), "車輪半径:%f  取り付け距離:%f", wheel_radius, attached_direction);
    RCLCPP_INFO(this->get_logger(), "最大並進速度:%f  最大回転速度:%f", linear_max_vel, angular_max_vel);
}

void ChassisDriver::_subscriber_callback_vel(const geometry_msgs::msg::Twist::SharedPtr msg){
    if(mode == Mode::stop) return;
    mode = Mode::cmd;

/*座標系変換*/
    this->prime_vel = *msg;
    rotate_vector(msg->linear.x, msg->linear.y, 0.0); // poseが手に入ったらmap座標系での角度を入力
    // この後にprime_velを操作すること

/*処理に使う一時データ*/
    const double omega = constrain(msg->angular.z, -angular_max_vel, angular_max_vel);
    // 並進の処理
    const double linear_length = std::sqrt(this->prime_vel.linear.x*this->prime_vel.linear.x + this->prime_vel.linear.y*this->prime_vel.linear.y);
    if(linear_length > linear_max_vel){
        this->prime_vel.linear.x *= linear_length / linear_max_vel;
        this->prime_vel.linear.y *= linear_length / linear_max_vel;
    }
    // 上のスカラー制限処理で問題ないが，最終的なフィルターとして置いておく
    const double vx_p = constrain(this->prime_vel.linear.x, -linear_max_vel, linear_max_vel);
    const double vy_p = constrain(this->prime_vel.linear.y, -linear_max_vel, linear_max_vel);

    std::array<double, 4> wheel_vel;

/*速度の計算*/
    wheel_vel[0] = (vx_p*sqrt2over2 + vy_p*sqrt2over2 - attached_direction * omega);    //cos45とsin45はsqrt(2)/2なため
    wheel_vel[1] = (-vx_p*sqrt2over2 + vy_p*sqrt2over2 - attached_direction * omega);
    wheel_vel[2] = (-vx_p*sqrt2over2 - vy_p*sqrt2over2 - attached_direction * omega);
    wheel_vel[3] = (vx_p*sqrt2over2 - vy_p*sqrt2over2 - attached_direction * omega);

    int index = 0;
    for (auto &vel : wheel_vel){
        vel = vel / wheel_radius;
        send_motorvel(index, vel);
        index++;
    }
}

void ChassisDriver::rotate_vector(const double vx, const double vy, const double theta){
    const double cos = std::cos(theta);
    const double sin = std::sin(theta);
    const double vx_p = -vy;
    const double vy_p = vx;
    this->prime_vel.linear.x = vx_p * cos - vy_p * sin;
    this->prime_vel.linear.y = vx_p * sin + vy_p * cos;
}

/*CANモータ設定*/
void ChassisDriver::send_motorvel(const int motor_num, const double vel){
    // 出版
    auto msg_can = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_can->canid = velocity_canid; // ベースID
    msg_can->canid = (msg_can->canid & 0xF0F) | (motor_num << 4);   // モータ番号によって変更
    msg_can->candlc = 4;

    uint8_t _candata[8];
    float_to_bytes(_candata, static_cast<float>(vel));

    for(int i=0; i<msg_can->candlc; i++) msg_can->candata[i]=_candata[i];
    publisher_can->publish(*msg_can);

    // RCLCPP_INFO(this->get_logger(), "canid:%X  vel:%f", msg_can->canid, vel);
}
void ChassisDriver::send_motor_rev(const int motor_num, const bool rev){
    // 出版
    auto msg_can = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_can->canid = motor_rev_canid; // ベースID
    msg_can->canid = (msg_can->canid & 0xF0F) | (motor_num << 4);   // モータ番号によって変更
    msg_can->candlc = 1;

    msg_can->candata[0] = rev;
    publisher_can->publish(*msg_can);
}
void ChassisDriver::send_encoder_rev(const int motor_num, const bool rev){
    // 出版
    auto msg_can = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_can->canid = encoder_rev_canid; // ベースID
    msg_can->canid = (msg_can->canid & 0xF0F) | (motor_num << 4);   // モータ番号によって変更
    msg_can->candlc = 1;

    msg_can->candata[0] = rev;
    publisher_can->publish(*msg_can);
}
void ChassisDriver::send_pidgain(const int motor_num, const std::vector<double> gain){
    // 出版
    auto msg_can = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_can->canid = pidgain_canid; // ベースID
    msg_can->canid = (msg_can->canid & 0xF0F) | (motor_num << 4);   // モータ番号によって変更
    msg_can->candlc = 5;

    for(int i=0; i<3; i++){
        uint8_t _candata[8];

        _candata[0] = static_cast<uint8_t>(i);
        float_to_bytes(_candata+1, static_cast<float>(gain.at(i)));

        for(int i=0; i<msg_can->candlc; i++) msg_can->candata[i]=_candata[i];
        publisher_can->publish(*msg_can);
    }
}

/*基幹入力*/
void ChassisDriver::_subscriber_callback_stop(const std_msgs::msg::Empty::SharedPtr msg){
    mode = Mode::stop;
    // 出版
    for(int motor_num=0; motor_num<4; motor_num++){
        auto msg_can = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_can->canid = stop_canid; // ベースID
        msg_can->canid = (msg_can->canid & 0xF0F) | (motor_num << 4);   // モータ番号によって変更
        msg_can->candlc = 0;
        publisher_can->publish(*msg_can);
    }

    RCLCPP_INFO(this->get_logger(), "停止");
}
void ChassisDriver::_subscriber_callback_restart(const std_msgs::msg::Empty::SharedPtr msg){
    mode = Mode::stay;
    // 出版
    for(int motor_num=0; motor_num<4; motor_num++){
        auto msg_can = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_can->canid = restart_canid; // ベースID
        msg_can->canid = (msg_can->canid & 0xF0F) | (motor_num << 4);   // モータ番号によって変更
        msg_can->candlc = 0;
        publisher_can->publish(*msg_can);
    }

    RCLCPP_INFO(this->get_logger(), "再稼働");
}

}  // namespace chassis_driver
