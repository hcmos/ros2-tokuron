#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

class OmniGoalFollower : public rclcpp::Node
{
public:
    OmniGoalFollower() : Node("omni_goal_follower")
    {
        // サブスクライバー: 目標位置
        goal_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "goal_pose", 10,
            std::bind(&OmniGoalFollower::goalCallback, this, std::placeholders::_1));

        // サブスクライバー: 自己位置
        pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "self_pose", 10,
            std::bind(&OmniGoalFollower::poseCallback, this, std::placeholders::_1));

        // パブリッシャー: 制御コマンド
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "OmniGoalFollower node started.");
    }

private:
    // コールバック: 目標位置の受信
    void goalCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        goal_pose_ = *msg;
        goal_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Received goal: x=%.2f, y=%.2f, theta=%.2f", goal_pose_.x, goal_pose_.y, goal_pose_.theta);
    }

    // コールバック: 自己位置の受信
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        self_pose_ = *msg;
        self_pose_received_ = true;

        if (goal_received_)
        {
            computeAndPublishControl();
        }
    }

    // 制御計算とパブリッシュ
    void computeAndPublishControl()
    {
        // 目標位置と自己位置の差を計算
        double dx = goal_pose_.x - self_pose_.pose.position.x;
        double dy = goal_pose_.y - self_pose_.pose.position.y;

        // 距離を計算
        double distance = std::sqrt(dx * dx + dy * dy);

        // 現在の姿勢の角度（クォータニオンからyawを計算）
        double current_yaw = std::atan2(
            2.0 * (self_pose_.pose.orientation.w * self_pose_.pose.orientation.z),
            1.0 - 2.0 * (self_pose_.pose.orientation.z * self_pose_.pose.orientation.z));

        // 並進速度を計算 (x, y方向)
        double velocity_x = kp_linear_ * dx;
        double velocity_y = kp_linear_ * dy;

        // 目標角度（Pose2Dのtheta）との差を計算
        double angle_error = goal_pose_.theta - current_yaw;

        // 角度誤差を正規化 [-pi, pi] の範囲に収める
        angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error));

        // 角速度を計算
        double angular_speed = kp_angular_ * angle_error;

        // 制限を適用
        velocity_x = std::clamp(velocity_x, -max_linear_speed_, max_linear_speed_);
        velocity_y = std::clamp(velocity_y, -max_linear_speed_, max_linear_speed_);
        angular_speed = std::clamp(angular_speed, -max_angular_speed_, max_angular_speed_);

        // 速度コマンドをパブリッシュ
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = velocity_x;
        cmd_vel.linear.y = velocity_y;
        cmd_vel.angular.z = angular_speed;

        velocity_publisher_->publish(cmd_vel);

        RCLCPP_INFO(this->get_logger(), "Published velocity: linear_x=%.2f, linear_y=%.2f, angular=%.2f",
                    velocity_x, velocity_y, angular_speed);
    }

    // ROS2サブスクライバーとパブリッシャー
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr goal_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;

    // 制御パラメータ
    double kp_linear_ = 1.0;     // 並進速度の比例ゲイン
    double kp_angular_ = 1.0;    // 角速度の比例ゲイン
    double max_linear_speed_ = 0.5;  // 最大並進速度
    double max_angular_speed_ = 1.0; // 最大角速度

    // 目標位置と自己位置の状態
    geometry_msgs::msg::Pose2D goal_pose_;
    geometry_msgs::msg::PoseStamped self_pose_;
    bool goal_received_ = false;
    bool self_pose_received_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OmniGoalFollower>());
    rclcpp::shutdown();
    return 0;
}
