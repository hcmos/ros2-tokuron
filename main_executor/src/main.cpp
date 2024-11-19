#include <rclcpp/rclcpp.hpp>

#include "controller/controller_node.hpp"
#include "chassis_driver/chassis_driver_node.hpp"
#include "socketcan_interface/socketcan_interface_node.hpp"
#include "cybergear_interface/cybergear_interface_node.hpp"
#include "landmark_localization/landmark_localization.hpp"

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    rclcpp::NodeOptions nodes_option;
    nodes_option.allow_undeclared_parameters(true);
    nodes_option.automatically_declare_parameters_from_overrides(true);

    auto socketcan_node = std::make_shared<socketcan_interface::SocketcanInterface>(nodes_option);
    auto chassis_driver_node = std::make_shared<chassis_driver::ChassisDriver>(nodes_option);
    auto controller_node = std::make_shared<controller::Controller>(nodes_option);
    auto cybergear_interface_node = std::make_shared<cybergear_interface::CybergearInterface>(nodes_option);
    auto landmark_localization_node = std::make_shared<landmark_localization::LandmarkLocalization>(nodes_option);

    exec.add_node(socketcan_node);
    exec.add_node(controller_node);
    exec.add_node(chassis_driver_node);
    exec.add_node(cybergear_interface_node);
    exec.add_node(landmark_localization_node);

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
