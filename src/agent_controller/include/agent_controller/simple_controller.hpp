#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "geometry_msgs/msg/twist.hpp"
#include "agent_interfaces/msg/controller_interface.hpp"

class SimpleController : public rclcpp::Node
{
    public:
        SimpleController();
        ~SimpleController();
        void process_control_command(const agent_interfaces::msg::ControllerInterface::SharedPtr msg);
        void publish_control_command();

        std::vector<std::string> control_commands = {"forward", "left", "right"};
        double forward_speed = 0.0;
        double rotation_speed = 0.0;
        double x , y , z , theta = 0.0;
        geometry_msgs::msg::Twist message;
    private:
        rclcpp::Subscription<agent_interfaces::msg::ControllerInterface>::SharedPtr controller_command_subscription_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr controller_command_publisher_;
};

#endif // SIMPLE_CONTROLLER_HPP
