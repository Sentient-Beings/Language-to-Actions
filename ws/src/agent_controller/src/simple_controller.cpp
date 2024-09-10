#include "rclcpp/rclcpp.hpp"
#include "agent_controller/simple_controller.hpp"
#include <algorithm>

#include <chrono>

SimpleController::SimpleController()
: Node("simple_controller_node")
{
    // we want the publisher and subscriber to be in different callback groups
    // to execute in parallel
    auto sub_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = sub_callback_group;

    controller_command_subscription_ = this->create_subscription<agent_interfaces::msg::ControllerInterface>(
        "controller_interface", 10, std::bind(&SimpleController::process_control_command, this, std::placeholders::_1), sub_options);

    controller_command_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("diff_cont/cmd_vel_unstamped", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SimpleController::publish_control_command, this));

    RCLCPP_INFO(this->get_logger(), "Simple Controller Node has been started.");
}

SimpleController::~SimpleController() {}

void SimpleController::process_control_command(const agent_interfaces::msg::ControllerInterface::SharedPtr msg) {
    std::string command = msg->move_direction;

    std::lock_guard<std::mutex> lock(twist_mutex_);
    if (std::find(control_commands.begin(), control_commands.end(), command) == control_commands.end()) {
        RCLCPP_INFO(this->get_logger(), "Invalid command received: %s", command.c_str());
        // we just stop the robot
        this->forward_speed = 0.0;
        this->rotation_speed = 0.0;
        this->x = 0.0;
        this->y = 0.0;
        this->z = 0.0;
        this->theta = 0.0;
        this->publish_control_command();
        return;
    }
    if (command == "forward") {
        this->forward_speed = 0.5;
        this->rotation_speed = 0.0;
        this->x = 1.0;
        this->y = 0.0;
        this->z = 0.0;
        this->theta = 0.0;
    }
    else if (command == "left") {
        this->forward_speed = 0.0;
        this->rotation_speed = 1.0;
        this->x = 0.0;
        this->y = 0.0;
        this->z = 0.0;
        this->theta = 1.0;
    }
    else if (command == "right") {
        this->forward_speed = 0.0;
        this->rotation_speed = 1.0;
        this->x = 0.0;
        this->y = 0.0;
        this->z = 0.0;
        this->theta = -1.0;
    }
}

void SimpleController::publish_control_command() {
    auto message = geometry_msgs::msg::Twist();
    std::lock_guard<std::mutex> lock(twist_mutex_);
    message.linear.x  = this->x * this->forward_speed;
    message.linear.y  = this->y * this->forward_speed;
    message.linear.z  = this->z * this->forward_speed;
    message.angular.x = 0.0;
    message.angular.y = 0.0;
    message.angular.z = this->theta * this->rotation_speed;
    this->controller_command_publisher_->publish(message); 
}