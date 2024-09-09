#include "rclcpp/rclcpp.hpp"
#include "agent_controller/simple_controller.hpp"

SimpleController::SimpleController()
: Node("simple_controller_node")
{
    controller_command_subscription_ = this->create_subscription<agent_interfaces::msg::ControllerInterface>(
        "controller_interface", 10, std::bind(&SimpleController::process_control_command, this, std::placeholders::_1));

    controller_command_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("diff_cont/cmd_vel_unstamped", 10);

    RCLCPP_INFO(this->get_logger(), "Simple Controller Node has been started.");
}

SimpleController::~SimpleController() {}

void SimpleController::process_control_command(const agent_interfaces::msg::ControllerInterface::SharedPtr msg) {
    std::string command = msg->move_direction;
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
    else {
        RCLCPP_INFO(this->get_logger(), "Invalid command received: %s", command.c_str());
        this->forward_speed = 0.0;
        this->rotation_speed = 0.0;
        this->x = 0.0;
        this->y = 0.0;
        this->z = 0.0;
        this->theta = 0.0;
        return;
    }

    this->publish_control_command();
}

void SimpleController::publish_control_command() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x  = x * this->forward_speed;
    message.linear.y  = y * this->forward_speed;
    message.linear.z  = z * this->forward_speed;
    message.angular.x = 0.0;
    message.angular.y = 0.0;
    message.angular.z = theta * this->rotation_speed;
    this->controller_command_publisher_->publish(message); 
}