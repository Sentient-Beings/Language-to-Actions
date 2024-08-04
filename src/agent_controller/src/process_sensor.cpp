#include "agent_controller/process_sensor.hpp"
#include <vector>

ProcessSensor::ProcessSensor() 
: Node("process_sensor_node")
{
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&ProcessSensor::process_sensor_data, this, std::placeholders::_1));
    
    publisher_ = this->create_publisher<agent_interfaces::msg::SensorInterface>("safety_distance", 10);

    service_ = this->create_service<agent_interfaces::srv::GetSensorDistances>(
        "get_saftey_distance", std::bind(&ProcessSensor::process_sensor_data_service, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Process Sensor Node has been started.");
}

ProcessSensor::~ProcessSensor() {}

double calculate_min(const std::vector<float>& data){
    double min = std::numeric_limits<float>::infinity();
    for (const auto& d : data){
        if (d < min){
            min = d;
        }
    }
    return min;
}

void ProcessSensor::process_sensor_data(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    double min_angle = msg->angle_min;
    double angle_increment = msg->angle_increment;
    const std::vector<float>& ranges = msg->ranges;

    // Right side of the robot (-2.356 to -0.785)
    int min_index_right = static_cast<int>((-2.356 - min_angle) / angle_increment);
    int max_index_right = static_cast<int>((-0.785 - min_angle) / angle_increment);

    // Front side of the robot (-0.785 to 0.785)
    int min_index_front = static_cast<int>((-0.785 - min_angle) / angle_increment);
    int max_index_front = static_cast<int>((0.785 - min_angle) / angle_increment);

    // Left side of the robot (0.785 to 2.356)
    int min_index_left = static_cast<int>((0.785 - min_angle) / angle_increment);
    int max_index_left = static_cast<int>((2.356 - min_angle) / angle_increment);

    // Now computing the mean distance for each side of the robot
    min_distance_right = calculate_min(std::vector<float>(ranges.begin() + min_index_right, ranges.begin() + max_index_right));
    min_distance_left =  calculate_min(std::vector<float>(ranges.begin() + min_index_left, ranges.begin() + max_index_left));
    min_distance_front = calculate_min(std::vector<float>(ranges.begin() + min_index_front, ranges.begin() + max_index_front));


    auto message = agent_interfaces::msg::SensorInterface();
    message.min_distance_right = min_distance_right;
    message.min_distance_left = min_distance_left;
    message.min_distance_front = min_distance_front;

    // RCLCPP_INFO(this->get_logger(), "Right: %f, Front: %f, Left: %f", min_distance_right, min_distance_front, min_distance_left);
    publisher_->publish(message);
}

void ProcessSensor::process_sensor_data_service(
    const std::shared_ptr<agent_interfaces::srv::GetSensorDistances::Request> request,
    std::shared_ptr<agent_interfaces::srv::GetSensorDistances::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Request for sensor distances received.");
    response->min_distance_right = min_distance_right;
    response->min_distance_left = min_distance_left;
    response->min_distance_front = min_distance_front;
    RCLCPP_INFO(this->get_logger(), "Response sent.");
}


