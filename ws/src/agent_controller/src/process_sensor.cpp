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

    // front right side of the robot (-90 to -30 degrees)
    int min_index_front_right = static_cast<int>((-M_PI_2 - min_angle) / angle_increment);
    int max_index_front_right = static_cast<int>((-M_PI / 6 - min_angle) / angle_increment);

    // front side of the robot (-30 to 30 degrees)
    int min_index_front = static_cast<int>((-M_PI / 6 - min_angle) / angle_increment);
    int max_index_front = static_cast<int>((M_PI / 6 - min_angle) / angle_increment);

    // front left side of the robot (30 to 90 degrees)
    int min_index_front_left = static_cast<int>((M_PI / 6 - min_angle) / angle_increment);
    int max_index_front_left = static_cast<int>((M_PI_2 - min_angle) / angle_increment);

    // Now computing the minimum distance for each side of the robot
    this->min_distance_front_right = calculate_min(std::vector<float>(ranges.begin() + min_index_front_right, ranges.begin() + max_index_front_right));
    this->min_distance_front_left = calculate_min(std::vector<float>(ranges.begin() + min_index_front_left, ranges.begin() + max_index_front_left));
    this->min_distance_front = calculate_min(std::vector<float>(ranges.begin() + min_index_front, ranges.begin() + max_index_front));

    is_initialized = true;
    auto message = agent_interfaces::msg::SensorInterface();
    message.min_distance_front_right = min_distance_front_right;
    message.min_distance_front_left = min_distance_front_left;
    message.min_distance_front = min_distance_front;

    // RCLCPP_INFO(this->get_logger(), "Front Right: %f, Front: %f, Front Left: %f", min_distance_front_right, min_distance_front, min_distance_front_left);
    publisher_->publish(message);
}

void ProcessSensor::process_sensor_data_service(
    const std::shared_ptr<agent_interfaces::srv::GetSensorDistances::Request> request,
    std::shared_ptr<agent_interfaces::srv::GetSensorDistances::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Request for sensor distances received.");
    if (!is_initialized){
        RCLCPP_ERROR(this->get_logger(), "Sensor data is not initialized.");
        return;
    }
    response->min_distance_front_right = this->min_distance_front_right;
    response->min_distance_front_left = this->min_distance_front_left;
    response->min_distance_front = this->min_distance_front;
    RCLCPP_INFO(this->get_logger(), "Response sent.");
}


