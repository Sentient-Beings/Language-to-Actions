#ifndef PROCESS_SENSOR_HPP
#define PROCESS_SENSOR_HPP

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "agent_interfaces/srv/get_sensor_distances.hpp"
#include "agent_interfaces/msg/sensor_interface.hpp"

class ProcessSensor : public rclcpp::Node
{
    public:
        ProcessSensor();
        ~ProcessSensor();
        void process_sensor_data(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        void process_sensor_data_service(
            const std::shared_ptr<agent_interfaces::srv::GetSensorDistances::Request> request,
            std::shared_ptr<agent_interfaces::srv::GetSensorDistances::Response> response);
        
        double min_distance_right;
        double min_distance_left;
        double min_distance_front;
        bool is_initialized;
    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
        rclcpp::Publisher<agent_interfaces::msg::SensorInterface>::SharedPtr publisher_;
        rclcpp::Service<agent_interfaces::srv::GetSensorDistances>::SharedPtr service_;
};

#endif // PROCESS_SENSOR_HPP
