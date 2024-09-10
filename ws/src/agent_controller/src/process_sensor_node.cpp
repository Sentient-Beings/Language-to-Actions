#include "rclcpp/rclcpp.hpp"
#include "agent_controller/process_sensor.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto process_sensor_node = std::make_shared<ProcessSensor>();

  rclcpp::spin(process_sensor_node);
  rclcpp::shutdown();

  return 0;
}
