#include "rclcpp/rclcpp.hpp"
#include "agent_controller/simple_controller.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto simple_controller_node = std::make_shared<SimpleController>();

  rclcpp::spin(simple_controller_node);
  rclcpp::shutdown();

  return 0;
}
