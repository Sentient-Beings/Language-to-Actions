cmake_minimum_required(VERSION 3.8)
project(agent_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(agent_interfaces REQUIRED)   

include_directories(
  include
)

add_executable(sensor_processor 
src/process_sensor.cpp
src/process_sensor_node.cpp
)
ament_target_dependencies(sensor_processor rclcpp sensor_msgs agent_interfaces)

install(TARGETS
  sensor_processor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
