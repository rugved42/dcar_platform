cmake_minimum_required(VERSION 3.5)
project(mpu9250_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(my_robot_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Build the hardware driver library
add_library(mpu9250_driver
  src/mpu9250_driver.cpp
)

ament_target_dependencies(mpu9250_driver
  rclcpp
)

# Build the driver node executable
add_executable(mpu9250_driver_node
  src/mpu9250_driver_node.cpp
)

ament_target_dependencies(mpu9250_driver_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  my_robot_msgs
)

# Link the driver library to the node
target_link_libraries(mpu9250_driver_node
  mpu9250_driver
)

# Install targets
install(TARGETS
  mpu9250_driver_node
  mpu9250_driver
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files (if you add later)
# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_package()
