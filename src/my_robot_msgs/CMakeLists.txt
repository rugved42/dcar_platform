cmake_minimum_required(VERSION 3.5)
project(my_robot_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RawImu.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_package()