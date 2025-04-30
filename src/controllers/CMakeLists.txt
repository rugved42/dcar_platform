cmake_minimum_required(VERSION 3.5)
project(controllers)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(wheel_encoder REQUIRED)
find_package(pca9685_driver REQUIRED)

# Library target
add_library(${PROJECT_NAME}
  src/drive_base.cpp
)

# Include directories (adds the headers from this package)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link against dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  wheel_encoder
  pca9685_driver
)

# Explicitly link the dependency libraries if they are built as libraries
# (Only necessary if those packages define their own add_library)
target_link_libraries(${PROJECT_NAME}
  pca9685_driver
  wheel_encoder
)

# Export include directories and libraries
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp wheel_encoder pca9685_driver)

ament_package()
