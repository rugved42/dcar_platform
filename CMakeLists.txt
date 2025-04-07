cmake_minimum_required(VERSION 3.10)
project(dcar_platform)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(drivers)
add_subdirectory(controllers)
add_subdirectory(examples)

include_directories(
    drivers/pca9685_driver/include
)