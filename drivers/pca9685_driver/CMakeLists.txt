cmake_minimum_required(VERSION 3.10)
project(PCA9685_Driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(pca9685_driver
    src/pca9685_driver.cpp
)

target_include_directories(pca9685_driver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)