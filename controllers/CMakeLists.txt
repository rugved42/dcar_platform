cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(controllers
    src/drive_base.cpp
)

target_include_directories(controllers
    PUBLIC ${PROJECT_SOURCE_DIR}/controllers/include
    PUBLIC ${PROJECT_SOURCE_DIR}/drivers/pca9685_driver/include
)

target_link_libraries(controllers pca9685_driver)
