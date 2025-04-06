cmake_minimum_required(VERSION 3.10)
project(PCA9685_Driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(pca_test
    src/pca_driver.cpp
)

target_link_libraries(pca_test PRIVATE m)
