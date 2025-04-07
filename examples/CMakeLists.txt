add_executable(drivebase_example drivebase_example.cpp)
target_link_libraries(drivebase_example PRIVATE controllers)
target_include_directories(drivebase_example PRIVATE
    PUBLIC ${PROJECT_SOURCE_DIR}/controllers/include
    PUBLIC ${PROJECT_SOURCE_DIR}/drivers/pca9685_driver/include
)