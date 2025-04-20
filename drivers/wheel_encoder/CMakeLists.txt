add_library(wheel_encoder
    src/wheel_encoder.cpp
)

target_include_directories(wheel_encoder
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(wheel_encoder PRIVATE gpiod)
