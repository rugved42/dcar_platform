#ifndef PI_CAMERA_DRIVER_CAMERA_NODE_HPP
#define PI_CAMERA_DRIVER_CAMERA_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

namespace pi_camera_driver {

class CameraNode : public rclcpp::Node {
public:
    CameraNode();

private:
    void timerCallback();
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

}  // namespace pi_camera_driver

#endif  // PI_CAMERA_DRIVER_CAMERA_NODE_HPP
