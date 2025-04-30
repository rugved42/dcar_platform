#include "pi_camera_driver/camera_node.hpp"
#include <cv_bridge/cv_bridge.h>

namespace pi_camera_driver {

CameraNode::CameraNode() : Node("pi_camera_node") {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CameraNode::timerCallback, this));

    cap_.open(0);  // /dev/video0

    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open Pi Camera.");
    }
}

void CameraNode::timerCallback() {
    cv::Mat frame;
    if (cap_.read(frame)) {
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now();
        publisher_->publish(*msg);
    } else {
        RCLCPP_WARN(this->get_logger(), "Failed to capture frame.");
    }
}

}  // namespace pi_camera_driver

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pi_camera_driver::CameraNode>());
    rclcpp::shutdown();
    return 0;
}
