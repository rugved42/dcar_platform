#include "pi_camera_driver/camera_node.hpp"
#include <cv_bridge/cv_bridge.h>

namespace pi_camera_driver {

CameraNode::CameraNode() : Node("pi_camera_node") {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", rclcpp::QoS(30));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&CameraNode::timerCallback, this));

    cap_.open(0);  // /dev/video0

    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open Pi Camera.");
    }
}

void CameraNode::timerCallback() {
    cv::Mat frame;
    if (cap_.read(frame)) {
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera_frame";
        auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
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
