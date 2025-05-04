#include "rclcpp/rclcpp.hpp"
#include "my_robot_msgs/msg/raw_imu.hpp"
#include "mpu9250_driver/mpu9250_driver.hpp"

using namespace std::chrono_literals;
using namespace mpu9250_driver;

class MPU9250DriverNode : public rclcpp::Node
{
public:
    MPU9250DriverNode()
    : Node("mpu9250_driver_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("i2c_device", "/dev/i2c-6");
        this->declare_parameter<std::string>("frame_id", "imu_link");
        this->declare_parameter<double>("publish_rate", 50.0); // Hz

        // Get parameters
        i2c_device_ = this->get_parameter("i2c_device").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        double publish_rate = this->get_parameter("publish_rate").as_double();

        // Initialize driver
        driver_ = std::make_unique<MPU9250Driver>(i2c_device_);
        driver_->initialize();
        driver_->calibrate();

        // Create publisher
        publisher_ = this->create_publisher<my_robot_msgs::msg::RawImu>("/imu/raw", 10);

        // Create timer to publish at fixed rate
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate),
            std::bind(&MPU9250DriverNode::publishImuData, this)
        );

        RCLCPP_INFO(this->get_logger(), "MPU9250 Driver Node initialized successfully");
    }

private:
    void publishImuData()
    {
        ImuData imu_data = driver_->readImu();

        auto msg = my_robot_msgs::msg::RawImu();
        msg.header.stamp = this->now();
        msg.header.frame_id = frame_id_;

        msg.linear_acceleration.x = imu_data.accel_x;
        msg.linear_acceleration.y = imu_data.accel_y;
        msg.linear_acceleration.z = imu_data.accel_z;

        msg.angular_velocity.x = imu_data.gyro_x;
        msg.angular_velocity.y = imu_data.gyro_y;
        msg.angular_velocity.z = imu_data.gyro_z;

        msg.magnetic_field.x = imu_data.mag_x;
        msg.magnetic_field.y = imu_data.mag_y;
        msg.magnetic_field.z = imu_data.mag_z;

        publisher_->publish(msg);
    }

    // Members
    rclcpp::Publisher<my_robot_msgs::msg::RawImu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<MPU9250Driver> driver_;
    std::string i2c_device_;
    std::string frame_id_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPU9250DriverNode>());
    rclcpp::shutdown();
    return 0;
}
