# lidar_multi
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarSubscriberPublisher : public rclcpp::Node
{
public:
    LidarSubscriberPublisher() : Node("lidar_subscriber_publisher")
    {
        // Initialize the subscriber
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LidarSubscriberPublisher::laserCallback, this, std::placeholders::_1));

        // Initialize the publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("filtered_lidar", 10);
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        float start_angle = 0.0;
        float end_angle = 20.0;

        start_angle *= M_PI / 180.0;
        end_angle *= M_PI / 180.0;

        float increment = msg->angle_increment;

        // Create a new LaserScan message for filtered data
        auto filtered_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
        *filtered_msg = *msg;  // Copy header and other properties
        filtered_msg->ranges.clear();

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float angle = increment * i;
            // RCLCPP_INFO(this->get_logger(), "angle_increment: %f", angle);
            if (angle >= start_angle && angle <= end_angle)
            {
                // Add the point to the filtered message
                filtered_msg->ranges.push_back(msg->ranges[i]);
            }
        }

        // Publish the filtered lidar data
        publisher_->publish(filtered_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarSubscriberPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}