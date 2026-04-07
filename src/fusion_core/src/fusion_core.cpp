#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
class FusionCore : public rclcpp::Node
{
public:
    FusionCore() : Node("fusion_core")
    {
        // this->declare_parameter("camera_topic", "/p2_img");
        // std::string camera_topic;
        // this->get_parameter("camera_topic", camera_topic);
        // RCLCPP_INFO(this->get_logger(), "Camera topic set to: %s", camera_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "FusionCore node has been started");
        // timer_ = this->create_wall_timer(
        //     std::chrono::seconds(1),
        //     std::bind(&FusionCore::timer_callback, this));
        camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera_topic",
            10,
            std::bind(&FusionCore::camera_subscriber_callback, this, std::placeholders::_1));
        lidar_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "lidar_pc",
            10,
            std::bind(&FusionCore::lidar_subscriber_callback, this, std::placeholders::_1));
    }

    void camera_subscriber_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (msg)
        {
            RCLCPP_INFO(this->get_logger(), "Camera subscriber callback triggered");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Received null message in camera subscriber callback");
        }
    }

    void lidar_subscriber_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (msg)
        {
            RCLCPP_INFO(this->get_logger(), "Lidar subscriber callback triggered");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Received null message in lidar subscriber callback");
        }
    }

    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Timer callback triggered");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FusionCore>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}