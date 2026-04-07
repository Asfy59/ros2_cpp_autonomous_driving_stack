#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

class LidarProcessing : public rclcpp::Node
{
public:
    LidarProcessing() : Node("lidar_processing")
    {
        RCLCPP_INFO(this->get_logger(), "LidarProcessing node has been started");
        processing_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&LidarProcessing::process_latest_point_cloud, this));
        lidar_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "lidar_in",
            10,
            std::bind(&LidarProcessing::lidar_subscriber_callback, this, std::placeholders::_1));
        processed_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "lidar_out",
            10);
    }

    void lidar_subscriber_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        latest_cloud_ = msg;
        new_cloud_available_ = true;
        std::cout << "Received new point cloud with " << msg->width * msg->height << " points." << std::endl;
    }

    void process_latest_point_cloud()
    {

        sensor_msgs::msg::PointCloud2::SharedPtr cloud_to_process_;

        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            if (new_cloud_available_ && latest_cloud_)
            {
                // Process the latest point cloud here
                std::cout << "Processing point cloud with " << latest_cloud_->width * latest_cloud_->height << " points." << std::endl;
                cloud_to_process_ = latest_cloud_;
                new_cloud_available_ = false; // Reset the flag after processing
            }
            else
            {
                // No new cloud available, skip processing
                return;
            }
        }

        // copy and conversion to pcl
        auto pcl_cloud = convert_ros2_pc_to_pcl(cloud_to_process_);
        // call the crop_box funciton
        auto cropped_pcl = crop_box(pcl_cloud);
        // call the voxelization function
        auto voxelized_pc = voxelize_pc(cropped_pcl);
        // call the ground plane segmentation function
        auto segmented_pc = segmentGround(voxelized_pc);
        // call the clustering function(optional)
        // publish what we have processed so far
        publish_processed_cloud(segmented_pc);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr convert_ros2_pc_to_pcl(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud)
    {
        // Convert the ROS PointCloud2 message to a PCL point cloud
        auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::fromROSMsg(*cloud, *pcl_cloud);
        return pcl_cloud;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr crop_box(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
    {
        // Implement cropping using a box filter or other methods
        pcl::CropBox<pcl::PointXYZI> crop_filter;
        crop_filter.setInputCloud(cloud);
        crop_filter.setMin(Eigen::Vector4f(-5.0, -15.0, -2.0, 1.0)); // Set the minimum point of the box
        crop_filter.setMax(Eigen::Vector4f(30.0, 15.0, 2.0, 1.0));    // Set the maximum point of the box
        auto cropped_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        crop_filter.filter(*cropped_cloud);

        return cropped_cloud;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr voxelize_pc(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
    {
        if (!cloud)
        {
            RCLCPP_WARN(this->get_logger(), "No point cloud available for voxelization");
            return nullptr;
        }

        // Perform voxelization on the PCL point cloud
        pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f); // Set the voxel size (m)
        auto voxelized_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        voxel_filter.filter(*voxelized_cloud);

        return voxelized_cloud;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr segmentGround(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
    {
        // Implement ground plane segmentation using RANSAC or other methods
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.2); // Set the distance threshold for inliers
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No non-ground points found after segmentation");
            return cloud; // Return the original cloud if no non-ground points are found
        }
        // Extract non-ground points (outliers)
        pcl::PointCloud<pcl::PointXYZI>::Ptr non_ground_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true); // true to extract non-ground points
        extract.filter(*non_ground_cloud);


        return non_ground_cloud;
    }

    sensor_msgs::msg::PointCloud2::SharedPtr convert_pcl_to_ros2_pc(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
    {
        // Convert the ROS PointCloud2 message to a PCL point cloud
        auto ros2_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*cloud, *ros2_cloud);
        return ros2_cloud;
    }

    void publish_processed_cloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
    {
        if (!cloud)
        {
            RCLCPP_WARN(this->get_logger(), "No point cloud available for publishing");
            return;
        }

        auto ros2_cloud = convert_pcl_to_ros2_pc(cloud);
        processed_cloud_publisher_->publish(*ros2_cloud);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
    bool new_cloud_available_{false};
    std::mutex cloud_mutex_;
    rclcpp::TimerBase::SharedPtr processing_timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_cloud_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarProcessing>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}