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
#include <pcl/common/angles.h>

struct ScopedTimer
{
    std::chrono::steady_clock::time_point start_time_;
    double &duration_ms_;

    explicit ScopedTimer(double &duration_ms) : duration_ms_(duration_ms)
    {
        start_time_ = std::chrono::steady_clock::now();
    }

    ~ScopedTimer()
    {
        auto end_time = std::chrono::steady_clock::now();
        duration_ms_ = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end_time - start_time_).count();
    }
};

class LidarProcessing : public rclcpp::Node
{
public:
    LidarProcessing() : Node("lidar_processing")
    {
        this->declare_parameter<double>("processing_rate", 10.0);
        this->declare_parameter<std::vector<double>>("crop_box_min", {-5.0, -15.0, -2.0});
        this->declare_parameter<std::vector<double>>("crop_box_max", {30.0, 15.0, 2.0});
        this->declare_parameter<std::vector<double>>("voxel_leaf_size", {0.1, 0.1, 0.1});
        this->declare_parameter<bool>("enable_ground_segmentation", true);
        this->declare_parameter<int>("profiling_interval_frames", 60);

        this->get_parameter("processing_rate", processing_rate_);
        this->get_parameter("crop_box_min", crop_box_min_);
        this->get_parameter("crop_box_max", crop_box_max_);
        this->get_parameter("voxel_leaf_size", voxel_leaf_size_);
        this->get_parameter("enable_ground_segmentation", enable_ground_segmentation_);
        this->get_parameter("profiling_interval_frames", profiling_interval_frames_);
        
        crop_box_min_vec = Eigen::Vector4f(crop_box_min_[0], crop_box_min_[1], crop_box_min_[2], 1.0);
        crop_box_max_vec = Eigen::Vector4f(crop_box_max_[0], crop_box_max_[1], crop_box_max_[2], 1.0);
        voxel_leaf_size_vec = Eigen::Vector4f(voxel_leaf_size_[0], voxel_leaf_size_[1], voxel_leaf_size_[2], 1.0);

        processing_timer_ = this->create_wall_timer(
            processing_rate_ > 0 ? std::chrono::milliseconds(static_cast<int>(1000.0 / processing_rate_)) : std::chrono::milliseconds(100),
            std::bind(&LidarProcessing::process_latest_point_cloud, this));
        lidar_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "lidar_in",
            10,
            std::bind(&LidarProcessing::lidar_subscriber_callback, this, std::placeholders::_1));
        processed_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "lidar_out",
            10);

        RCLCPP_INFO(this->get_logger(), "LidarProcessing node has been initialized.");

    }

    void lidar_subscriber_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        if (new_cloud_available_)
        {
            total_overwritten_frames_++;
        }
        latest_cloud_ = msg;
        latest_cloud_received_steady_ = std::chrono::steady_clock::now();
        new_cloud_available_ = true;
        total_received_frames_++;
        std::cout << "Received new point cloud with " << msg->width * msg->height << " points." << std::endl;
    }

    void process_latest_point_cloud()
    {
        PointCloudProcessingMetrics metrics;
        {
            ScopedTimer total_frame_timer(metrics.frame_total_time_ms);
            sensor_msgs::msg::PointCloud2::SharedPtr cloud_to_process_;
            std::chrono::steady_clock::time_point cloud_received_steady_;

            {
                std::lock_guard<std::mutex> lock(cloud_mutex_);
                if (new_cloud_available_ && latest_cloud_)
                {
                    std::cout << "Processing point cloud with " << latest_cloud_->width * latest_cloud_->height << " points." << std::endl;
                    cloud_to_process_ = latest_cloud_;
                    cloud_received_steady_ = latest_cloud_received_steady_;
                    new_cloud_available_ = false;
                }
                else
                {
                    return;
                }
            }

            auto now_steady = std::chrono::steady_clock::now();
            metrics.buffer_age_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(now_steady - cloud_received_steady_).count();
            metrics.input_points = static_cast<std::size_t>(cloud_to_process_->width) * static_cast<std::size_t>(cloud_to_process_->height);

            pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud;
            {
                ScopedTimer conversion_timer(metrics.conversion_time_ms);
                pcl_cloud = convert_ros2_pc_to_pcl(cloud_to_process_);
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr cropped_pcl;
            {
                ScopedTimer crop_box_timer(metrics.crop_box_time_ms);
                cropped_pcl = crop_box(pcl_cloud);
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr voxelized_pc;
            {
                ScopedTimer voxelization_timer(metrics.voxelization_time_ms);
                voxelized_pc = voxelize_pc(cropped_pcl);
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud = voxelized_pc;
            if (enable_ground_segmentation_)
            {
                ScopedTimer ground_segmentation_timer(metrics.ground_segmentation_time_ms);
                output_cloud = segmentGround(voxelized_pc);
            }

            {
                ScopedTimer publish_timer(metrics.publish_time_ms);
                publish_processed_cloud(output_cloud);
            }

            if (output_cloud)
            {
                metrics.output_points = output_cloud->points.size();
            }

            total_processed_frames_++;
        }

        update_profiling_metrics(metrics);
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
        crop_filter.setMin(crop_box_min_vec); // Set the minimum point of the box
        crop_filter.setMax(crop_box_max_vec);    // Set the maximum point of the box
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
        voxel_filter.setLeafSize(voxel_leaf_size_vec); // Set the voxel size (m)
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
        seg.setMaxIterations(100);       // Set the maximum number of iterations for RANSAC
        seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0)); // Set the axis for ground plane segmentation
        seg.setEpsAngle(pcl::deg2rad(10.0)); // Set the angle threshold for ground plane segmentation
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
    struct PointCloudProcessingMetrics
    {
        double buffer_age_ms{0.0};
        double conversion_time_ms{0.0};
        double crop_box_time_ms{0.0};
        double voxelization_time_ms{0.0};
        double ground_segmentation_time_ms{0.0};
        double publish_time_ms{0.0};
        double frame_total_time_ms{0.0};
        std::size_t input_points{0};
        std::size_t output_points{0};
    };

    void update_profiling_metrics(const PointCloudProcessingMetrics &metrics)
    {
        interval_sum_buffer_age_ms_ += metrics.buffer_age_ms;
        interval_sum_conversion_ms_ += metrics.conversion_time_ms;
        interval_sum_crop_box_ms_ += metrics.crop_box_time_ms;
        interval_sum_voxelization_ms_ += metrics.voxelization_time_ms;
        interval_sum_ground_segmentation_ms_ += metrics.ground_segmentation_time_ms;
        interval_sum_publish_ms_ += metrics.publish_time_ms;
        interval_sum_frame_total_ms_ += metrics.frame_total_time_ms;
        interval_sum_input_points_ += metrics.input_points;
        interval_sum_output_points_ += metrics.output_points;
        interval_frame_count_++;

        if (interval_frame_count_ >= profiling_interval_frames_)
        {
            const double avg_buffer_age_ms = interval_sum_buffer_age_ms_ / interval_frame_count_;
            const double avg_conversion_ms = interval_sum_conversion_ms_ / interval_frame_count_;
            const double avg_crop_box_ms = interval_sum_crop_box_ms_ / interval_frame_count_;
            const double avg_voxelization_ms = interval_sum_voxelization_ms_ / interval_frame_count_;
            const double avg_ground_segmentation_ms = interval_sum_ground_segmentation_ms_ / interval_frame_count_;
            const double avg_publish_ms = interval_sum_publish_ms_ / interval_frame_count_;
            const double avg_frame_total_ms = interval_sum_frame_total_ms_ / interval_frame_count_;
            const double avg_input_points = interval_sum_input_points_ / interval_frame_count_;
            const double avg_output_points = interval_sum_output_points_ / interval_frame_count_;

            RCLCPP_INFO(
                this->get_logger(),
                "Average point cloud processing metrics over last %d frames: Buffer Age: %.2f ms, Conversion Time: %.2f ms, Crop Box Time: %.2f ms, Voxelization Time: %.2f ms, Ground Segmentation Time: %.2f ms, Publish Time: %.2f ms, Frame Total Time: %.2f ms, Average Input Points: %.2f, Average Output Points: %.2f, Total Received Frames: %ld, Total Processed Frames: %ld, Total Overwritten Frames: %ld",
                profiling_interval_frames_,
                avg_buffer_age_ms,
                avg_conversion_ms,
                avg_crop_box_ms,
                avg_voxelization_ms,
                avg_ground_segmentation_ms,
                avg_publish_ms,
                avg_frame_total_ms,
                avg_input_points,
                avg_output_points,
                total_received_frames_,
                total_processed_frames_,
                total_overwritten_frames_);

            interval_frame_count_ = 0;
            interval_sum_buffer_age_ms_ = 0.0;
            interval_sum_conversion_ms_ = 0.0;
            interval_sum_crop_box_ms_ = 0.0;
            interval_sum_voxelization_ms_ = 0.0;
            interval_sum_ground_segmentation_ms_ = 0.0;
            interval_sum_publish_ms_ = 0.0;
            interval_sum_frame_total_ms_ = 0.0;
            interval_sum_input_points_ = 0.0;
            interval_sum_output_points_ = 0.0;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
    std::chrono::steady_clock::time_point latest_cloud_received_steady_;
    bool new_cloud_available_{false};
    std::mutex cloud_mutex_;
    rclcpp::TimerBase::SharedPtr processing_timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_cloud_publisher_;
    double processing_rate_;
    std::vector<double> crop_box_min_;
    std::vector<double> crop_box_max_;
    std::vector<double> voxel_leaf_size_;
    bool enable_ground_segmentation_;
    Eigen::Vector4f crop_box_min_vec;
    Eigen::Vector4f crop_box_max_vec;
    Eigen::Vector4f voxel_leaf_size_vec;
    int interval_frame_count_{0};
    double interval_sum_buffer_age_ms_{0.0};
    double interval_sum_conversion_ms_{0.0};
    double interval_sum_crop_box_ms_{0.0};
    double interval_sum_voxelization_ms_{0.0};
    double interval_sum_ground_segmentation_ms_{0.0};
    double interval_sum_publish_ms_{0.0};
    double interval_sum_frame_total_ms_{0.0};
    double interval_sum_input_points_{0.0};
    double interval_sum_output_points_{0.0};
    std::uint64_t total_received_frames_{0};
    std::uint64_t total_processed_frames_{0};
    std::uint64_t total_overwritten_frames_{0};
    int profiling_interval_frames_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarProcessing>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
