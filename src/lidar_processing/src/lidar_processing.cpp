#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <unordered_map>
#include <sys/resource.h>
#include <unistd.h>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>

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

// These process stats are Linux-specific, which is okay for the current deployment target.
static double read_process_cpu_time_ms()
{
    rusage usage{};
    if (getrusage(RUSAGE_SELF, &usage) != 0)
    {
        return 0.0;
    }

    const double user_time_ms =
        (static_cast<double>(usage.ru_utime.tv_sec) * 1000.0) +
        (static_cast<double>(usage.ru_utime.tv_usec) / 1000.0);
    const double system_time_ms =
        (static_cast<double>(usage.ru_stime.tv_sec) * 1000.0) +
        (static_cast<double>(usage.ru_stime.tv_usec) / 1000.0);
    return user_time_ms + system_time_ms;
}

static double read_current_rss_mb()
{
    std::ifstream statm_stream("/proc/self/statm");
    long total_pages = 0;
    long resident_pages = 0;
    if (!(statm_stream >> total_pages >> resident_pages))
    {
        return 0.0;
    }
    (void)total_pages;

    const long page_size_bytes = sysconf(_SC_PAGESIZE);
    return (static_cast<double>(resident_pages) * static_cast<double>(page_size_bytes)) /
           (1024.0 * 1024.0);
}

static double read_peak_rss_mb()
{
    rusage usage{};
    if (getrusage(RUSAGE_SELF, &usage) != 0)
    {
        return 0.0;
    }

    return static_cast<double>(usage.ru_maxrss) / 1024.0;
}

class LidarProcessing : public rclcpp::Node
{
public:
    struct BufferedCloud
    {
        sensor_msgs::msg::PointCloud2::SharedPtr msg;
        std::chrono::steady_clock::time_point received_steady;
    };

    LidarProcessing() : Node("lidar_processing")
    {
        this->declare_parameter<double>("processing_rate", 10.0);
        this->declare_parameter<std::vector<double>>("crop_box_min", {-5.0, -10.0, -2.0});
        this->declare_parameter<std::vector<double>>("crop_box_max", {30.0, 10.0, 2.0});
        this->declare_parameter<std::vector<double>>("voxel_leaf_size", {0.1, 0.1, 0.1});
        this->declare_parameter<bool>("publish_processed_lidar_pc", true);
        this->declare_parameter<bool>("enable_ground_segmentation", true);
        this->declare_parameter<int>("min_cluster_points", 30);
        this->declare_parameter<int>("max_cluster_points", 5000);
        this->declare_parameter<std::vector<double>>("min_cluster_size", {0.2, 0.2, 0.2});
        this->declare_parameter<std::vector<double>>("max_cluster_size", {15.0, 10.0, 5.0});
        this->declare_parameter<int>("profiling_interval_frames", 60);
        this->declare_parameter<bool>("enable_csv_logging", false);
        this->declare_parameter<std::string>("csv_log_dir", "csv_logs/lidar_processing");
        this->declare_parameter<std::string>("dataset_sequence", "unknown");
        this->declare_parameter<int>("input_queue_size", 3);
        this->declare_parameter<double>("max_buffer_age_ms", 300.0);
        this->declare_parameter<int>("processing_timer_fallback_period_ms", 100);
        this->declare_parameter<int>("lidar_input_qos_depth", 5);
        this->declare_parameter<int>("processed_cloud_qos_depth", 5);
        this->declare_parameter<int>("detection_qos_depth", 5);
        this->declare_parameter<int>("marker_qos_depth", 5);
        this->declare_parameter<double>("ground_segmentation_distance_threshold_m", 0.33);
        this->declare_parameter<int>("ground_segmentation_max_iterations", 1000);
        this->declare_parameter<std::vector<double>>("ground_segmentation_axis", {0.0, 0.0, 1.0});
        this->declare_parameter<double>("ground_segmentation_eps_angle_deg", 10.0);
        this->declare_parameter<std::vector<double>>("clustering_voxel_leaf_size", {0.3, 0.3, 0.3});
        this->declare_parameter<bool>("enable_range_adaptive_clustering", true);
        this->declare_parameter<std::vector<double>>("clustering_range_breakpoints_m", {10.0, 20.0});
        this->declare_parameter<std::vector<double>>("clustering_tolerance_by_range_m", {0.7, 1.0, 1.4});
        this->declare_parameter<double>("marker_min_dimension_m", 0.05);
        this->declare_parameter<std::vector<double>>("marker_color_rgba", {0.0, 1.0, 0.0, 0.25});
        this->declare_parameter<double>("marker_lifetime_sec", 0.0);
        this->declare_parameter<std::string>("marker_namespace", "lidar_detection_boxes");

        this->get_parameter("processing_rate", processing_rate_);
        this->get_parameter("crop_box_min", crop_box_min_);
        this->get_parameter("crop_box_max", crop_box_max_);
        this->get_parameter("voxel_leaf_size", voxel_leaf_size_);
        this->get_parameter("publish_processed_lidar_pc", publish_processed_lidar_pc_);
        this->get_parameter("enable_ground_segmentation", enable_ground_segmentation_);
        this->get_parameter("min_cluster_points", min_cluster_points_);
        this->get_parameter("max_cluster_points", max_cluster_points_);
        this->get_parameter("min_cluster_size", min_cluster_size_);
        this->get_parameter("max_cluster_size", max_cluster_size_);
        this->get_parameter("profiling_interval_frames", profiling_interval_frames_);
        this->get_parameter("enable_csv_logging", csv_logging_);
        this->get_parameter("csv_log_dir", csv_log_dir_);
        this->get_parameter("dataset_sequence", dataset_sequence_);
        input_queue_size_ = std::max(1, static_cast<int>(this->get_parameter("input_queue_size").as_int()));
        max_buffer_age_ms_ = std::max(0.0, this->get_parameter("max_buffer_age_ms").as_double());
        processing_timer_fallback_period_ms_ = std::max(1, static_cast<int>(this->get_parameter("processing_timer_fallback_period_ms").as_int()));
        lidar_input_qos_depth_ = std::max(1, static_cast<int>(this->get_parameter("lidar_input_qos_depth").as_int()));
        processed_cloud_qos_depth_ = std::max(1, static_cast<int>(this->get_parameter("processed_cloud_qos_depth").as_int()));
        detection_qos_depth_ = std::max(1, static_cast<int>(this->get_parameter("detection_qos_depth").as_int()));
        marker_qos_depth_ = std::max(1, static_cast<int>(this->get_parameter("marker_qos_depth").as_int()));
        ground_segmentation_distance_threshold_m_ = std::max(0.0, this->get_parameter("ground_segmentation_distance_threshold_m").as_double());
        ground_segmentation_max_iterations_ = std::max(1, static_cast<int>(this->get_parameter("ground_segmentation_max_iterations").as_int()));
        this->get_parameter("ground_segmentation_axis", ground_segmentation_axis_);
        ground_segmentation_eps_angle_deg_ = std::max(0.0, this->get_parameter("ground_segmentation_eps_angle_deg").as_double());
        this->get_parameter("clustering_voxel_leaf_size", clustering_voxel_leaf_size_);
        this->get_parameter("enable_range_adaptive_clustering", enable_range_adaptive_clustering_);
        this->get_parameter("clustering_range_breakpoints_m", clustering_range_breakpoints_m_);
        this->get_parameter("clustering_tolerance_by_range_m", clustering_tolerance_by_range_m_);
        marker_min_dimension_m_ = std::max(0.0, this->get_parameter("marker_min_dimension_m").as_double());
        this->get_parameter("marker_color_rgba", marker_color_rgba_);
        marker_lifetime_sec_ = std::max(0.0, this->get_parameter("marker_lifetime_sec").as_double());
        this->get_parameter("marker_namespace", marker_namespace_);

        if (ground_segmentation_axis_.size() != 3)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Parameter 'ground_segmentation_axis' must have 3 elements. Falling back to [0.0, 0.0, 1.0].");
            ground_segmentation_axis_ = {0.0, 0.0, 1.0};
        }

        if (marker_color_rgba_.size() != 4)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Parameter 'marker_color_rgba' must have 4 elements. Falling back to [0.0, 1.0, 0.0, 0.25].");
            marker_color_rgba_ = {0.0, 1.0, 0.0, 0.25};
        }

        if (clustering_voxel_leaf_size_.size() != 3 ||
            clustering_voxel_leaf_size_[0] <= 0.0 ||
            clustering_voxel_leaf_size_[1] <= 0.0 ||
            clustering_voxel_leaf_size_[2] <= 0.0)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Parameter 'clustering_voxel_leaf_size' must have 3 positive elements. Falling back to [0.3, 0.3, 0.3].");
            clustering_voxel_leaf_size_ = {0.3, 0.3, 0.3};
        }

        if (clustering_tolerance_by_range_m_.empty())
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Parameter 'clustering_tolerance_by_range_m' must not be empty. Falling back to [1.0].");
            clustering_tolerance_by_range_m_ = {1.0};
            clustering_range_breakpoints_m_.clear();
            enable_range_adaptive_clustering_ = false;
        }

        const bool adaptive_tolerances_positive = std::all_of(
            clustering_tolerance_by_range_m_.begin(),
            clustering_tolerance_by_range_m_.end(),
            [](double tolerance_m) { return tolerance_m > 0.0; });

        const bool adaptive_band_count_valid =
            clustering_tolerance_by_range_m_.size() == (clustering_range_breakpoints_m_.size() + 1);
        const bool adaptive_breakpoints_sorted = std::is_sorted(
            clustering_range_breakpoints_m_.begin(),
            clustering_range_breakpoints_m_.end());

        if (!adaptive_band_count_valid || !adaptive_breakpoints_sorted || !adaptive_tolerances_positive)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Range-adaptive clustering config is invalid. Falling back to a single clustering tolerance band.");
            clustering_tolerance_by_range_m_ = {1.0};
            clustering_range_breakpoints_m_.clear();
            enable_range_adaptive_clustering_ = false;
        }
        
        crop_box_min_vec = Eigen::Vector4f(crop_box_min_[0], crop_box_min_[1], crop_box_min_[2], 1.0);
        crop_box_max_vec = Eigen::Vector4f(crop_box_max_[0], crop_box_max_[1], crop_box_max_[2], 1.0);
        voxel_leaf_size_vec = Eigen::Vector4f(voxel_leaf_size_[0], voxel_leaf_size_[1], voxel_leaf_size_[2], 1.0);
        min_cluster_size_vec_ = Eigen::Vector3f(min_cluster_size_[0], min_cluster_size_[1], min_cluster_size_[2]);
        max_cluster_size_vec_ = Eigen::Vector3f(max_cluster_size_[0], max_cluster_size_[1], max_cluster_size_[2]);
        clustering_voxel_leaf_size_vec_ = Eigen::Vector3f(
            clustering_voxel_leaf_size_[0],
            clustering_voxel_leaf_size_[1],
            clustering_voxel_leaf_size_[2]);
        ground_segmentation_axis_vec_ = Eigen::Vector3f(
            ground_segmentation_axis_[0],
            ground_segmentation_axis_[1],
            ground_segmentation_axis_[2]);

        const auto lidar_input_qos =
            rclcpp::SensorDataQoS().keep_last(static_cast<std::size_t>(lidar_input_qos_depth_));
        const auto processed_cloud_qos =
            rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(processed_cloud_qos_depth_)))
                .reliable()
                .durability_volatile();
        const auto detection_qos =
            rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(detection_qos_depth_)))
                .reliable()
                .durability_volatile();
        const auto marker_qos =
            rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(marker_qos_depth_)))
                .reliable()
                .durability_volatile();

        processing_timer_ = this->create_wall_timer(
            processing_rate_ > 0 ? std::chrono::milliseconds(static_cast<int>(1000.0 / processing_rate_))
                                 : std::chrono::milliseconds(processing_timer_fallback_period_ms_),
            std::bind(&LidarProcessing::process_latest_point_cloud, this));
        lidar_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "lidar_in",
            lidar_input_qos,
            std::bind(&LidarProcessing::lidar_subscriber_callback, this, std::placeholders::_1));
        if (publish_processed_lidar_pc_)
        {
            processed_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "lidar_out",
                processed_cloud_qos);
        }
        object_detection_publisher_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(
            "lidar_detections",
            detection_qos);
        object_detection_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "lidar_detection_markers",
            marker_qos);

        initialize_csv_logging();
        reset_resource_window();
        RCLCPP_INFO(this->get_logger(), "LidarProcessing node has been initialized.");

    }

    void lidar_subscriber_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        if (static_cast<int>(cloud_queue_.size()) >= input_queue_size_)
        {
            cloud_queue_.pop_front();
            total_overwritten_frames_++;
        }
        cloud_queue_.push_back(BufferedCloud{msg, std::chrono::steady_clock::now()});
        total_received_frames_++;
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
                const auto now_steady = std::chrono::steady_clock::now();
                while (!cloud_queue_.empty())
                {
                    const double queue_age_ms =
                        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                            now_steady - cloud_queue_.front().received_steady)
                            .count();
                    if (queue_age_ms <= max_buffer_age_ms_)
                    {
                        break;
                    }

                    cloud_queue_.pop_front();
                    total_overwritten_frames_++;
                }

                if (!cloud_queue_.empty())
                {
                    cloud_to_process_ = cloud_queue_.front().msg;
                    cloud_received_steady_ = cloud_queue_.front().received_steady;
                    cloud_queue_.pop_front();
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

            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> raw_clusters;
            {
                ScopedTimer clustering_timer(metrics.clustering_time_ms);
                raw_clusters = voxelBasedClustering(output_cloud);
            }

            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obstacle_clusters;
            {
                ScopedTimer cluster_filtering_timer(metrics.cluster_filtering_time_ms);
                obstacle_clusters = filterClusters(raw_clusters);
            }

            std::vector<ClusterBox> cluster_boxes;
            {
                ScopedTimer bounding_box_timer(metrics.bounding_box_time_ms);
                cluster_boxes = compute3dBoundingBoxes(obstacle_clusters);
            }

            vision_msgs::msg::Detection3DArray detection_array;
            {
                ScopedTimer detection_conversion_timer(metrics.detection_conversion_time_ms);
                detection_array = convert_3d_bounding_box_to_ros2_message(cluster_boxes, cloud_to_process_->header);
            }

            visualization_msgs::msg::MarkerArray detection_markers;
            {
                ScopedTimer marker_conversion_timer(metrics.marker_conversion_time_ms);
                detection_markers = convert_3d_bounding_box_to_marker_array(cluster_boxes, cloud_to_process_->header);
            }

            RCLCPP_DEBUG(
                this->get_logger(),
                "Extracted %zu raw clusters, kept %zu filtered clusters, and published %zu lidar detections from %zu lidar points",
                raw_clusters.size(),
                obstacle_clusters.size(),
                detection_array.detections.size(),
                output_cloud ? output_cloud->points.size() : 0U);

            {
                ScopedTimer publish_timer(metrics.publish_time_ms);
                {
                    ScopedTimer processed_cloud_publish_timer(metrics.processed_cloud_publish_time_ms);
                    publish_processed_cloud(output_cloud, cloud_to_process_->header);
                }
                {
                    ScopedTimer detection_publish_timer(metrics.detection_publish_time_ms);
                    publish3dBoundingBoxes_2_ros(detection_array);
                }
                {
                    ScopedTimer marker_publish_timer(metrics.marker_publish_time_ms);
                    publish3dBoundingBoxMarkers_2_ros(detection_markers);
                }
            }

            if (output_cloud)
            {
                metrics.output_points = output_cloud->points.size();
            }
            metrics.raw_clusters = raw_clusters.size();
            metrics.filtered_clusters = obstacle_clusters.size();
            metrics.output_detections = detection_array.detections.size();
            metrics.rss_mb = read_current_rss_mb();
            metrics.peak_rss_mb = read_peak_rss_mb();

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
        seg.setDistanceThreshold(ground_segmentation_distance_threshold_m_);
        seg.setMaxIterations(ground_segmentation_max_iterations_);
        seg.setAxis(ground_segmentation_axis_vec_);
        seg.setEpsAngle(pcl::deg2rad(ground_segmentation_eps_angle_deg_));
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

    sensor_msgs::msg::PointCloud2::SharedPtr convert_pcl_to_ros2_pc(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
        const std_msgs::msg::Header &header)
    {
        // Convert the ROS PointCloud2 message to a PCL point cloud
        auto ros2_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*cloud, *ros2_cloud);
        ros2_cloud->header = header;
        return ros2_cloud;
    }

    void publish_processed_cloud(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
        const std_msgs::msg::Header &header)
    {
        if (!publish_processed_lidar_pc_ || !processed_cloud_publisher_)
        {
            return;
        }

        if (!cloud)
        {
            RCLCPP_WARN(this->get_logger(), "No point cloud available for publishing");
            return;
        }

        auto ros2_cloud = convert_pcl_to_ros2_pc(cloud, header);
        processed_cloud_publisher_->publish(*ros2_cloud);
    }

    struct VoxelKey
    {
        int x{0};
        int y{0};
        int z{0};

        bool operator==(const VoxelKey &other) const
        {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct VoxelKeyHash
    {
        std::size_t operator()(const VoxelKey &key) const noexcept
        {
            std::size_t seed = 0;
            seed ^= std::hash<int>{}(key.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= std::hash<int>{}(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= std::hash<int>{}(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            return seed;
        }
    };

    struct VoxelCell
    {
        std::vector<int> point_indices;
        Eigen::Vector3f centroid{Eigen::Vector3f::Zero()};
        double range_m{0.0};
        bool visited{false};
    };

    VoxelKey computeVoxelKey(const pcl::PointXYZI &point) const
    {
        return VoxelKey{
            static_cast<int>(std::floor(point.x / clustering_voxel_leaf_size_vec_.x())),
            static_cast<int>(std::floor(point.y / clustering_voxel_leaf_size_vec_.y())),
            static_cast<int>(std::floor(point.z / clustering_voxel_leaf_size_vec_.z()))};
    }

    double getRangeAdaptiveClusterTolerance(double range_m) const
    {
        if (!enable_range_adaptive_clustering_ || clustering_tolerance_by_range_m_.empty())
        {
            return clustering_tolerance_by_range_m_.empty() ? 1.0 : clustering_tolerance_by_range_m_.front();
        }

        for (std::size_t band_index = 0; band_index < clustering_range_breakpoints_m_.size(); ++band_index)
        {
            if (range_m <= clustering_range_breakpoints_m_[band_index])
            {
                return clustering_tolerance_by_range_m_[band_index];
            }
        }

        return clustering_tolerance_by_range_m_.back();
    }

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> voxelBasedClustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) const
    {
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;

        if (!cloud || cloud->empty())
        {
            return clusters;
        }

        std::unordered_map<VoxelKey, VoxelCell, VoxelKeyHash> occupied_voxels;
        occupied_voxels.reserve(cloud->points.size());

        for (std::size_t point_index = 0; point_index < cloud->points.size(); ++point_index)
        {
            const auto &point = cloud->points[point_index];
            auto &cell = occupied_voxels[computeVoxelKey(point)];
            cell.point_indices.push_back(static_cast<int>(point_index));
            cell.centroid += point.getVector3fMap();
        }

        for (auto &voxel_entry : occupied_voxels)
        {
            auto &cell = voxel_entry.second;
            cell.centroid /= static_cast<float>(cell.point_indices.size());
            cell.range_m = std::hypot(static_cast<double>(cell.centroid.x()), static_cast<double>(cell.centroid.y()));
        }

        clusters.reserve(occupied_voxels.size());
        for (auto &seed_entry : occupied_voxels)
        {
            auto &seed_cell = seed_entry.second;
            if (seed_cell.visited)
            {
                continue;
            }

            seed_cell.visited = true;
            std::deque<VoxelKey> voxel_queue{seed_entry.first};
            std::vector<int> cluster_point_indices;

            while (!voxel_queue.empty())
            {
                const VoxelKey current_key = voxel_queue.front();
                voxel_queue.pop_front();

                auto current_it = occupied_voxels.find(current_key);
                if (current_it == occupied_voxels.end())
                {
                    continue;
                }

                const auto &current_cell = current_it->second;
                cluster_point_indices.insert(
                    cluster_point_indices.end(),
                    current_cell.point_indices.begin(),
                    current_cell.point_indices.end());

                const double current_tolerance = getRangeAdaptiveClusterTolerance(current_cell.range_m);
                const int neighbor_x = std::max(
                    1,
                    static_cast<int>(std::ceil(current_tolerance / static_cast<double>(clustering_voxel_leaf_size_vec_.x()))));
                const int neighbor_y = std::max(
                    1,
                    static_cast<int>(std::ceil(current_tolerance / static_cast<double>(clustering_voxel_leaf_size_vec_.y()))));
                const int neighbor_z = std::max(
                    1,
                    static_cast<int>(std::ceil(current_tolerance / static_cast<double>(clustering_voxel_leaf_size_vec_.z()))));

                for (int dx = -neighbor_x; dx <= neighbor_x; ++dx)
                {
                    for (int dy = -neighbor_y; dy <= neighbor_y; ++dy)
                    {
                        for (int dz = -neighbor_z; dz <= neighbor_z; ++dz)
                        {
                            if (dx == 0 && dy == 0 && dz == 0)
                            {
                                continue;
                            }

                            const VoxelKey neighbor_key{current_key.x + dx, current_key.y + dy, current_key.z + dz};
                            auto neighbor_it = occupied_voxels.find(neighbor_key);
                            if (neighbor_it == occupied_voxels.end() || neighbor_it->second.visited)
                            {
                                continue;
                            }

                            const auto &neighbor_cell = neighbor_it->second;
                            const double neighbor_tolerance = getRangeAdaptiveClusterTolerance(neighbor_cell.range_m);
                            const double pair_tolerance = std::max(current_tolerance, neighbor_tolerance);
                            const double centroid_distance =
                                (current_cell.centroid - neighbor_cell.centroid).norm();

                            if (centroid_distance > pair_tolerance)
                            {
                                continue;
                            }

                            neighbor_it->second.visited = true;
                            voxel_queue.push_back(neighbor_key);
                        }
                    }
                }
            }

            auto cluster_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            cluster_cloud->points.reserve(cluster_point_indices.size());
            for (const int point_index : cluster_point_indices)
            {
                cluster_cloud->points.push_back(cloud->points[point_index]);
            }

            cluster_cloud->width = static_cast<std::uint32_t>(cluster_cloud->points.size());
            cluster_cloud->height = 1;
            cluster_cloud->is_dense = cloud->is_dense;
            clusters.push_back(cluster_cloud);
        }

        return clusters;
    }

    bool clusterPassesPointCountFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cluster) const
    {
        if (!cluster)
        {
            return false;
        }

        const auto point_count = static_cast<int>(cluster->points.size());
        return point_count >= min_cluster_points_ && point_count <= max_cluster_points_;
    }

    bool clusterPassesSizeFilter(const Eigen::Vector3f &cluster_dimensions) const
    {
        return (cluster_dimensions.array() >= min_cluster_size_vec_.array()).all() &&
               (cluster_dimensions.array() <= max_cluster_size_vec_.array()).all();
    }

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> filterClusters(
        const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &clusters) const
    {
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> filtered_clusters;
        filtered_clusters.reserve(clusters.size());

        for (const auto &cluster : clusters)
        {
            if (!clusterPassesPointCountFilter(cluster))
            {
                continue;
            }

            const auto cluster_box = computeOrientedClusterBox(cluster);
            if (!clusterPassesSizeFilter(cluster_box.size))
            {
                continue;
            }

            filtered_clusters.push_back(cluster);
        }

        return filtered_clusters;
    }

    struct ClusterBox
    {
        Eigen::Vector3f center{Eigen::Vector3f::Zero()};
        Eigen::Vector3f size{Eigen::Vector3f::Zero()};
        Eigen::Quaternionf orientation{Eigen::Quaternionf::Identity()};
    };

    ClusterBox computeOrientedClusterBox(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cluster) const
    {
        ClusterBox box;
        if (!cluster || cluster->empty())
        {
            return box;
        }

        Eigen::Vector4f centroid_4f = Eigen::Vector4f::Zero();
        pcl::compute3DCentroid(*cluster, centroid_4f);
        const Eigen::Vector2f centroid_xy = centroid_4f.head<2>();

        Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();
        for (const auto &point : cluster->points)
        {
            const Eigen::Vector2f delta(point.x - centroid_xy.x(), point.y - centroid_xy.y());
            covariance += delta * delta.transpose();
        }
        covariance /= static_cast<float>(cluster->points.size());

        Eigen::Vector2f principal_axis = Eigen::Vector2f::UnitX();
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigen_solver(covariance);
        if (eigen_solver.info() == Eigen::Success)
        {
            principal_axis = eigen_solver.eigenvectors().col(1);
            if (principal_axis.x() < 0.0f ||
                (std::abs(principal_axis.x()) < 1e-5f && principal_axis.y() < 0.0f))
            {
                principal_axis = -principal_axis;
            }
        }

        const float yaw = std::atan2(principal_axis.y(), principal_axis.x());
        const Eigen::Matrix2f rotation =
            Eigen::Rotation2Df(yaw).toRotationMatrix();

        Eigen::Vector2f min_local = Eigen::Vector2f::Constant(std::numeric_limits<float>::max());
        Eigen::Vector2f max_local = Eigen::Vector2f::Constant(std::numeric_limits<float>::lowest());
        float min_z = std::numeric_limits<float>::max();
        float max_z = std::numeric_limits<float>::lowest();

        for (const auto &point : cluster->points)
        {
            const Eigen::Vector2f point_xy(point.x, point.y);
            const Eigen::Vector2f local_xy = rotation.transpose() * (point_xy - centroid_xy);
            min_local = min_local.cwiseMin(local_xy);
            max_local = max_local.cwiseMax(local_xy);
            min_z = std::min(min_z, point.z);
            max_z = std::max(max_z, point.z);
        }

        const Eigen::Vector2f local_center = 0.5f * (min_local + max_local);
        const Eigen::Vector2f world_center_xy = centroid_xy + (rotation * local_center);

        box.center = Eigen::Vector3f(world_center_xy.x(), world_center_xy.y(), 0.5f * (min_z + max_z));
        box.size = Eigen::Vector3f(
            std::max(0.0f, max_local.x() - min_local.x()),
            std::max(0.0f, max_local.y() - min_local.y()),
            std::max(0.0f, max_z - min_z));
        box.orientation = Eigen::Quaternionf(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
        return box;
    }

    std::vector<ClusterBox> compute3dBoundingBoxes(
        const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &clusters) const
    {
        std::vector<ClusterBox> cluster_boxes;
        cluster_boxes.reserve(clusters.size());

        for (const auto &cluster : clusters)
        {
            cluster_boxes.push_back(computeOrientedClusterBox(cluster));
        }

        return cluster_boxes;
    }

    vision_msgs::msg::Detection3DArray convert_3d_bounding_box_to_ros2_message(
        const std::vector<ClusterBox> &cluster_boxes,
        const std_msgs::msg::Header &header) const
    {
        vision_msgs::msg::Detection3DArray detection_array;
        detection_array.header = header;
        detection_array.detections.reserve(cluster_boxes.size());

        for (std::size_t cluster_index = 0; cluster_index < cluster_boxes.size(); ++cluster_index)
        {
            const auto &cluster_box = cluster_boxes[cluster_index];

            vision_msgs::msg::Detection3D detection_msg;
            detection_msg.header = header;
            detection_msg.id = "cluster_" + std::to_string(cluster_index);
            detection_msg.bbox.center.position.x = cluster_box.center.x();
            detection_msg.bbox.center.position.y = cluster_box.center.y();
            detection_msg.bbox.center.position.z = cluster_box.center.z();
            detection_msg.bbox.center.orientation.x = cluster_box.orientation.x();
            detection_msg.bbox.center.orientation.y = cluster_box.orientation.y();
            detection_msg.bbox.center.orientation.z = cluster_box.orientation.z();
            detection_msg.bbox.center.orientation.w = cluster_box.orientation.w();
            detection_msg.bbox.size.x = cluster_box.size.x();
            detection_msg.bbox.size.y = cluster_box.size.y();
            detection_msg.bbox.size.z = cluster_box.size.z();
            detection_array.detections.push_back(detection_msg);
        }

        return detection_array;
    }

    visualization_msgs::msg::MarkerArray convert_3d_bounding_box_to_marker_array(
        const std::vector<ClusterBox> &cluster_boxes,
        const std_msgs::msg::Header &header) const
    {
        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.reserve(cluster_boxes.size());

        for (std::size_t cluster_index = 0; cluster_index < cluster_boxes.size(); ++cluster_index)
        {
            const auto &cluster_box = cluster_boxes[cluster_index];

            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.header.stamp = rclcpp::Time(0);
            marker.ns = marker_namespace_;
            marker.id = static_cast<int>(cluster_index);
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = cluster_box.center.x();
            marker.pose.position.y = cluster_box.center.y();
            marker.pose.position.z = cluster_box.center.z();
            marker.pose.orientation.x = cluster_box.orientation.x();
            marker.pose.orientation.y = cluster_box.orientation.y();
            marker.pose.orientation.z = cluster_box.orientation.z();
            marker.pose.orientation.w = cluster_box.orientation.w();
            marker.scale.x = std::max(static_cast<double>(cluster_box.size.x()), marker_min_dimension_m_);
            marker.scale.y = std::max(static_cast<double>(cluster_box.size.y()), marker_min_dimension_m_);
            marker.scale.z = std::max(static_cast<double>(cluster_box.size.z()), marker_min_dimension_m_);
            marker.color.r = static_cast<float>(marker_color_rgba_[0]);
            marker.color.g = static_cast<float>(marker_color_rgba_[1]);
            marker.color.b = static_cast<float>(marker_color_rgba_[2]);
            marker.color.a = static_cast<float>(marker_color_rgba_[3]);
            marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);
            marker.frame_locked = false;
            marker_array.markers.push_back(marker);
        }

        return marker_array;
    }

    void publish3dBoundingBoxes_2_ros(const vision_msgs::msg::Detection3DArray &detection_array)
    {
        object_detection_publisher_->publish(detection_array);
    }

    void publish3dBoundingBoxMarkers_2_ros(const visualization_msgs::msg::MarkerArray &marker_array)
    {
        object_detection_marker_publisher_->publish(marker_array);
    }
private:
    struct PointCloudProcessingMetrics
    {
        double buffer_age_ms{0.0};
        double conversion_time_ms{0.0};
        double crop_box_time_ms{0.0};
        double voxelization_time_ms{0.0};
        double ground_segmentation_time_ms{0.0};
        double clustering_time_ms{0.0};
        double cluster_filtering_time_ms{0.0};
        double bounding_box_time_ms{0.0};
        double detection_conversion_time_ms{0.0};
        double marker_conversion_time_ms{0.0};
        double publish_time_ms{0.0};
        double processed_cloud_publish_time_ms{0.0};
        double detection_publish_time_ms{0.0};
        double marker_publish_time_ms{0.0};
        double frame_total_time_ms{0.0};
        double rss_mb{0.0};
        double peak_rss_mb{0.0};
        std::size_t input_points{0};
        std::size_t output_points{0};
        std::size_t raw_clusters{0};
        std::size_t filtered_clusters{0};
        std::size_t output_detections{0};
    };

    void update_profiling_metrics(const PointCloudProcessingMetrics &metrics)
    {
        interval_sum_buffer_age_ms_ += metrics.buffer_age_ms;
        interval_sum_conversion_ms_ += metrics.conversion_time_ms;
        interval_sum_crop_box_ms_ += metrics.crop_box_time_ms;
        interval_sum_voxelization_ms_ += metrics.voxelization_time_ms;
        interval_sum_ground_segmentation_ms_ += metrics.ground_segmentation_time_ms;
        interval_sum_clustering_ms_ += metrics.clustering_time_ms;
        interval_sum_cluster_filtering_ms_ += metrics.cluster_filtering_time_ms;
        interval_sum_bounding_box_ms_ += metrics.bounding_box_time_ms;
        interval_sum_detection_conversion_ms_ += metrics.detection_conversion_time_ms;
        interval_sum_marker_conversion_ms_ += metrics.marker_conversion_time_ms;
        interval_sum_publish_ms_ += metrics.publish_time_ms;
        interval_sum_processed_cloud_publish_ms_ += metrics.processed_cloud_publish_time_ms;
        interval_sum_detection_publish_ms_ += metrics.detection_publish_time_ms;
        interval_sum_marker_publish_ms_ += metrics.marker_publish_time_ms;
        interval_sum_frame_total_ms_ += metrics.frame_total_time_ms;
        interval_sum_rss_mb_ += metrics.rss_mb;
        interval_sum_peak_rss_mb_ += metrics.peak_rss_mb;
        interval_sum_input_points_ += metrics.input_points;
        interval_sum_output_points_ += metrics.output_points;
        interval_sum_raw_clusters_ += metrics.raw_clusters;
        interval_sum_filtered_clusters_ += metrics.filtered_clusters;
        interval_sum_output_detections_ += metrics.output_detections;
        interval_frame_count_++;

        if (interval_frame_count_ >= profiling_interval_frames_)
        {
            const double avg_buffer_age_ms = interval_sum_buffer_age_ms_ / interval_frame_count_;
            const double avg_conversion_ms = interval_sum_conversion_ms_ / interval_frame_count_;
            const double avg_crop_box_ms = interval_sum_crop_box_ms_ / interval_frame_count_;
            const double avg_voxelization_ms = interval_sum_voxelization_ms_ / interval_frame_count_;
            const double avg_ground_segmentation_ms = interval_sum_ground_segmentation_ms_ / interval_frame_count_;
            const double avg_clustering_ms = interval_sum_clustering_ms_ / interval_frame_count_;
            const double avg_cluster_filtering_ms = interval_sum_cluster_filtering_ms_ / interval_frame_count_;
            const double avg_bounding_box_ms = interval_sum_bounding_box_ms_ / interval_frame_count_;
            const double avg_detection_conversion_ms = interval_sum_detection_conversion_ms_ / interval_frame_count_;
            const double avg_marker_conversion_ms = interval_sum_marker_conversion_ms_ / interval_frame_count_;
            const double avg_publish_ms = interval_sum_publish_ms_ / interval_frame_count_;
            const double avg_processed_cloud_publish_ms = interval_sum_processed_cloud_publish_ms_ / interval_frame_count_;
            const double avg_detection_publish_ms = interval_sum_detection_publish_ms_ / interval_frame_count_;
            const double avg_marker_publish_ms = interval_sum_marker_publish_ms_ / interval_frame_count_;
            const double avg_frame_total_ms = interval_sum_frame_total_ms_ / interval_frame_count_;
            const double avg_rss_mb = interval_sum_rss_mb_ / interval_frame_count_;
            const double avg_peak_rss_mb = interval_sum_peak_rss_mb_ / interval_frame_count_;
            const double avg_input_points = interval_sum_input_points_ / interval_frame_count_;
            const double avg_output_points = interval_sum_output_points_ / interval_frame_count_;
            const double avg_raw_clusters = interval_sum_raw_clusters_ / interval_frame_count_;
            const double avg_filtered_clusters = interval_sum_filtered_clusters_ / interval_frame_count_;
            const double avg_output_detections = interval_sum_output_detections_ / interval_frame_count_;
            double avg_process_cpu_percent = 0.0;
            double effective_output_rate_hz = 0.0;
            compute_interval_resource_metrics(avg_process_cpu_percent, effective_output_rate_hz);

            write_csv_interval_metrics(
                interval_frame_count_,
                avg_buffer_age_ms,
                avg_conversion_ms,
                avg_crop_box_ms,
                avg_voxelization_ms,
                avg_ground_segmentation_ms,
                avg_clustering_ms,
                avg_cluster_filtering_ms,
                avg_bounding_box_ms,
                avg_detection_conversion_ms,
                avg_marker_conversion_ms,
                avg_publish_ms,
                avg_processed_cloud_publish_ms,
                avg_detection_publish_ms,
                avg_marker_publish_ms,
                avg_frame_total_ms,
                avg_process_cpu_percent,
                effective_output_rate_hz,
                avg_rss_mb,
                avg_peak_rss_mb,
                avg_input_points,
                avg_output_points,
                avg_raw_clusters,
                avg_filtered_clusters,
                avg_output_detections);

            // RCLCPP_INFO(
            //     this->get_logger(),
            //     "Average point cloud processing metrics over last %d frames: Buffer Age: %.2f ms, Conversion Time: %.2f ms, Crop Box Time: %.2f ms, Voxelization Time: %.2f ms, Ground Segmentation Time: %.2f ms, Publish Time: %.2f ms, Frame Total Time: %.2f ms, Average Input Points: %.2f, Average Output Points: %.2f, Total Received Frames: %ld, Total Processed Frames: %ld, Total Overwritten Frames: %ld",
            //     profiling_interval_frames_,
            //     avg_buffer_age_ms,
            //     avg_conversion_ms,
            //     avg_crop_box_ms,
            //     avg_voxelization_ms,
            //     avg_ground_segmentation_ms,
            //     avg_publish_ms,
            //     avg_frame_total_ms,
            //     avg_input_points,
            //     avg_output_points,
            //     total_received_frames_,
            //     total_processed_frames_,
            //     total_overwritten_frames_);

            interval_frame_count_ = 0;
            interval_sum_buffer_age_ms_ = 0.0;
            interval_sum_conversion_ms_ = 0.0;
            interval_sum_crop_box_ms_ = 0.0;
            interval_sum_voxelization_ms_ = 0.0;
            interval_sum_ground_segmentation_ms_ = 0.0;
            interval_sum_clustering_ms_ = 0.0;
            interval_sum_cluster_filtering_ms_ = 0.0;
            interval_sum_bounding_box_ms_ = 0.0;
            interval_sum_detection_conversion_ms_ = 0.0;
            interval_sum_marker_conversion_ms_ = 0.0;
            interval_sum_publish_ms_ = 0.0;
            interval_sum_processed_cloud_publish_ms_ = 0.0;
            interval_sum_detection_publish_ms_ = 0.0;
            interval_sum_marker_publish_ms_ = 0.0;
            interval_sum_frame_total_ms_ = 0.0;
            interval_sum_rss_mb_ = 0.0;
            interval_sum_peak_rss_mb_ = 0.0;
            interval_sum_input_points_ = 0.0;
            interval_sum_output_points_ = 0.0;
            interval_sum_raw_clusters_ = 0.0;
            interval_sum_filtered_clusters_ = 0.0;
            interval_sum_output_detections_ = 0.0;
        }
    }

    void initialize_csv_logging()
    {
        if (!csv_logging_)
        {
            return;
        }

        try
        {
            const std::filesystem::path log_directory(csv_log_dir_);
            std::filesystem::create_directories(log_directory);

            csv_log_file_path_ = (log_directory / build_csv_filename()).string();
            csv_log_stream_.open(csv_log_file_path_, std::ios::out | std::ios::trunc);
            if (!csv_log_stream_.is_open())
            {
                RCLCPP_WARN(this->get_logger(), "Failed to open CSV log file at '%s'. Disabling CSV logging.", csv_log_file_path_.c_str());
                csv_logging_ = false;
                return;
            }

            csv_log_stream_ << "timestamp_utc,dataset_sequence,interval_frames,avg_buffer_age_ms,avg_conversion_time_ms,avg_crop_box_time_ms,avg_voxelization_time_ms,avg_ground_segmentation_time_ms,avg_clustering_time_ms,avg_cluster_filtering_time_ms,avg_bounding_box_time_ms,avg_detection_conversion_time_ms,avg_marker_conversion_time_ms,avg_publish_time_ms,avg_processed_cloud_publish_time_ms,avg_detection_publish_time_ms,avg_marker_publish_time_ms,avg_frame_total_time_ms,avg_process_cpu_percent,effective_output_rate_hz,avg_rss_mb,avg_peak_rss_mb,avg_input_points,avg_output_points,avg_raw_clusters,avg_filtered_clusters,avg_output_detections,total_received_frames,total_processed_frames,total_overwritten_frames\n";
            csv_log_stream_.flush();
            RCLCPP_INFO(this->get_logger(), "CSV logging enabled. Writing interval metrics to '%s'.", csv_log_file_path_.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to initialize CSV logging: %s. Disabling CSV logging.", e.what());
            csv_logging_ = false;
        }
    }

    void write_csv_interval_metrics(
        int interval_frames,
        double avg_buffer_age_ms,
        double avg_conversion_ms,
        double avg_crop_box_ms,
        double avg_voxelization_ms,
        double avg_ground_segmentation_ms,
        double avg_clustering_ms,
        double avg_cluster_filtering_ms,
        double avg_bounding_box_ms,
        double avg_detection_conversion_ms,
        double avg_marker_conversion_ms,
        double avg_publish_ms,
        double avg_processed_cloud_publish_ms,
        double avg_detection_publish_ms,
        double avg_marker_publish_ms,
        double avg_frame_total_ms,
        double avg_process_cpu_percent,
        double effective_output_rate_hz,
        double avg_rss_mb,
        double avg_peak_rss_mb,
        double avg_input_points,
        double avg_output_points,
        double avg_raw_clusters,
        double avg_filtered_clusters,
        double avg_output_detections)
    {
        if (!csv_logging_ || !csv_log_stream_.is_open())
        {
            return;
        }

        csv_log_stream_ << current_utc_timestamp("%Y-%m-%dT%H:%M:%SZ") << ','
                        << dataset_sequence_ << ','
                        << interval_frames << ','
                        << std::fixed << std::setprecision(2)
                        << avg_buffer_age_ms << ','
                        << avg_conversion_ms << ','
                        << avg_crop_box_ms << ','
                        << avg_voxelization_ms << ','
                        << avg_ground_segmentation_ms << ','
                        << avg_clustering_ms << ','
                        << avg_cluster_filtering_ms << ','
                        << avg_bounding_box_ms << ','
                        << avg_detection_conversion_ms << ','
                        << avg_marker_conversion_ms << ','
                        << avg_publish_ms << ','
                        << avg_processed_cloud_publish_ms << ','
                        << avg_detection_publish_ms << ','
                        << avg_marker_publish_ms << ','
                        << avg_frame_total_ms << ','
                        << avg_process_cpu_percent << ','
                        << effective_output_rate_hz << ','
                        << avg_rss_mb << ','
                        << avg_peak_rss_mb << ','
                        << avg_input_points << ','
                        << avg_output_points << ','
                        << avg_raw_clusters << ','
                        << avg_filtered_clusters << ','
                        << avg_output_detections << ','
                        << total_received_frames_ << ','
                        << total_processed_frames_ << ','
                        << total_overwritten_frames_ << '\n';
        csv_log_stream_.flush();
    }

    std::string build_csv_filename() const
    {
        std::ostringstream filename_builder;
        filename_builder << this->get_name()
                         << "_seq_"
                         << sanitize_for_filename(dataset_sequence_)
                         << "_"
                         << current_utc_timestamp("%Y-%m-%dT%H-%M-%S")
                         << ".csv";
        return filename_builder.str();
    }

    std::string current_utc_timestamp(const char *format) const
    {
        const auto now = std::chrono::system_clock::now();
        const auto now_time_t = std::chrono::system_clock::to_time_t(now);
        std::tm utc_time{};
        gmtime_r(&now_time_t, &utc_time);

        std::ostringstream timestamp_builder;
        timestamp_builder << std::put_time(&utc_time, format);
        return timestamp_builder.str();
    }

    static std::string sanitize_for_filename(std::string value)
    {
        for (char &character : value)
        {
            if (!std::isalnum(static_cast<unsigned char>(character)) && character != '-' && character != '_')
            {
                character = '_';
            }
        }
        return value;
    }

    void reset_resource_window()
    {
        interval_resource_window_start_ = std::chrono::steady_clock::now();
        interval_resource_window_cpu_ms_ = read_process_cpu_time_ms();
    }

    void compute_interval_resource_metrics(
        double &avg_process_cpu_percent,
        double &effective_output_rate_hz)
    {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed_wall_ms =
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                now - interval_resource_window_start_)
                .count();
        const double current_cpu_ms = read_process_cpu_time_ms();
        const double elapsed_cpu_ms =
            std::max(0.0, current_cpu_ms - interval_resource_window_cpu_ms_);

        avg_process_cpu_percent =
            elapsed_wall_ms > 0.0 ? (100.0 * elapsed_cpu_ms / elapsed_wall_ms) : 0.0;
        effective_output_rate_hz =
            elapsed_wall_ms > 0.0 ? (1000.0 * static_cast<double>(interval_frame_count_) / elapsed_wall_ms) : 0.0;

        interval_resource_window_start_ = now;
        interval_resource_window_cpu_ms_ = current_cpu_ms;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
    std::deque<BufferedCloud> cloud_queue_;
    std::mutex cloud_mutex_;
    rclcpp::TimerBase::SharedPtr processing_timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_cloud_publisher_;
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr object_detection_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr object_detection_marker_publisher_;
    double processing_rate_;
    std::vector<double> crop_box_min_;
    std::vector<double> crop_box_max_;
    std::vector<double> voxel_leaf_size_;
    bool publish_processed_lidar_pc_;
    bool enable_ground_segmentation_;
    int min_cluster_points_;
    int max_cluster_points_;
    std::vector<double> min_cluster_size_;
    std::vector<double> max_cluster_size_;
    int input_queue_size_{3};
    double max_buffer_age_ms_{300.0};
    int processing_timer_fallback_period_ms_{100};
    int lidar_input_qos_depth_{10};
    int processed_cloud_qos_depth_{10};
    int detection_qos_depth_{10};
    int marker_qos_depth_{10};
    double ground_segmentation_distance_threshold_m_{0.33};
    int ground_segmentation_max_iterations_{1000};
    std::vector<double> ground_segmentation_axis_{0.0, 0.0, 1.0};
    double ground_segmentation_eps_angle_deg_{10.0};
    std::vector<double> clustering_voxel_leaf_size_{0.3, 0.3, 0.3};
    bool enable_range_adaptive_clustering_{true};
    std::vector<double> clustering_range_breakpoints_m_{10.0, 20.0};
    std::vector<double> clustering_tolerance_by_range_m_{0.7, 1.0, 1.4};
    double marker_min_dimension_m_{0.05};
    std::vector<double> marker_color_rgba_{0.0, 1.0, 0.0, 0.25};
    double marker_lifetime_sec_{0.0};
    std::string marker_namespace_{"lidar_detection_boxes"};
    Eigen::Vector4f crop_box_min_vec;
    Eigen::Vector4f crop_box_max_vec;
    Eigen::Vector4f voxel_leaf_size_vec;
    Eigen::Vector3f min_cluster_size_vec_;
    Eigen::Vector3f max_cluster_size_vec_;
    Eigen::Vector3f clustering_voxel_leaf_size_vec_{0.3f, 0.3f, 0.3f};
    Eigen::Vector3f ground_segmentation_axis_vec_{0.0f, 0.0f, 1.0f};
    int interval_frame_count_{0};
    double interval_sum_buffer_age_ms_{0.0};
    double interval_sum_conversion_ms_{0.0};
    double interval_sum_crop_box_ms_{0.0};
    double interval_sum_voxelization_ms_{0.0};
    double interval_sum_ground_segmentation_ms_{0.0};
    double interval_sum_clustering_ms_{0.0};
    double interval_sum_cluster_filtering_ms_{0.0};
    double interval_sum_bounding_box_ms_{0.0};
    double interval_sum_detection_conversion_ms_{0.0};
    double interval_sum_marker_conversion_ms_{0.0};
    double interval_sum_publish_ms_{0.0};
    double interval_sum_processed_cloud_publish_ms_{0.0};
    double interval_sum_detection_publish_ms_{0.0};
    double interval_sum_marker_publish_ms_{0.0};
    double interval_sum_frame_total_ms_{0.0};
    double interval_sum_rss_mb_{0.0};
    double interval_sum_peak_rss_mb_{0.0};
    double interval_sum_input_points_{0.0};
    double interval_sum_output_points_{0.0};
    double interval_sum_raw_clusters_{0.0};
    double interval_sum_filtered_clusters_{0.0};
    double interval_sum_output_detections_{0.0};
    std::chrono::steady_clock::time_point interval_resource_window_start_;
    double interval_resource_window_cpu_ms_{0.0};
    std::uint64_t total_received_frames_{0};
    std::uint64_t total_processed_frames_{0};
    std::uint64_t total_overwritten_frames_{0};
    int profiling_interval_frames_;
    bool csv_logging_{false};
    std::string csv_log_dir_{"csv_logs/lidar_processing"};
    std::string csv_log_file_path_;
    std::string dataset_sequence_{"unknown"};
    std::ofstream csv_log_stream_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarProcessing>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
