#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <pcl/search/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/angles.h>
#include <pcl/common/common.h>

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
        this->declare_parameter<bool>("publish_processed_lidar_pc", true);
        this->declare_parameter<bool>("enable_ground_segmentation", true);
        this->declare_parameter<double>("cluster_tolerance_m", 0.75);
        this->declare_parameter<int>("min_cluster_points", 5);
        this->declare_parameter<int>("max_cluster_points", 5000);
        this->declare_parameter<std::vector<double>>("min_cluster_size", {0.2, 0.2, 0.2});
        this->declare_parameter<std::vector<double>>("max_cluster_size", {15.0, 8.0, 5.0});
        this->declare_parameter<int>("profiling_interval_frames", 60);
        this->declare_parameter<bool>("enable_csv_logging", false);
        this->declare_parameter<std::string>("csv_log_dir", "csv_logs/lidar_processing");
        this->declare_parameter<std::string>("dataset_sequence", "unknown");

        this->get_parameter("processing_rate", processing_rate_);
        this->get_parameter("crop_box_min", crop_box_min_);
        this->get_parameter("crop_box_max", crop_box_max_);
        this->get_parameter("voxel_leaf_size", voxel_leaf_size_);
        this->get_parameter("publish_processed_lidar_pc", publish_processed_lidar_pc_);
        this->get_parameter("enable_ground_segmentation", enable_ground_segmentation_);
        this->get_parameter("cluster_tolerance_m", cluster_tolerance_m_);
        this->get_parameter("min_cluster_points", min_cluster_points_);
        this->get_parameter("max_cluster_points", max_cluster_points_);
        this->get_parameter("min_cluster_size", min_cluster_size_);
        this->get_parameter("max_cluster_size", max_cluster_size_);
        this->get_parameter("profiling_interval_frames", profiling_interval_frames_);
        this->get_parameter("enable_csv_logging", csv_logging_);
        this->get_parameter("csv_log_dir", csv_log_dir_);
        this->get_parameter("dataset_sequence", dataset_sequence_);
        
        crop_box_min_vec = Eigen::Vector4f(crop_box_min_[0], crop_box_min_[1], crop_box_min_[2], 1.0);
        crop_box_max_vec = Eigen::Vector4f(crop_box_max_[0], crop_box_max_[1], crop_box_max_[2], 1.0);
        voxel_leaf_size_vec = Eigen::Vector4f(voxel_leaf_size_[0], voxel_leaf_size_[1], voxel_leaf_size_[2], 1.0);
        min_cluster_size_vec_ = Eigen::Vector3f(min_cluster_size_[0], min_cluster_size_[1], min_cluster_size_[2]);
        max_cluster_size_vec_ = Eigen::Vector3f(max_cluster_size_[0], max_cluster_size_[1], max_cluster_size_[2]);

        processing_timer_ = this->create_wall_timer(
            processing_rate_ > 0 ? std::chrono::milliseconds(static_cast<int>(1000.0 / processing_rate_)) : std::chrono::milliseconds(100),
            std::bind(&LidarProcessing::process_latest_point_cloud, this));
        lidar_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "lidar_in",
            10,
            std::bind(&LidarProcessing::lidar_subscriber_callback, this, std::placeholders::_1));
        if (publish_processed_lidar_pc_)
        {
            processed_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "lidar_out",
                10);
        }
        object_detection_publisher_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(
            "lidar_detections",
            10);
        object_detection_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "lidar_detection_markers",
            10);

        initialize_csv_logging();
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

            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> raw_clusters;
            {
                ScopedTimer clustering_timer(metrics.clustering_time_ms);
                raw_clusters = euclideanClustering(output_cloud);
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

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> euclideanClustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
    {
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;

        if (!cloud || cloud->empty())
        {
            return clusters;
        }

        pcl::search::KdTree<pcl::PointXYZI>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZI>());
        search_tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> cluster_extractor;
        cluster_extractor.setInputCloud(cloud);
        cluster_extractor.setSearchMethod(search_tree);
        cluster_extractor.setClusterTolerance(cluster_tolerance_m_);
        cluster_extractor.setMinClusterSize(1);
        cluster_extractor.setMaxClusterSize(static_cast<int>(std::min<std::size_t>(
            cloud->points.size(),
            static_cast<std::size_t>(std::numeric_limits<int>::max()))));
        cluster_extractor.extract(cluster_indices);

        clusters.reserve(cluster_indices.size());
        for (const auto &indices : cluster_indices)
        {
            auto cluster_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            cluster_cloud->points.reserve(indices.indices.size());

            for (const int point_index : indices.indices)
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

    struct AxisAlignedClusterBounds
    {
        Eigen::Vector4f min_point{Eigen::Vector4f::Zero()};
        Eigen::Vector4f max_point{Eigen::Vector4f::Zero()};

        Eigen::Vector3f dimensions() const
        {
            return (max_point - min_point).head<3>();
        }
    };

    AxisAlignedClusterBounds computeAxisAlignedClusterBounds(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cluster) const
    {
        AxisAlignedClusterBounds bounds;
        if (!cluster || cluster->empty())
        {
            return bounds;
        }

        pcl::getMinMax3D(*cluster, bounds.min_point, bounds.max_point);
        return bounds;
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

    bool clusterPassesSizeFilter(const AxisAlignedClusterBounds &bounds) const
    {
        const Eigen::Vector3f cluster_dimensions = bounds.dimensions();
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

            const auto cluster_bounds = computeAxisAlignedClusterBounds(cluster);
            if (!clusterPassesSizeFilter(cluster_bounds))
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
    };

    std::vector<ClusterBox> compute3dBoundingBoxes(
        const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &clusters) const
    {
        std::vector<ClusterBox> cluster_boxes;
        cluster_boxes.reserve(clusters.size());

        for (const auto &cluster : clusters)
        {
            const auto cluster_bounds = computeAxisAlignedClusterBounds(cluster);
            ClusterBox box;
            box.center = ((cluster_bounds.min_point + cluster_bounds.max_point) * 0.5f).head<3>();
            box.size = cluster_bounds.dimensions();
            cluster_boxes.push_back(box);
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
            detection_msg.bbox.center.orientation.x = 0.0;
            detection_msg.bbox.center.orientation.y = 0.0;
            detection_msg.bbox.center.orientation.z = 0.0;
            detection_msg.bbox.center.orientation.w = 1.0;
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
            marker.ns = "lidar_detection_boxes";
            marker.id = static_cast<int>(cluster_index);
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = cluster_box.center.x();
            marker.pose.position.y = cluster_box.center.y();
            marker.pose.position.z = cluster_box.center.z();
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = std::max(static_cast<double>(cluster_box.size.x()), 0.05);
            marker.scale.y = std::max(static_cast<double>(cluster_box.size.y()), 0.05);
            marker.scale.z = std::max(static_cast<double>(cluster_box.size.z()), 0.05);
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.25f;
            marker.lifetime = rclcpp::Duration::from_seconds(0.0);
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
            const double avg_input_points = interval_sum_input_points_ / interval_frame_count_;
            const double avg_output_points = interval_sum_output_points_ / interval_frame_count_;
            const double avg_raw_clusters = interval_sum_raw_clusters_ / interval_frame_count_;
            const double avg_filtered_clusters = interval_sum_filtered_clusters_ / interval_frame_count_;
            const double avg_output_detections = interval_sum_output_detections_ / interval_frame_count_;

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

            csv_log_stream_ << "timestamp_utc,dataset_sequence,interval_frames,avg_buffer_age_ms,avg_conversion_time_ms,avg_crop_box_time_ms,avg_voxelization_time_ms,avg_ground_segmentation_time_ms,avg_clustering_time_ms,avg_cluster_filtering_time_ms,avg_bounding_box_time_ms,avg_detection_conversion_time_ms,avg_marker_conversion_time_ms,avg_publish_time_ms,avg_processed_cloud_publish_time_ms,avg_detection_publish_time_ms,avg_marker_publish_time_ms,avg_frame_total_time_ms,avg_input_points,avg_output_points,avg_raw_clusters,avg_filtered_clusters,avg_output_detections,total_received_frames,total_processed_frames,total_overwritten_frames\n";
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

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
    std::chrono::steady_clock::time_point latest_cloud_received_steady_;
    bool new_cloud_available_{false};
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
    double cluster_tolerance_m_;
    int min_cluster_points_;
    int max_cluster_points_;
    std::vector<double> min_cluster_size_;
    std::vector<double> max_cluster_size_;
    Eigen::Vector4f crop_box_min_vec;
    Eigen::Vector4f crop_box_max_vec;
    Eigen::Vector4f voxel_leaf_size_vec;
    Eigen::Vector3f min_cluster_size_vec_;
    Eigen::Vector3f max_cluster_size_vec_;
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
    double interval_sum_input_points_{0.0};
    double interval_sum_output_points_{0.0};
    double interval_sum_raw_clusters_{0.0};
    double interval_sum_filtered_clusters_{0.0};
    double interval_sum_output_detections_{0.0};
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
