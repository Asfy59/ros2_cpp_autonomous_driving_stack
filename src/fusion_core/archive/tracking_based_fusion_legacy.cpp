#include <rclcpp/rclcpp.hpp>

#include <auto_stack_msgs/msg/decision_state.hpp>
#include <auto_stack_msgs/msg/tracked_object_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <sys/resource.h>
#include <unistd.h>
#include <utility>
#include <vector>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/imgproc.hpp>

namespace
{
double read_process_cpu_time_ms()
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

double read_current_rss_mb()
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

double read_peak_rss_mb()
{
    rusage usage{};
    if (getrusage(RUSAGE_SELF, &usage) != 0)
    {
        return 0.0;
    }

    return static_cast<double>(usage.ru_maxrss) / 1024.0;
}
}  // namespace

/*
 * Tracking-based fusion model summary
 *
 * Internal EKF track state lives in a stable planar tracking frame
 * (configured as `tracking_frame`, typically `map`):
 *
 *   x = [px, py, vx, vy, yaw, length, width]^T
 *
 * where:
 *   px, py   -> object center position in meters
 *   vx, vy   -> planar velocity in meters / second
 *   yaw      -> object heading in radians in the planar frame
 *   length   -> object extent along the forward axis
 *   width    -> object extent along the lateral axis
 *
 * We use a constant-velocity motion model over timestep dt:
 *
 *   px' = px + vx * dt
 *   py' = py + vy * dt
 *   vx' = vx
 *   vy' = vy
 *   yaw' = yaw
 *   length' = length
 *   width' = width
 *
 * which gives the linear prediction:
 *
 *   x_k^- = F(dt) * x_{k-1} + w_k
 *   P_k^- = F(dt) * P_{k-1} * F(dt)^T + Q
 *
 * with process noise w_k ~ N(0, Q).
 *
 * LiDAR provides a linear measurement in metric space:
 *
 *   z_lidar = [px, py, yaw, length, width]^T
 *   z_lidar = H_lidar * x + v_lidar
 *
 * Camera provides a nonlinear image-space measurement:
 *
 *   z_camera = [u, v, bbox_width_px, bbox_height_px]^T
 *   z_camera = h_camera(x) + v_camera
 *
 * where h_camera(x) projects the object hypothesis into the image using
 * camera intrinsics and the tracking-frame-to-camera transform.
 *
 * We intentionally keep center_z and height outside the EKF state for now.
 * They remain track metadata updated mainly from LiDAR, while the EKF focuses
 * on planar motion and footprint estimation.
 */

class TrackingBasedFusion final : public rclcpp::Node
{
public:
    static constexpr std::size_t kSensorImageQosDepth{5};
    static constexpr std::size_t kSemanticQosDepth{5};

    TrackingBasedFusion()
        : rclcpp::Node("tracking_based_fusion"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        const auto lidar_detections_topic =
            this->declare_parameter<std::string>("lidar_detections_topic", "lidar_detections");
        const auto camera_detections_topic =
            this->declare_parameter<std::string>("camera_detections_topic", "object_detections");
        const auto camera_stereo_detections_topic =
            this->declare_parameter<std::string>("camera_stereo_detections_topic", "camera_stereo_detections");
        const auto camera_info_topic =
            this->declare_parameter<std::string>("camera_info_topic", "p2_camera_info");
        const auto camera_image_topic =
            this->declare_parameter<std::string>("camera_image_topic", "/p2_img");
        const auto tracked_objects_topic =
            this->declare_parameter<std::string>("tracked_objects_topic", "tracked_objects");
        const auto decision_state_topic =
            this->declare_parameter<std::string>("decision_state_topic", "decision_state");
        tracking_frame_ =
            this->declare_parameter<std::string>("tracking_frame", "map");
        const auto fusion_overlay_topic =
            this->declare_parameter<std::string>("fusion_overlay_topic", "fusion_overlay_image");
        const auto tracked_object_markers_topic =
            this->declare_parameter<std::string>("tracked_object_markers_topic", "tracked_object_markers");
        tracked_marker_lifetime_sec_ =
            this->declare_parameter<double>("tracked_marker_lifetime_sec", 1.0);
        camera_sync_tolerance_ms_ =
            this->declare_parameter<double>("camera_sync_tolerance_ms", 200.0);
        match_iou_threshold_ =
            this->declare_parameter<double>("match_iou_threshold", 0.10);
        max_match_center_distance_px_ =
            this->declare_parameter<double>("max_match_center_distance_px", 160.0);
        tf_lookup_timeout_ms_ =
            this->declare_parameter<double>("tf_lookup_timeout_ms", 100.0);
        min_projection_depth_m_ =
            this->declare_parameter<double>("min_projection_depth_m", 0.10);
        (void)this->declare_parameter<double>("stop_distance_m", 6.0);
        (void)this->declare_parameter<double>("slow_distance_m", 12.0);
        (void)this->declare_parameter<double>("decision_lateral_gate_m", 2.5);
        camera_history_size_ =
            std::max(1, static_cast<int>(this->declare_parameter<int>("camera_history_size", 10)));
        max_active_tracks_ =
            std::max(1, static_cast<int>(this->declare_parameter<int>("max_active_tracks", 128)));
        profiling_interval_frames_ =
            std::max(1, static_cast<int>(this->declare_parameter<int>("profiling_interval_frames", 60)));
        publish_overlay_image_ =
            this->declare_parameter<bool>("publish_overlay_image", true);
        csv_logging_ =
            this->declare_parameter<bool>("enable_csv_logging", false);
        csv_log_dir_ =
            this->declare_parameter<std::string>("csv_log_dir", "csv_logs/fusion_core");
        dataset_sequence_ =
            this->declare_parameter<std::string>("dataset_sequence", "unknown");
        lidar_association_distance_gate_m_ =
            this->declare_parameter<double>("lidar_association_distance_gate_m", 2.0);
        camera_stereo_association_distance_gate_m_ =
            this->declare_parameter<double>("camera_stereo_association_distance_gate_m", 3.0);
        initial_existence_probability_ =
            this->declare_parameter<double>("initial_existence_probability", 0.55);
        existence_probability_hit_gain_ =
            this->declare_parameter<double>("existence_probability_hit_gain", 0.20);
        existence_probability_miss_decay_ =
            this->declare_parameter<double>("existence_probability_miss_decay", 0.15);
        track_confirmation_threshold_ =
            this->declare_parameter<double>("track_confirmation_threshold", 0.75);
        track_deletion_threshold_ =
            this->declare_parameter<double>("track_deletion_threshold", 0.20);
        process_noise_position_variance_per_s2_ =
            this->declare_parameter<double>("process_noise_position_variance_per_s2", 0.25);
        process_noise_velocity_variance_per_s_ =
            this->declare_parameter<double>("process_noise_velocity_variance_per_s", 1.0);
        process_noise_yaw_variance_per_s_ =
            this->declare_parameter<double>("process_noise_yaw_variance_per_s", 0.04);
        process_noise_size_variance_per_s_ =
            this->declare_parameter<double>("process_noise_size_variance_per_s", 0.05);
        lidar_measurement_noise_position_variance_ =
            this->declare_parameter<double>("lidar_measurement_noise_position_variance", 0.25);
        lidar_measurement_noise_yaw_variance_ =
            this->declare_parameter<double>("lidar_measurement_noise_yaw_variance", 0.09);
        lidar_measurement_noise_size_variance_ =
            this->declare_parameter<double>("lidar_measurement_noise_size_variance", 0.5);

        lidar_association_distance_gate_m_ = std::max(0.1, lidar_association_distance_gate_m_);
        camera_stereo_association_distance_gate_m_ =
            std::max(0.1, camera_stereo_association_distance_gate_m_);
        initial_existence_probability_ = std::clamp(initial_existence_probability_, 0.0, 1.0);
        existence_probability_hit_gain_ = std::clamp(existence_probability_hit_gain_, 0.0, 1.0);
        existence_probability_miss_decay_ = std::clamp(existence_probability_miss_decay_, 0.0, 1.0);
        track_confirmation_threshold_ = std::clamp(track_confirmation_threshold_, 0.0, 1.0);
        track_deletion_threshold_ = std::clamp(track_deletion_threshold_, 0.0, track_confirmation_threshold_);
        tracked_marker_lifetime_sec_ = std::max(0.0, tracked_marker_lifetime_sec_);
        camera_sync_tolerance_ms_ = std::max(0.0, camera_sync_tolerance_ms_);
        match_iou_threshold_ = std::clamp(match_iou_threshold_, 0.0, 1.0);
        max_match_center_distance_px_ = std::max(1.0, max_match_center_distance_px_);
        tf_lookup_timeout_ms_ = std::max(1.0, tf_lookup_timeout_ms_);
        min_projection_depth_m_ = std::max(1e-3, min_projection_depth_m_);
        if (tracking_frame_.empty())
        {
            tracking_frame_ = "map";
        }
        if (dataset_sequence_.empty())
        {
            dataset_sequence_ = "unknown";
        }
        process_noise_position_variance_per_s2_ = std::max(0.0, process_noise_position_variance_per_s2_);
        process_noise_velocity_variance_per_s_ = std::max(0.0, process_noise_velocity_variance_per_s_);
        process_noise_yaw_variance_per_s_ = std::max(0.0, process_noise_yaw_variance_per_s_);
        process_noise_size_variance_per_s_ = std::max(0.0, process_noise_size_variance_per_s_);
        lidar_measurement_noise_position_variance_ = std::max(1e-6, lidar_measurement_noise_position_variance_);
        lidar_measurement_noise_yaw_variance_ = std::max(1e-6, lidar_measurement_noise_yaw_variance_);
        lidar_measurement_noise_size_variance_ = std::max(1e-6, lidar_measurement_noise_size_variance_);

        const auto semantic_qos =
            rclcpp::QoS(rclcpp::KeepLast(kSemanticQosDepth))
                .reliable()
                .durability_volatile();
        const auto camera_image_qos =
            rclcpp::SensorDataQoS().keep_last(kSensorImageQosDepth);

        lidar_detections_subscription_ =
            this->create_subscription<vision_msgs::msg::Detection3DArray>(
                lidar_detections_topic,
                semantic_qos,
                std::bind(&TrackingBasedFusion::lidar_detections_callback, this, std::placeholders::_1));

        camera_detections_subscription_ =
            this->create_subscription<vision_msgs::msg::Detection2DArray>(
                camera_detections_topic,
                semantic_qos,
                std::bind(&TrackingBasedFusion::camera_detections_callback, this, std::placeholders::_1));
        camera_stereo_detections_subscription_ =
            this->create_subscription<vision_msgs::msg::Detection3DArray>(
                camera_stereo_detections_topic,
                semantic_qos,
                std::bind(&TrackingBasedFusion::camera_stereo_detections_callback, this, std::placeholders::_1));

        camera_info_subscription_ =
            this->create_subscription<sensor_msgs::msg::CameraInfo>(
                camera_info_topic,
                semantic_qos,
                std::bind(&TrackingBasedFusion::camera_info_callback, this, std::placeholders::_1));

        camera_image_subscription_ =
            this->create_subscription<sensor_msgs::msg::Image>(
                camera_image_topic,
                camera_image_qos,
                std::bind(&TrackingBasedFusion::camera_image_callback, this, std::placeholders::_1));

        tracked_objects_publisher_ =
            this->create_publisher<auto_stack_msgs::msg::TrackedObjectArray>(tracked_objects_topic, semantic_qos);
        decision_state_publisher_ =
            this->create_publisher<auto_stack_msgs::msg::DecisionState>(decision_state_topic, semantic_qos);
        tracked_object_markers_publisher_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>(tracked_object_markers_topic, semantic_qos);
        if (publish_overlay_image_)
        {
            fusion_overlay_publisher_ =
                this->create_publisher<sensor_msgs::msg::Image>(fusion_overlay_topic, semantic_qos);
        }

        initialize_csv_logging();
        reset_resource_window();

        RCLCPP_INFO(
            this->get_logger(),
            "TrackingBasedFusion initialized with yaw-aware LiDAR tracking and camera validation.");
    }

private:
    static constexpr std::size_t kTrackStateDim{7};
    static constexpr std::size_t kLidarMeasurementDim{5};
    static constexpr std::size_t kCameraMeasurementDim{4};
    static constexpr std::size_t kPosXIndex{0};
    static constexpr std::size_t kPosYIndex{1};
    static constexpr std::size_t kVelXIndex{2};
    static constexpr std::size_t kVelYIndex{3};
    static constexpr std::size_t kYawIndex{4};
    static constexpr std::size_t kLengthIndex{5};
    static constexpr std::size_t kWidthIndex{6};
    static constexpr std::size_t kLidarPosXMeasurementIndex{0};
    static constexpr std::size_t kLidarPosYMeasurementIndex{1};
    static constexpr std::size_t kLidarYawMeasurementIndex{2};
    static constexpr std::size_t kLidarLengthMeasurementIndex{3};
    static constexpr std::size_t kLidarWidthMeasurementIndex{4};

    using TrackStateVector = std::array<double, kTrackStateDim>;
    using StateCovarianceMatrix = std::array<double, kTrackStateDim * kTrackStateDim>;
    using TrackStateMatrix = StateCovarianceMatrix;
    using LidarMeasurementVector = std::array<double, kLidarMeasurementDim>;
    using LidarMeasurementCovarianceMatrix =
        std::array<double, kLidarMeasurementDim * kLidarMeasurementDim>;

    struct LidarMeasurement
    {
        std::size_t detection_index{0};
        builtin_interfaces::msg::Time stamp;
        double center_x_m{0.0};
        double center_y_m{0.0};
        double center_z_m{0.0};
        double yaw_rad{0.0};
        double length_m{0.0};
        double width_m{0.0};
        double height_m{0.0};
    };

    struct CameraStereoMeasurement
    {
        std::size_t detection_index{0};
        builtin_interfaces::msg::Time stamp;
        double center_x_m{0.0};
        double center_y_m{0.0};
        double center_z_m{0.0};
        double length_m{0.0};
        double width_m{0.0};
        double height_m{0.0};
        std::string classification{"unknown"};
        double confidence{0.0};
    };

    struct FusedTrack
    {
        std::uint32_t track_id{0};
        TrackStateVector track_state{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        StateCovarianceMatrix state_covariance{{0.0}};
        builtin_interfaces::msg::Time last_predict_stamp;
        builtin_interfaces::msg::Time last_update_stamp;
        std::size_t age_in_updates{0};
        bool is_confirmed{false};

        double center_z_m{0.0};
        double height_m{0.0};
        std::string classification{"unknown"};
        float existence_probability{0.0F};

        std::optional<std::size_t> associated_lidar_detection_index;
        std::optional<std::size_t> associated_camera_detection_index;
        bool camera_supported_this_frame{false};
        bool stereo_supported_this_frame{false};
    };

    struct LidarAssociationMatch
    {
        std::size_t track_index{0};
        std::size_t measurement_index{0};
        double planar_distance_m{0.0};
    };

    struct LidarAssociationResult
    {
        std::vector<LidarAssociationMatch> matches;
        std::vector<std::size_t> unmatched_track_indices;
        std::vector<std::size_t> unmatched_measurement_indices;
    };

    struct TrackerFrameMetrics
    {
        std::size_t input_lidar_detections{0};
        std::size_t tracks_before_prediction{0};
        std::size_t matched_pairs{0};
        std::size_t unmatched_tracks{0};
        std::size_t unmatched_measurements{0};
        std::size_t new_tracks_created{0};
        std::size_t deleted_tracks{0};
        std::size_t confirmed_tracks_after_update{0};
        std::size_t camera_supported_tracks{0};
        double average_match_distance_m{0.0};
        double max_match_distance_m{0.0};
        double camera_lidar_skew_ms{0.0};
        double processing_time_ms{0.0};
    };

    struct ImageRoi
    {
        double min_x{0.0};
        double min_y{0.0};
        double max_x{0.0};
        double max_y{0.0};

        [[nodiscard]] double width() const
        {
            return std::max(0.0, max_x - min_x);
        }

        [[nodiscard]] double height() const
        {
            return std::max(0.0, max_y - min_y);
        }

        [[nodiscard]] double center_x() const
        {
            return 0.5 * (min_x + max_x);
        }

        [[nodiscard]] double center_y() const
        {
            return 0.5 * (min_y + max_y);
        }
    };

    struct CameraMatch
    {
        std::size_t detection_index{0};
        double iou{0.0};
        double score{0.0};
        std::string class_id{"unknown"};
    };

    struct CameraFrameSelection
    {
        std::shared_ptr<vision_msgs::msg::Detection2DArray> detections;
        double skew_ms{0.0};
    };

    struct CameraStereoFrameSelection
    {
        std::shared_ptr<vision_msgs::msg::Detection3DArray> detections;
        double skew_ms{0.0};
    };

    static double normalize_angle_rad(double angle_rad)
    {
        if (!std::isfinite(angle_rad))
        {
            return 0.0;
        }
        constexpr double kPi = 3.14159265358979323846;
        return std::remainder(angle_rad, 2.0 * kPi);
    }

    static double yaw_from_quaternion(
        const geometry_msgs::msg::Quaternion &orientation_msg)
    {
        const double siny_cosp =
            2.0 * ((orientation_msg.w * orientation_msg.z) + (orientation_msg.x * orientation_msg.y));
        const double cosy_cosp =
            1.0 - (2.0 * ((orientation_msg.y * orientation_msg.y) + (orientation_msg.z * orientation_msg.z)));
        return std::atan2(siny_cosp, cosy_cosp);
    }

    static geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw_rad)
    {
        geometry_msgs::msg::Quaternion orientation_msg;
        orientation_msg.x = 0.0;
        orientation_msg.y = 0.0;
        orientation_msg.z = std::sin(0.5 * yaw_rad);
        orientation_msg.w = std::cos(0.5 * yaw_rad);
        return orientation_msg;
    }

    static geometry_msgs::msg::Quaternion to_geometry_quaternion(const tf2::Quaternion &quaternion)
    {
        geometry_msgs::msg::Quaternion orientation_msg;
        orientation_msg.x = quaternion.x();
        orientation_msg.y = quaternion.y();
        orientation_msg.z = quaternion.z();
        orientation_msg.w = quaternion.w();
        return orientation_msg;
    }

    std::optional<tf2::Transform> lookup_transform(
        const std::string &target_frame,
        const std::string &source_frame,
        const builtin_interfaces::msg::Time &stamp)
    {
        if (target_frame == source_frame)
        {
            return tf2::Transform::getIdentity();
        }

        try
        {
            const auto transform_stamped = tf_buffer_.lookupTransform(
                target_frame,
                source_frame,
                stamp,
                rclcpp::Duration::from_seconds(tf_lookup_timeout_ms_ / 1000.0));
            tf2::Transform transform;
            tf2::fromMsg(transform_stamped.transform, transform);
            return transform;
        }
        catch (const tf2::TransformException &exception)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "TF lookup failed from %s to %s: %s",
                source_frame.c_str(),
                target_frame.c_str(),
                exception.what());
            return std::nullopt;
        }
    }

    FusedTrack initialize_track_from_lidar_measurement(const LidarMeasurement &lidar_measurement)
    {
        FusedTrack new_track;
        new_track.track_id = next_track_id_++;
        new_track.track_state = {
            lidar_measurement.center_x_m,
            lidar_measurement.center_y_m,
            0.0,
            0.0,
            lidar_measurement.yaw_rad,
            lidar_measurement.length_m,
            lidar_measurement.width_m};
        new_track.state_covariance = {
            lidar_measurement_noise_position_variance_, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, lidar_measurement_noise_position_variance_, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, lidar_measurement_noise_yaw_variance_, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, lidar_measurement_noise_size_variance_, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, lidar_measurement_noise_size_variance_};
        new_track.last_predict_stamp = lidar_measurement.stamp;
        new_track.last_update_stamp = lidar_measurement.stamp;
        new_track.age_in_updates = 1;
        new_track.is_confirmed = initial_existence_probability_ >= track_confirmation_threshold_;
        new_track.center_z_m = lidar_measurement.center_z_m;
        new_track.height_m = lidar_measurement.height_m;
        new_track.classification = "unknown";
        new_track.existence_probability = static_cast<float>(initial_existence_probability_);
        new_track.associated_lidar_detection_index = lidar_measurement.detection_index;
        return new_track;
    }

    std::vector<LidarMeasurement> make_lidar_measurements(
        const vision_msgs::msg::Detection3DArray &lidar_detections,
        const tf2::Transform &lidar_to_tracking_tf) const
    {
        std::vector<LidarMeasurement> lidar_measurements;
        lidar_measurements.reserve(lidar_detections.detections.size());

        for (std::size_t detection_index = 0; detection_index < lidar_detections.detections.size(); ++detection_index)
        {
            const auto &detection = lidar_detections.detections[detection_index];

            LidarMeasurement measurement;
            measurement.detection_index = detection_index;
            measurement.stamp = lidar_detections.header.stamp;
            const tf2::Vector3 detection_center(
                detection.bbox.center.position.x,
                detection.bbox.center.position.y,
                detection.bbox.center.position.z);
            const tf2::Vector3 transformed_center = lidar_to_tracking_tf * detection_center;
            measurement.center_x_m = transformed_center.x();
            measurement.center_y_m = transformed_center.y();
            measurement.center_z_m = transformed_center.z();

            tf2::Quaternion detection_orientation;
            tf2::fromMsg(detection.bbox.center.orientation, detection_orientation);
            const tf2::Quaternion transformed_orientation =
                lidar_to_tracking_tf.getRotation() * detection_orientation;
            measurement.yaw_rad = yaw_from_quaternion(to_geometry_quaternion(transformed_orientation));
            measurement.length_m = detection.bbox.size.x;
            measurement.width_m = detection.bbox.size.y;
            measurement.height_m = detection.bbox.size.z;

            lidar_measurements.push_back(measurement);
        }

        return lidar_measurements;
    }

    std::vector<CameraStereoMeasurement> make_camera_stereo_measurements(
        const vision_msgs::msg::Detection3DArray &camera_stereo_detections,
        const tf2::Transform &camera_to_tracking_tf) const
    {
        std::vector<CameraStereoMeasurement> camera_stereo_measurements;
        camera_stereo_measurements.reserve(camera_stereo_detections.detections.size());

        for (std::size_t detection_index = 0;
             detection_index < camera_stereo_detections.detections.size();
             ++detection_index)
        {
            const auto &detection = camera_stereo_detections.detections[detection_index];

            CameraStereoMeasurement measurement;
            measurement.detection_index = detection_index;
            measurement.stamp = camera_stereo_detections.header.stamp;
            const tf2::Vector3 detection_center(
                detection.bbox.center.position.x,
                detection.bbox.center.position.y,
                detection.bbox.center.position.z);
            const tf2::Vector3 transformed_center = camera_to_tracking_tf * detection_center;
            measurement.center_x_m = transformed_center.x();
            measurement.center_y_m = transformed_center.y();
            measurement.center_z_m = transformed_center.z();
            measurement.length_m = detection.bbox.size.x;
            measurement.width_m = detection.bbox.size.y;
            measurement.height_m = detection.bbox.size.z;

            if (!detection.results.empty())
            {
                const auto &hypothesis = detection.results.front().hypothesis;
                measurement.classification = class_id_to_label(hypothesis.class_id);
                measurement.confidence = hypothesis.score;
            }

            camera_stereo_measurements.push_back(measurement);
        }

        return camera_stereo_measurements;
    }

    static double time_delta_ms(
        const builtin_interfaces::msg::Time &lhs,
        const builtin_interfaces::msg::Time &rhs)
    {
        return std::abs((rclcpp::Time(lhs) - rclcpp::Time(rhs)).seconds()) * 1000.0;
    }

    static std::string class_id_to_label(const std::string &class_id)
    {
        if (class_id == "0")
        {
            return "person";
        }
        if (class_id == "1")
        {
            return "bicycle";
        }
        if (class_id == "2")
        {
            return "car";
        }
        if (class_id == "3")
        {
            return "motorcycle";
        }
        if (class_id == "5")
        {
            return "bus";
        }
        if (class_id == "7")
        {
            return "truck";
        }
        return "class_" + class_id;
    }

    static ImageRoi detection_2d_to_roi(const vision_msgs::msg::Detection2D &detection)
    {
        ImageRoi roi;
        roi.min_x = detection.bbox.center.position.x - (0.5 * detection.bbox.size_x);
        roi.max_x = detection.bbox.center.position.x + (0.5 * detection.bbox.size_x);
        roi.min_y = detection.bbox.center.position.y - (0.5 * detection.bbox.size_y);
        roi.max_y = detection.bbox.center.position.y + (0.5 * detection.bbox.size_y);
        return roi;
    }

    static void draw_roi(
        cv::Mat &image,
        const ImageRoi &roi,
        const cv::Scalar &color,
        const std::string &label)
    {
        const cv::Point top_left(
            static_cast<int>(std::round(roi.min_x)),
            static_cast<int>(std::round(roi.min_y)));
        const cv::Point bottom_right(
            static_cast<int>(std::round(roi.max_x)),
            static_cast<int>(std::round(roi.max_y)));

        cv::rectangle(image, top_left, bottom_right, color, 2);
        if (label.empty())
        {
            return;
        }

        int baseline = 0;
        const cv::Size label_size =
            cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        const int label_top = std::max(top_left.y, label_size.height);
        cv::rectangle(
            image,
            cv::Point(top_left.x, label_top - label_size.height),
            cv::Point(top_left.x + label_size.width, label_top + baseline),
            color,
            cv::FILLED);
        cv::putText(
            image,
            label,
            cv::Point(top_left.x, label_top),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            cv::Scalar(0, 0, 0),
            1);
    }

    static double compute_iou(const ImageRoi &lhs, const ImageRoi &rhs)
    {
        const double intersect_min_x = std::max(lhs.min_x, rhs.min_x);
        const double intersect_min_y = std::max(lhs.min_y, rhs.min_y);
        const double intersect_max_x = std::min(lhs.max_x, rhs.max_x);
        const double intersect_max_y = std::min(lhs.max_y, rhs.max_y);
        const double intersect_width = std::max(0.0, intersect_max_x - intersect_min_x);
        const double intersect_height = std::max(0.0, intersect_max_y - intersect_min_y);
        const double intersection_area = intersect_width * intersect_height;
        const double union_area = (lhs.width() * lhs.height()) + (rhs.width() * rhs.height()) - intersection_area;

        if (union_area <= 0.0)
        {
            return 0.0;
        }

        return intersection_area / union_area;
    }

    static double compute_center_distance_px(const ImageRoi &lhs, const ImageRoi &rhs)
    {
        const double delta_x = lhs.center_x() - rhs.center_x();
        const double delta_y = lhs.center_y() - rhs.center_y();
        return std::sqrt((delta_x * delta_x) + (delta_y * delta_y));
    }

    CameraFrameSelection find_nearest_camera_detections(
        const builtin_interfaces::msg::Time &target_stamp)
    {
        CameraFrameSelection selection;
        std::lock_guard<std::mutex> lock(camera_mutex_);
        if (camera_detections_buffer_.empty())
        {
            return selection;
        }

        auto best_match = camera_detections_buffer_.front();
        double best_delta_ms = time_delta_ms(best_match->header.stamp, target_stamp);

        for (const auto &candidate : camera_detections_buffer_)
        {
            const double candidate_delta_ms = time_delta_ms(candidate->header.stamp, target_stamp);
            if (candidate_delta_ms < best_delta_ms)
            {
                best_delta_ms = candidate_delta_ms;
                best_match = candidate;
            }
        }

        if (best_delta_ms > camera_sync_tolerance_ms_)
        {
            return selection;
        }

        selection.detections = best_match;
        selection.skew_ms = best_delta_ms;
        return selection;
    }

    CameraStereoFrameSelection find_nearest_camera_stereo_detections(
        const builtin_interfaces::msg::Time &target_stamp)
    {
        CameraStereoFrameSelection selection;
        std::lock_guard<std::mutex> lock(camera_mutex_);
        if (camera_stereo_detections_buffer_.empty())
        {
            return selection;
        }

        auto best_match = camera_stereo_detections_buffer_.front();
        double best_delta_ms = time_delta_ms(best_match->header.stamp, target_stamp);

        for (const auto &candidate : camera_stereo_detections_buffer_)
        {
            const double candidate_delta_ms = time_delta_ms(candidate->header.stamp, target_stamp);
            if (candidate_delta_ms < best_delta_ms)
            {
                best_delta_ms = candidate_delta_ms;
                best_match = candidate;
            }
        }

        if (best_delta_ms > camera_sync_tolerance_ms_)
        {
            return selection;
        }

        selection.detections = best_match;
        selection.skew_ms = best_delta_ms;
        return selection;
    }

    std::shared_ptr<sensor_msgs::msg::CameraInfo> get_latest_camera_info()
    {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        return latest_camera_info_;
    }

    struct ImageFrameSelection
    {
        sensor_msgs::msg::Image::SharedPtr image;
        double skew_ms{0.0};
    };

    ImageFrameSelection find_nearest_camera_image(
        const builtin_interfaces::msg::Time &target_stamp)
    {
        ImageFrameSelection selection;
        std::lock_guard<std::mutex> lock(camera_mutex_);
        if (camera_image_buffer_.empty())
        {
            return selection;
        }

        auto best_match = camera_image_buffer_.front();
        double best_delta_ms = time_delta_ms(best_match->header.stamp, target_stamp);

        for (const auto &candidate : camera_image_buffer_)
        {
            const double candidate_delta_ms = time_delta_ms(candidate->header.stamp, target_stamp);
            if (candidate_delta_ms < best_delta_ms)
            {
                best_delta_ms = candidate_delta_ms;
                best_match = candidate;
            }
        }

        if (best_delta_ms > camera_sync_tolerance_ms_)
        {
            return selection;
        }

        selection.image = best_match;
        selection.skew_ms = best_delta_ms;
        return selection;
    }

    static std::array<tf2::Vector3, 8> build_track_box_corners(const FusedTrack &track)
    {
        const double half_length = std::max(0.05, 0.5 * track.track_state[kLengthIndex]);
        const double half_width = std::max(0.05, 0.5 * track.track_state[kWidthIndex]);
        const double half_height = std::max(0.05, 0.5 * track.height_m);
        const double center_x = track.track_state[kPosXIndex];
        const double center_y = track.track_state[kPosYIndex];
        const double center_z = track.center_z_m;
        const double cos_yaw = std::cos(track.track_state[kYawIndex]);
        const double sin_yaw = std::sin(track.track_state[kYawIndex]);

        const std::array<tf2::Vector3, 8> local_corners{{
            tf2::Vector3(-half_length, -half_width, -half_height),
            tf2::Vector3(-half_length, -half_width, half_height),
            tf2::Vector3(-half_length, half_width, -half_height),
            tf2::Vector3(-half_length, half_width, half_height),
            tf2::Vector3(half_length, -half_width, -half_height),
            tf2::Vector3(half_length, -half_width, half_height),
            tf2::Vector3(half_length, half_width, -half_height),
            tf2::Vector3(half_length, half_width, half_height)}};

        std::array<tf2::Vector3, 8> world_corners{};
        for (std::size_t corner_index = 0; corner_index < local_corners.size(); ++corner_index)
        {
            const double local_x = local_corners[corner_index].x();
            const double local_y = local_corners[corner_index].y();
            const double rotated_x = (cos_yaw * local_x) - (sin_yaw * local_y);
            const double rotated_y = (sin_yaw * local_x) + (cos_yaw * local_y);
            world_corners[corner_index] = tf2::Vector3(
                center_x + rotated_x,
                center_y + rotated_y,
                center_z + local_corners[corner_index].z());
        }

        return world_corners;
    }

    static std::optional<std::pair<double, double>> project_camera_point(
        const tf2::Vector3 &camera_point,
        const sensor_msgs::msg::CameraInfo &camera_info)
    {
        const auto &p = camera_info.p;
        const double px =
            (p[0] * camera_point.x()) + (p[1] * camera_point.y()) + (p[2] * camera_point.z()) + p[3];
        const double py =
            (p[4] * camera_point.x()) + (p[5] * camera_point.y()) + (p[6] * camera_point.z()) + p[7];
        const double pz =
            (p[8] * camera_point.x()) + (p[9] * camera_point.y()) + (p[10] * camera_point.z()) + p[11];

        if (std::abs(pz) < 1e-6)
        {
            return std::nullopt;
        }

        return std::make_pair(px / pz, py / pz);
    }

    std::optional<ImageRoi> project_track_box_to_image(
        const FusedTrack &track,
        const tf2::Transform &track_to_camera_tf,
        const sensor_msgs::msg::CameraInfo &camera_info) const
    {
        if (camera_info.header.frame_id.empty() || camera_info.width == 0U || camera_info.height == 0U)
        {
            return std::nullopt;
        }

        const auto corners = build_track_box_corners(track);
        double min_u = std::numeric_limits<double>::max();
        double min_v = std::numeric_limits<double>::max();
        double max_u = std::numeric_limits<double>::lowest();
        double max_v = std::numeric_limits<double>::lowest();
        std::size_t valid_corner_count = 0;

        for (const auto &corner : corners)
        {
            const auto camera_point = track_to_camera_tf * corner;
            if (camera_point.z() <= min_projection_depth_m_)
            {
                continue;
            }

            const auto projected = project_camera_point(camera_point, camera_info);
            if (!projected)
            {
                continue;
            }

            min_u = std::min(min_u, projected->first);
            min_v = std::min(min_v, projected->second);
            max_u = std::max(max_u, projected->first);
            max_v = std::max(max_v, projected->second);
            ++valid_corner_count;
        }

        if (valid_corner_count < 2U)
        {
            return std::nullopt;
        }

        const double image_max_x = static_cast<double>(camera_info.width - 1U);
        const double image_max_y = static_cast<double>(camera_info.height - 1U);
        ImageRoi roi;
        roi.min_x = std::clamp(min_u, 0.0, image_max_x);
        roi.min_y = std::clamp(min_v, 0.0, image_max_y);
        roi.max_x = std::clamp(max_u, 0.0, image_max_x);
        roi.max_y = std::clamp(max_v, 0.0, image_max_y);

        if (roi.width() < 1.0 || roi.height() < 1.0)
        {
            return std::nullopt;
        }

        return roi;
    }

    std::optional<CameraMatch> find_best_camera_match(
        const ImageRoi &projected_roi,
        const vision_msgs::msg::Detection2DArray &camera_detections,
        const std::vector<bool> &camera_detection_used) const
    {
        std::optional<CameraMatch> best_match;
        double best_combined_score = std::numeric_limits<double>::lowest();

        for (std::size_t detection_index = 0; detection_index < camera_detections.detections.size(); ++detection_index)
        {
            if (detection_index < camera_detection_used.size() && camera_detection_used[detection_index])
            {
                continue;
            }

            const auto &camera_detection = camera_detections.detections[detection_index];
            if (camera_detection.results.empty())
            {
                continue;
            }

            const auto camera_roi = detection_2d_to_roi(camera_detection);
            const double iou = compute_iou(projected_roi, camera_roi);
            if (iou < match_iou_threshold_)
            {
                continue;
            }

            const double center_distance_px = compute_center_distance_px(projected_roi, camera_roi);
            if (center_distance_px > max_match_center_distance_px_)
            {
                continue;
            }

            const auto &hypothesis = camera_detection.results.front().hypothesis;
            const double combined_score = (0.7 * iou) + (0.3 * hypothesis.score);
            if (combined_score <= best_combined_score)
            {
                continue;
            }

            best_combined_score = combined_score;
            best_match = CameraMatch{
                detection_index,
                iou,
                hypothesis.score,
                hypothesis.class_id};
        }

        return best_match;
    }

    double compute_delta_time_seconds(
        const builtin_interfaces::msg::Time &previous_stamp,
        const builtin_interfaces::msg::Time &current_stamp) const
    {
        const rclcpp::Time previous_time(previous_stamp);
        const rclcpp::Time current_time(current_stamp);
        const rclcpp::Duration delta_time = current_time - previous_time;
        return delta_time.seconds();
    }

    static bool is_track_state_finite(const FusedTrack &track)
    {
        for (const double value : track.track_state)
        {
            if (!std::isfinite(value))
            {
                return false;
            }
        }
        for (const double value : track.state_covariance)
        {
            if (!std::isfinite(value))
            {
                return false;
            }
        }
        return std::isfinite(track.center_z_m) &&
               std::isfinite(track.height_m) &&
               std::isfinite(static_cast<double>(track.existence_probability));
    }

    double compute_planar_distance_m(
        const FusedTrack &track,
        const LidarMeasurement &lidar_measurement) const
    {
        if (!is_track_state_finite(track) ||
            !std::isfinite(lidar_measurement.center_x_m) ||
            !std::isfinite(lidar_measurement.center_y_m))
        {
            return std::numeric_limits<double>::infinity();
        }
        const double delta_x_m = track.track_state[kPosXIndex] - lidar_measurement.center_x_m;
        const double delta_y_m = track.track_state[kPosYIndex] - lidar_measurement.center_y_m;
        return std::sqrt((delta_x_m * delta_x_m) + (delta_y_m * delta_y_m));
    }

    double compute_planar_distance_m(
        const FusedTrack &track,
        const CameraStereoMeasurement &camera_stereo_measurement) const
    {
        if (!is_track_state_finite(track) ||
            !std::isfinite(camera_stereo_measurement.center_x_m) ||
            !std::isfinite(camera_stereo_measurement.center_y_m))
        {
            return std::numeric_limits<double>::infinity();
        }
        const double delta_x_m = track.track_state[kPosXIndex] - camera_stereo_measurement.center_x_m;
        const double delta_y_m = track.track_state[kPosYIndex] - camera_stereo_measurement.center_y_m;
        return std::sqrt((delta_x_m * delta_x_m) + (delta_y_m * delta_y_m));
    }

    void update_track_with_camera_stereo_measurement(
        FusedTrack &track,
        const CameraStereoMeasurement &camera_stereo_measurement)
    {
        const auto update_scalar_state =
            [&track](const std::size_t state_index, const double measurement_value, const double measurement_variance)
            {
                const double innovation = measurement_value - track.track_state[state_index];
                const double innovation_variance =
                    track.state_covariance[matrix_index(state_index, state_index)] + measurement_variance;
                if (!std::isfinite(innovation) || !std::isfinite(innovation_variance) || innovation_variance <= 1e-9)
                {
                    return;
                }

                TrackStateVector kalman_gain{};
                for (std::size_t row = 0; row < kTrackStateDim; ++row)
                {
                    kalman_gain[row] =
                        track.state_covariance[matrix_index(row, state_index)] / innovation_variance;
                }

                for (std::size_t row = 0; row < kTrackStateDim; ++row)
                {
                    track.track_state[row] += kalman_gain[row] * innovation;
                }

                StateCovarianceMatrix updated_covariance = track.state_covariance;
                for (std::size_t row = 0; row < kTrackStateDim; ++row)
                {
                    for (std::size_t column = 0; column < kTrackStateDim; ++column)
                    {
                        updated_covariance[matrix_index(row, column)] =
                            track.state_covariance[matrix_index(row, column)] -
                            (kalman_gain[row] * track.state_covariance[matrix_index(state_index, column)]);
                    }
                }
                track.state_covariance = updated_covariance;
            };

        constexpr double kCameraStereoPositionVariance = 1.0;
        constexpr double kCameraStereoSizeVariance = 1.5;
        update_scalar_state(kPosXIndex, camera_stereo_measurement.center_x_m, kCameraStereoPositionVariance);
        update_scalar_state(kPosYIndex, camera_stereo_measurement.center_y_m, kCameraStereoPositionVariance);
        update_scalar_state(kLengthIndex, camera_stereo_measurement.length_m, kCameraStereoSizeVariance);
        update_scalar_state(kWidthIndex, camera_stereo_measurement.width_m, kCameraStereoSizeVariance);

        track.last_update_stamp = camera_stereo_measurement.stamp;
        track.center_z_m = camera_stereo_measurement.center_z_m;
        track.height_m = camera_stereo_measurement.height_m;
        if (!camera_stereo_measurement.classification.empty() &&
            camera_stereo_measurement.classification != "unknown")
        {
            track.classification = camera_stereo_measurement.classification;
        }

        const double support_gain = 0.5 * existence_probability_hit_gain_ *
                                    std::clamp(0.5 + (0.5 * camera_stereo_measurement.confidence), 0.0, 1.0);
        track.existence_probability = static_cast<float>(std::clamp(
            static_cast<double>(track.existence_probability) + support_gain,
            0.0,
            1.0));
    }

    void update_tracks_with_camera_stereo_measurements(
        const builtin_interfaces::msg::Time &target_stamp)
    {
        const auto camera_stereo_selection = find_nearest_camera_stereo_detections(target_stamp);
        if (!camera_stereo_selection.detections)
        {
            return;
        }

        const auto camera_to_tracking_tf = lookup_transform(
            tracking_frame_,
            camera_stereo_selection.detections->header.frame_id,
            camera_stereo_selection.detections->header.stamp);
        if (!camera_to_tracking_tf)
        {
            return;
        }

        const auto camera_stereo_measurements =
            make_camera_stereo_measurements(*camera_stereo_selection.detections, *camera_to_tracking_tf);
        std::vector<bool> measurement_used(camera_stereo_measurements.size(), false);

        for (auto &track : fused_tracks_)
        {
            if (track.associated_lidar_detection_index.has_value())
            {
                continue;
            }

            double best_distance_m = camera_stereo_association_distance_gate_m_;
            std::optional<std::size_t> best_measurement_index;
            for (std::size_t measurement_index = 0;
                 measurement_index < camera_stereo_measurements.size();
                 ++measurement_index)
            {
                if (measurement_used[measurement_index])
                {
                    continue;
                }

                const double planar_distance_m =
                    compute_planar_distance_m(track, camera_stereo_measurements[measurement_index]);
                if (planar_distance_m <= best_distance_m)
                {
                    best_distance_m = planar_distance_m;
                    best_measurement_index = measurement_index;
                }
            }

            if (!best_measurement_index)
            {
                continue;
            }

            update_track_with_camera_stereo_measurement(
                track,
                camera_stereo_measurements[*best_measurement_index]);
            track.age_in_updates += 1;
            track.stereo_supported_this_frame = true;
            measurement_used[*best_measurement_index] = true;
            refresh_track_confirmation_state(track);
        }
    }

    static constexpr std::size_t matrix_index(const std::size_t row, const std::size_t column)
    {
        return (row * kTrackStateDim) + column;
    }

    TrackStateMatrix build_state_transition_matrix(const double delta_time_seconds) const
    {
        TrackStateMatrix state_transition_matrix{};
        state_transition_matrix[matrix_index(kPosXIndex, kPosXIndex)] = 1.0;
        state_transition_matrix[matrix_index(kPosYIndex, kPosYIndex)] = 1.0;
        state_transition_matrix[matrix_index(kVelXIndex, kVelXIndex)] = 1.0;
        state_transition_matrix[matrix_index(kVelYIndex, kVelYIndex)] = 1.0;
        state_transition_matrix[matrix_index(kYawIndex, kYawIndex)] = 1.0;
        state_transition_matrix[matrix_index(kLengthIndex, kLengthIndex)] = 1.0;
        state_transition_matrix[matrix_index(kWidthIndex, kWidthIndex)] = 1.0;
        state_transition_matrix[matrix_index(kPosXIndex, kVelXIndex)] = delta_time_seconds;
        state_transition_matrix[matrix_index(kPosYIndex, kVelYIndex)] = delta_time_seconds;
        return state_transition_matrix;
    }

    TrackStateMatrix build_process_noise_matrix(const double delta_time_seconds) const
    {
        TrackStateMatrix process_noise_matrix{};
        const double position_variance =
            process_noise_position_variance_per_s2_ * delta_time_seconds * delta_time_seconds;
        const double velocity_variance =
            process_noise_velocity_variance_per_s_ * delta_time_seconds;
        const double yaw_variance =
            process_noise_yaw_variance_per_s_ * delta_time_seconds;
        const double size_variance =
            process_noise_size_variance_per_s_ * delta_time_seconds;

        process_noise_matrix[matrix_index(kPosXIndex, kPosXIndex)] = position_variance;
        process_noise_matrix[matrix_index(kPosYIndex, kPosYIndex)] = position_variance;
        process_noise_matrix[matrix_index(kVelXIndex, kVelXIndex)] = velocity_variance;
        process_noise_matrix[matrix_index(kVelYIndex, kVelYIndex)] = velocity_variance;
        process_noise_matrix[matrix_index(kYawIndex, kYawIndex)] = yaw_variance;
        process_noise_matrix[matrix_index(kLengthIndex, kLengthIndex)] = size_variance;
        process_noise_matrix[matrix_index(kWidthIndex, kWidthIndex)] = size_variance;
        return process_noise_matrix;
    }

    void predict_track(FusedTrack &track, const builtin_interfaces::msg::Time &prediction_stamp)
    {
        const double delta_time_seconds =
            compute_delta_time_seconds(track.last_predict_stamp, prediction_stamp);
        if (delta_time_seconds <= 0.0)
        {
            return;
        }

        const TrackStateMatrix state_transition_matrix =
            build_state_transition_matrix(delta_time_seconds);
        const TrackStateMatrix process_noise_matrix =
            build_process_noise_matrix(delta_time_seconds);

        TrackStateVector predicted_state = track.track_state;
        predicted_state[kPosXIndex] += track.track_state[kVelXIndex] * delta_time_seconds;
        predicted_state[kPosYIndex] += track.track_state[kVelYIndex] * delta_time_seconds;

        TrackStateMatrix predicted_covariance{};
        for (std::size_t row = 0; row < kTrackStateDim; ++row)
        {
            for (std::size_t column = 0; column < kTrackStateDim; ++column)
            {
                double predicted_entry = 0.0;
                for (std::size_t left_index = 0; left_index < kTrackStateDim; ++left_index)
                {
                    for (std::size_t right_index = 0; right_index < kTrackStateDim; ++right_index)
                    {
                        predicted_entry +=
                            state_transition_matrix[matrix_index(row, left_index)] *
                            track.state_covariance[matrix_index(left_index, right_index)] *
                            state_transition_matrix[matrix_index(column, right_index)];
                    }
                }

                predicted_covariance[matrix_index(row, column)] =
                    predicted_entry + process_noise_matrix[matrix_index(row, column)];
            }
        }

        track.track_state = predicted_state;
        track.track_state[kYawIndex] = normalize_angle_rad(track.track_state[kYawIndex]);
        track.state_covariance = predicted_covariance;
        track.last_predict_stamp = prediction_stamp;
    }

    LidarMeasurementVector build_lidar_measurement_vector(
        const LidarMeasurement &lidar_measurement) const
    {
        return {
            lidar_measurement.center_x_m,
            lidar_measurement.center_y_m,
            lidar_measurement.yaw_rad,
            lidar_measurement.length_m,
            lidar_measurement.width_m};
    }

    LidarMeasurementCovarianceMatrix build_lidar_measurement_noise_matrix() const
    {
        LidarMeasurementCovarianceMatrix measurement_noise_matrix{};
        measurement_noise_matrix[(kLidarPosXMeasurementIndex * kLidarMeasurementDim) + kLidarPosXMeasurementIndex] =
            lidar_measurement_noise_position_variance_;
        measurement_noise_matrix[(kLidarPosYMeasurementIndex * kLidarMeasurementDim) + kLidarPosYMeasurementIndex] =
            lidar_measurement_noise_position_variance_;
        measurement_noise_matrix[(kLidarYawMeasurementIndex * kLidarMeasurementDim) + kLidarYawMeasurementIndex] =
            lidar_measurement_noise_yaw_variance_;
        measurement_noise_matrix[(kLidarLengthMeasurementIndex * kLidarMeasurementDim) + kLidarLengthMeasurementIndex] =
            lidar_measurement_noise_size_variance_;
        measurement_noise_matrix[(kLidarWidthMeasurementIndex * kLidarMeasurementDim) + kLidarWidthMeasurementIndex] =
            lidar_measurement_noise_size_variance_;
        return measurement_noise_matrix;
    }

    std::size_t lidar_measurement_to_state_index(const std::size_t measurement_index) const
    {
        switch (measurement_index)
        {
        case kLidarPosXMeasurementIndex:
            return kPosXIndex;
        case kLidarPosYMeasurementIndex:
            return kPosYIndex;
        case kLidarYawMeasurementIndex:
            return kYawIndex;
        case kLidarLengthMeasurementIndex:
            return kLengthIndex;
        case kLidarWidthMeasurementIndex:
            return kWidthIndex;
        default:
            return kPosXIndex;
        }
    }

    void update_track_with_lidar_measurement(
        FusedTrack &track,
        const LidarMeasurement &lidar_measurement)
    {
        const LidarMeasurementVector measurement_vector =
            build_lidar_measurement_vector(lidar_measurement);
        const LidarMeasurementCovarianceMatrix measurement_noise_matrix =
            build_lidar_measurement_noise_matrix();

        for (std::size_t measurement_index = 0; measurement_index < kLidarMeasurementDim; ++measurement_index)
        {
            const std::size_t state_index = lidar_measurement_to_state_index(measurement_index);
            const double predicted_measurement = track.track_state[state_index];
            const double innovation =
                state_index == kYawIndex
                    ? normalize_angle_rad(measurement_vector[measurement_index] - predicted_measurement)
                    : measurement_vector[measurement_index] - predicted_measurement;
            const double innovation_variance =
                track.state_covariance[matrix_index(state_index, state_index)] +
                measurement_noise_matrix[(measurement_index * kLidarMeasurementDim) + measurement_index];

            if (!std::isfinite(innovation) || !std::isfinite(innovation_variance) || innovation_variance <= 1e-9)
            {
                continue;
            }

            TrackStateVector kalman_gain{};
            for (std::size_t row = 0; row < kTrackStateDim; ++row)
            {
                kalman_gain[row] =
                    track.state_covariance[matrix_index(row, state_index)] / innovation_variance;
            }

            for (std::size_t row = 0; row < kTrackStateDim; ++row)
            {
                track.track_state[row] += kalman_gain[row] * innovation;
            }
            track.track_state[kYawIndex] = normalize_angle_rad(track.track_state[kYawIndex]);

            StateCovarianceMatrix updated_covariance = track.state_covariance;
            for (std::size_t row = 0; row < kTrackStateDim; ++row)
            {
                for (std::size_t column = 0; column < kTrackStateDim; ++column)
                {
                    updated_covariance[matrix_index(row, column)] =
                        track.state_covariance[matrix_index(row, column)] -
                        (kalman_gain[row] * track.state_covariance[matrix_index(state_index, column)]);
                }
            }

            track.state_covariance = updated_covariance;
        }

        track.last_update_stamp = lidar_measurement.stamp;
        track.center_z_m = lidar_measurement.center_z_m;
        track.height_m = lidar_measurement.height_m;
        track.associated_lidar_detection_index = lidar_measurement.detection_index;
        track.existence_probability = static_cast<float>(std::clamp(
            static_cast<double>(track.existence_probability) + existence_probability_hit_gain_,
            0.0,
            1.0));
    }

    std::vector<int> solve_hungarian_assignment(const std::vector<std::vector<double>> &cost_matrix) const
    {
        if (cost_matrix.empty() || cost_matrix.front().empty())
        {
            return {};
        }

        const std::size_t num_rows = cost_matrix.size();
        const std::size_t num_columns = cost_matrix.front().size();
        const std::size_t matrix_size = std::max(num_rows, num_columns);
        const double large_cost = 1e9;

        std::vector<std::vector<double>> padded_costs(
            matrix_size + 1,
            std::vector<double>(matrix_size + 1, large_cost));

        for (std::size_t row = 0; row < num_rows; ++row)
        {
            for (std::size_t column = 0; column < num_columns; ++column)
            {
                padded_costs[row + 1][column + 1] = cost_matrix[row][column];
            }
        }

        std::vector<double> row_potential(matrix_size + 1, 0.0);
        std::vector<double> column_potential(matrix_size + 1, 0.0);
        std::vector<std::size_t> column_match(matrix_size + 1, 0);
        std::vector<std::size_t> augmenting_path(matrix_size + 1, 0);

        for (std::size_t row = 1; row <= matrix_size; ++row)
        {
            column_match[0] = row;
            std::size_t current_column = 0;
            std::vector<double> min_reduced_cost(matrix_size + 1, large_cost);
            std::vector<bool> used_columns(matrix_size + 1, false);

            do
            {
                used_columns[current_column] = true;
                const std::size_t current_row = column_match[current_column];
                double delta = std::numeric_limits<double>::infinity();
                std::size_t next_column = 0;

                for (std::size_t column = 1; column <= matrix_size; ++column)
                {
                    if (used_columns[column])
                    {
                        continue;
                    }

                    const double reduced_cost =
                        padded_costs[current_row][column] -
                        row_potential[current_row] -
                        column_potential[column];

                    if (reduced_cost < min_reduced_cost[column])
                    {
                        min_reduced_cost[column] = reduced_cost;
                        augmenting_path[column] = current_column;
                    }

                    if (min_reduced_cost[column] < delta)
                    {
                        delta = min_reduced_cost[column];
                        next_column = column;
                    }
                }

                for (std::size_t column = 0; column <= matrix_size; ++column)
                {
                    if (used_columns[column])
                    {
                        row_potential[column_match[column]] += delta;
                        column_potential[column] -= delta;
                    }
                    else
                    {
                        min_reduced_cost[column] -= delta;
                    }
                }

                current_column = next_column;
            }
            while (column_match[current_column] != 0);

            do
            {
                const std::size_t previous_column = augmenting_path[current_column];
                column_match[current_column] = column_match[previous_column];
                current_column = previous_column;
            }
            while (current_column != 0);
        }

        std::vector<int> assigned_column_per_row(num_rows, -1);
        for (std::size_t column = 1; column <= matrix_size; ++column)
        {
            const std::size_t matched_row = column_match[column];
            if (matched_row >= 1 && matched_row <= num_rows && column <= num_columns)
            {
                assigned_column_per_row[matched_row - 1] = static_cast<int>(column - 1);
            }
        }

        return assigned_column_per_row;
    }

    LidarAssociationResult associate_lidar_measurements_to_tracks(
        const std::vector<FusedTrack> &predicted_tracks,
        const std::vector<LidarMeasurement> &lidar_measurements) const
    {
        LidarAssociationResult association_result;

        if (predicted_tracks.empty())
        {
            for (std::size_t measurement_index = 0; measurement_index < lidar_measurements.size(); ++measurement_index)
            {
                association_result.unmatched_measurement_indices.push_back(measurement_index);
            }
            return association_result;
        }

        if (lidar_measurements.empty())
        {
            for (std::size_t track_index = 0; track_index < predicted_tracks.size(); ++track_index)
            {
                association_result.unmatched_track_indices.push_back(track_index);
            }
            return association_result;
        }

        const double ungated_cost = 1e9;
        std::vector<std::vector<double>> cost_matrix(
            predicted_tracks.size(),
            std::vector<double>(lidar_measurements.size(), ungated_cost));

        for (std::size_t track_index = 0; track_index < predicted_tracks.size(); ++track_index)
        {
            for (std::size_t measurement_index = 0; measurement_index < lidar_measurements.size(); ++measurement_index)
            {
                const double planar_distance_m =
                    compute_planar_distance_m(predicted_tracks[track_index], lidar_measurements[measurement_index]);

                if (planar_distance_m <= lidar_association_distance_gate_m_)
                {
                    cost_matrix[track_index][measurement_index] = planar_distance_m;
                }
            }
        }

        const std::vector<int> assigned_column_per_row =
            solve_hungarian_assignment(cost_matrix);
        std::vector<bool> measurement_is_matched(lidar_measurements.size(), false);

        for (std::size_t track_index = 0; track_index < predicted_tracks.size(); ++track_index)
        {
            const int assigned_measurement_index = assigned_column_per_row[track_index];
            if (assigned_measurement_index < 0)
            {
                association_result.unmatched_track_indices.push_back(track_index);
                continue;
            }

            const double assignment_cost =
                cost_matrix[track_index][static_cast<std::size_t>(assigned_measurement_index)];
            if (assignment_cost > lidar_association_distance_gate_m_)
            {
                association_result.unmatched_track_indices.push_back(track_index);
                continue;
            }

            measurement_is_matched[static_cast<std::size_t>(assigned_measurement_index)] = true;
            association_result.matches.push_back({
                track_index,
                static_cast<std::size_t>(assigned_measurement_index),
                assignment_cost});
        }

        for (std::size_t measurement_index = 0; measurement_index < lidar_measurements.size(); ++measurement_index)
        {
            if (!measurement_is_matched[measurement_index])
            {
                association_result.unmatched_measurement_indices.push_back(measurement_index);
            }
        }

        std::sort(
            association_result.matches.begin(),
            association_result.matches.end(),
            [](const LidarAssociationMatch &left_match, const LidarAssociationMatch &right_match)
            {
                return left_match.track_index < right_match.track_index;
            });

        return association_result;
    }

    void prepare_track_for_new_lidar_frame(FusedTrack &track)
    {
        track.associated_lidar_detection_index = std::nullopt;
        track.associated_camera_detection_index = std::nullopt;
        track.camera_supported_this_frame = false;
        track.stereo_supported_this_frame = false;
    }

    void advance_unmatched_track(FusedTrack &track)
    {
        track.age_in_updates += 1;
        track.existence_probability = static_cast<float>(std::clamp(
            static_cast<double>(track.existence_probability) - existence_probability_miss_decay_,
            0.0,
            1.0));
    }

    void refresh_track_confirmation_state(FusedTrack &track)
    {
        track.is_confirmed =
            static_cast<double>(track.existence_probability) >= track_confirmation_threshold_;
    }

    void validate_tracks_with_camera(
        const std_msgs::msg::Header &frame_header,
        const tf2::Transform &tracking_to_camera_tf,
        TrackerFrameMetrics &metrics)
    {
        const auto camera_selection = find_nearest_camera_detections(frame_header.stamp);
        metrics.camera_lidar_skew_ms = camera_selection.skew_ms;
        const auto camera_info = get_latest_camera_info();
        if (!camera_selection.detections || !camera_info)
        {
            return;
        }

        std::vector<bool> camera_detection_used(camera_selection.detections->detections.size(), false);
        std::vector<std::size_t> ordered_track_indices;
        ordered_track_indices.reserve(fused_tracks_.size());
        for (std::size_t track_index = 0; track_index < fused_tracks_.size(); ++track_index)
        {
            ordered_track_indices.push_back(track_index);
        }

        std::sort(
            ordered_track_indices.begin(),
            ordered_track_indices.end(),
            [this](const std::size_t lhs_index, const std::size_t rhs_index)
            {
                const auto &lhs_track = fused_tracks_[lhs_index];
                const auto &rhs_track = fused_tracks_[rhs_index];
                if (lhs_track.is_confirmed != rhs_track.is_confirmed)
                {
                    return lhs_track.is_confirmed;
                }
                if (lhs_track.existence_probability != rhs_track.existence_probability)
                {
                    return lhs_track.existence_probability > rhs_track.existence_probability;
                }
                return lhs_track.track_id < rhs_track.track_id;
            });

        for (const auto track_index : ordered_track_indices)
        {
            auto &track = fused_tracks_[track_index];
            const auto projected_roi =
                project_track_box_to_image(track, tracking_to_camera_tf, *camera_info);
            if (!projected_roi)
            {
                continue;
            }

            const auto match =
                find_best_camera_match(*projected_roi, *camera_selection.detections, camera_detection_used);
            if (!match)
            {
                continue;
            }

            camera_detection_used[match->detection_index] = true;
            track.associated_camera_detection_index = match->detection_index;
            track.camera_supported_this_frame = true;
            track.classification = class_id_to_label(match->class_id);

            const double camera_support_strength = std::clamp(0.5 * (match->iou + match->score), 0.0, 1.0);
            const double camera_support_gain = 0.5 * existence_probability_hit_gain_ * camera_support_strength;
            track.existence_probability = static_cast<float>(std::clamp(
                static_cast<double>(track.existence_probability) + camera_support_gain,
                0.0,
                1.0));
            refresh_track_confirmation_state(track);
            metrics.camera_supported_tracks += 1;
        }
    }

    struct OutputTrackPose
    {
        tf2::Vector3 center{0.0, 0.0, 0.0};
        tf2::Quaternion orientation{0.0, 0.0, 0.0, 1.0};
        tf2::Vector3 velocity{0.0, 0.0, 0.0};
    };

    OutputTrackPose transform_track_for_output(
        const FusedTrack &track,
        const tf2::Transform &tracking_to_output_tf) const
    {
        OutputTrackPose output_pose;
        const auto yaw_orientation_msg = quaternion_from_yaw(track.track_state[kYawIndex]);
        tf2::Quaternion yaw_orientation;
        tf2::fromMsg(yaw_orientation_msg, yaw_orientation);
        output_pose.center = tracking_to_output_tf * tf2::Vector3(
            track.track_state[kPosXIndex],
            track.track_state[kPosYIndex],
            track.center_z_m);
        output_pose.orientation =
            tracking_to_output_tf.getRotation() * yaw_orientation;
        output_pose.velocity = tracking_to_output_tf.getBasis() * tf2::Vector3(
            track.track_state[kVelXIndex],
            track.track_state[kVelYIndex],
            0.0);
        return output_pose;
    }

    void publish_tracked_object_markers(
        const std_msgs::msg::Header &header,
        const tf2::Transform &tracking_to_output_tf) const
    {
        if (!tracked_object_markers_publisher_)
        {
            return;
        }

        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header = header;
        clear_marker.ns = "tracked_objects";
        clear_marker.id = 0;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        for (const auto &track : fused_tracks_)
        {
            if (!track.is_confirmed)
            {
                continue;
            }

            const bool camera_supported = track.camera_supported_this_frame;
            const auto output_pose = transform_track_for_output(track, tracking_to_output_tf);
            const std_msgs::msg::ColorRGBA box_color = [&camera_supported]() {
                std_msgs::msg::ColorRGBA color;
                color.r = camera_supported ? 0.15F : 1.0F;
                color.g = 1.0F;
                color.b = camera_supported ? 0.20F : 0.10F;
                color.a = 0.35F;
                return color;
            }();

            visualization_msgs::msg::Marker box_marker;
            box_marker.header = header;
            box_marker.ns = "tracked_boxes";
            box_marker.id = static_cast<int>(track.track_id);
            box_marker.type = visualization_msgs::msg::Marker::CUBE;
            box_marker.action = visualization_msgs::msg::Marker::ADD;
            box_marker.pose.position.x = output_pose.center.x();
            box_marker.pose.position.y = output_pose.center.y();
            box_marker.pose.position.z = output_pose.center.z();
            box_marker.pose.orientation = to_geometry_quaternion(output_pose.orientation);
            box_marker.scale.x = std::max(0.05, track.track_state[kLengthIndex]);
            box_marker.scale.y = std::max(0.05, track.track_state[kWidthIndex]);
            box_marker.scale.z = std::max(0.05, track.height_m);
            box_marker.color = box_color;
            box_marker.lifetime = rclcpp::Duration::from_seconds(tracked_marker_lifetime_sec_);
            marker_array.markers.push_back(box_marker);

            visualization_msgs::msg::Marker text_marker;
            text_marker.header = header;
            text_marker.ns = "tracked_labels";
            text_marker.id = static_cast<int>(track.track_id + 10000U);
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            text_marker.pose.position.x = output_pose.center.x();
            text_marker.pose.position.y = output_pose.center.y();
            text_marker.pose.position.z = output_pose.center.z() + (0.6 * std::max(0.05, track.height_m));
            text_marker.scale.z = 0.45;
            text_marker.color.r = 1.0F;
            text_marker.color.g = 1.0F;
            text_marker.color.b = 1.0F;
            text_marker.color.a = 0.95F;
            text_marker.text =
                std::to_string(track.track_id) + " " +
                track.classification + (camera_supported ? " cam" : " lidar");
            text_marker.lifetime = rclcpp::Duration::from_seconds(tracked_marker_lifetime_sec_);
            marker_array.markers.push_back(text_marker);

            const double speed_mps = std::hypot(track.track_state[kVelXIndex], track.track_state[kVelYIndex]);
            if (speed_mps < 0.05)
            {
                continue;
            }

            visualization_msgs::msg::Marker velocity_marker;
            velocity_marker.header = header;
            velocity_marker.ns = "tracked_velocity";
            velocity_marker.id = static_cast<int>(track.track_id + 20000U);
            velocity_marker.type = visualization_msgs::msg::Marker::ARROW;
            velocity_marker.action = visualization_msgs::msg::Marker::ADD;
            velocity_marker.scale.x = 0.10;
            velocity_marker.scale.y = 0.20;
            velocity_marker.scale.z = 0.20;
            velocity_marker.color.r = 0.10F;
            velocity_marker.color.g = 0.70F;
            velocity_marker.color.b = 1.0F;
            velocity_marker.color.a = 0.95F;
            velocity_marker.lifetime = rclcpp::Duration::from_seconds(tracked_marker_lifetime_sec_);

            geometry_msgs::msg::Point start_point;
            start_point.x = output_pose.center.x();
            start_point.y = output_pose.center.y();
            start_point.z = output_pose.center.z();
            geometry_msgs::msg::Point end_point = start_point;
            end_point.x += output_pose.velocity.x();
            end_point.y += output_pose.velocity.y();
            velocity_marker.points = {start_point, end_point};
            marker_array.markers.push_back(velocity_marker);
        }

        tracked_object_markers_publisher_->publish(marker_array);
    }

    void publish_fusion_overlay(
        const std_msgs::msg::Header &frame_header,
        const tf2::Transform &tracking_to_camera_tf)
    {
        if (!publish_overlay_image_ || !fusion_overlay_publisher_)
        {
            return;
        }

        const auto image_selection = find_nearest_camera_image(frame_header.stamp);
        const auto camera_selection = find_nearest_camera_detections(frame_header.stamp);
        const auto camera_info = get_latest_camera_info();
        if (!image_selection.image || !camera_selection.detections || !camera_info)
        {
            return;
        }

        cv::Mat overlay_image;
        try
        {
            overlay_image = cv_bridge::toCvCopy(
                                image_selection.image,
                                sensor_msgs::image_encodings::BGR8)
                                ->image;
        }
        catch (const cv_bridge::Exception &exception)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Failed to build fusion overlay image: %s",
                exception.what());
            return;
        }

        const auto &camera_detections = *camera_selection.detections;
        for (const auto &camera_detection : camera_detections.detections)
        {
            const auto camera_roi = detection_2d_to_roi(camera_detection);
            draw_roi(
                overlay_image,
                camera_roi,
                cv::Scalar(255, 0, 0),
                camera_detection.results.empty()
                    ? "cam"
                    : class_id_to_label(camera_detection.results.front().hypothesis.class_id));
        }

        for (const auto &track : fused_tracks_)
        {
            if (!track.is_confirmed)
            {
                continue;
            }

            const auto projected_roi =
                project_track_box_to_image(track, tracking_to_camera_tf, *camera_info);
            if (!projected_roi)
            {
                continue;
            }

            const bool camera_supported = track.camera_supported_this_frame;
            draw_roi(
                overlay_image,
                *projected_roi,
                camera_supported ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255),
                std::to_string(track.track_id) + " " + track.classification);
        }

        std_msgs::msg::Header image_header = image_selection.image->header;
        image_header.stamp = this->get_clock()->now();
        cv_bridge::CvImage overlay_bridge(
            image_header,
            sensor_msgs::image_encodings::BGR8,
            overlay_image);
        fusion_overlay_publisher_->publish(*overlay_bridge.toImageMsg());
    }

    void prune_deleted_tracks()
    {
        const auto previous_track_count = fused_tracks_.size();
        fused_tracks_.erase(
            std::remove_if(
                fused_tracks_.begin(),
                fused_tracks_.end(),
                [this](const FusedTrack &track)
                {
                    return !is_track_state_finite(track) ||
                           static_cast<double>(track.existence_probability) <= track_deletion_threshold_;
                }),
            fused_tracks_.end());

        const auto removed_track_count = previous_track_count - fused_tracks_.size();
        if (removed_track_count > 0U)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Pruned %zu invalid or expired tracks.",
                removed_track_count);
        }
    }

    void enforce_track_capacity()
    {
        if (static_cast<int>(fused_tracks_.size()) <= max_active_tracks_)
        {
            return;
        }

        std::sort(
            fused_tracks_.begin(),
            fused_tracks_.end(),
            [](const FusedTrack &lhs, const FusedTrack &rhs)
            {
                if (lhs.is_confirmed != rhs.is_confirmed)
                {
                    return lhs.is_confirmed > rhs.is_confirmed;
                }
                if (lhs.existence_probability != rhs.existence_probability)
                {
                    return lhs.existence_probability > rhs.existence_probability;
                }
                if (lhs.camera_supported_this_frame != rhs.camera_supported_this_frame)
                {
                    return lhs.camera_supported_this_frame > rhs.camera_supported_this_frame;
                }
                if (lhs.age_in_updates != rhs.age_in_updates)
                {
                    return lhs.age_in_updates > rhs.age_in_updates;
                }
                return lhs.track_id < rhs.track_id;
            });

        const std::size_t removed_track_count =
            fused_tracks_.size() - static_cast<std::size_t>(max_active_tracks_);
        fused_tracks_.resize(static_cast<std::size_t>(max_active_tracks_));

        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "Track capacity reached. Dropped %zu weakest tracks to enforce max_active_tracks=%d.",
            removed_track_count,
            max_active_tracks_);
    }

    std::size_t count_confirmed_tracks() const
    {
        return static_cast<std::size_t>(std::count_if(
            fused_tracks_.begin(),
            fused_tracks_.end(),
            [](const FusedTrack &track)
            {
                return track.is_confirmed;
            }));
    }

    void log_tracker_frame_metrics(const TrackerFrameMetrics &metrics)
    {
        tracker_frame_count_ += 1;
        interval_frame_count_ += 1;
        cumulative_matches_ += metrics.matched_pairs;
        cumulative_new_tracks_ += metrics.new_tracks_created;
        cumulative_deleted_tracks_ += metrics.deleted_tracks;
        cumulative_processing_time_ms_ += metrics.processing_time_ms;
        interval_sum_input_lidar_detections_ += static_cast<double>(metrics.input_lidar_detections);
        interval_sum_active_tracks_ += static_cast<double>(fused_tracks_.size());
        interval_sum_confirmed_tracks_ += static_cast<double>(metrics.confirmed_tracks_after_update);
        interval_sum_camera_supported_tracks_ += static_cast<double>(metrics.camera_supported_tracks);
        interval_sum_matches_ += static_cast<double>(metrics.matched_pairs);
        interval_sum_new_tracks_ += static_cast<double>(metrics.new_tracks_created);
        interval_sum_deleted_tracks_ += static_cast<double>(metrics.deleted_tracks);
        interval_sum_unmatched_tracks_ += static_cast<double>(metrics.unmatched_tracks);
        interval_sum_unmatched_measurements_ += static_cast<double>(metrics.unmatched_measurements);
        interval_sum_average_match_distance_m_ += metrics.average_match_distance_m;
        interval_sum_max_match_distance_m_ += metrics.max_match_distance_m;
        interval_sum_camera_lidar_skew_ms_ += metrics.camera_lidar_skew_ms;
        interval_sum_frame_total_ms_ += metrics.processing_time_ms;
        interval_sum_rss_mb_ += read_current_rss_mb();
        interval_sum_peak_rss_mb_ += read_peak_rss_mb();

        if ((tracker_frame_count_ % 10U) == 0U)
        {
            const double average_matches_per_frame =
                static_cast<double>(cumulative_matches_) / static_cast<double>(tracker_frame_count_);
            const double average_processing_time_ms =
                cumulative_processing_time_ms_ / static_cast<double>(tracker_frame_count_);

            RCLCPP_INFO(
                this->get_logger(),
                "Tracker summary: frame=%zu detections=%zu active=%zu confirmed=%zu cam_supported=%zu matches=%zu new=%zu deleted=%zu unmatched_tracks=%zu unmatched_detections=%zu avg_match_dist=%.3f m max_match_dist=%.3f m cam_skew=%.1f ms frame_time=%.2f ms avg_matches=%.2f avg_frame_time=%.2f ms",
                tracker_frame_count_,
                metrics.input_lidar_detections,
                fused_tracks_.size(),
                metrics.confirmed_tracks_after_update,
                metrics.camera_supported_tracks,
                metrics.matched_pairs,
                metrics.new_tracks_created,
                metrics.deleted_tracks,
                metrics.unmatched_tracks,
                metrics.unmatched_measurements,
                metrics.average_match_distance_m,
                metrics.max_match_distance_m,
                metrics.camera_lidar_skew_ms,
                metrics.processing_time_ms,
                average_matches_per_frame,
                average_processing_time_ms);
        }

        if (!fused_tracks_.empty() && metrics.confirmed_tracks_after_update == 0U && tracker_frame_count_ > 5U)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "Tracker has active tracks but no confirmed tracks yet. Check confirmation threshold or association quality.");
        }

        if (metrics.max_match_distance_m > lidar_association_distance_gate_m_)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Observed LiDAR match distance %.3f m larger than gate %.3f m. This should not happen.",
                metrics.max_match_distance_m,
                lidar_association_distance_gate_m_);
        }

        if (interval_frame_count_ >= profiling_interval_frames_)
        {
            double avg_process_cpu_percent = 0.0;
            double effective_output_rate_hz = 0.0;
            compute_interval_resource_metrics(avg_process_cpu_percent, effective_output_rate_hz);

            const double divisor = static_cast<double>(interval_frame_count_);
            write_csv_interval_metrics(
                interval_frame_count_,
                interval_sum_input_lidar_detections_ / divisor,
                interval_sum_active_tracks_ / divisor,
                interval_sum_confirmed_tracks_ / divisor,
                interval_sum_camera_supported_tracks_ / divisor,
                interval_sum_matches_ / divisor,
                interval_sum_new_tracks_ / divisor,
                interval_sum_deleted_tracks_ / divisor,
                interval_sum_unmatched_tracks_ / divisor,
                interval_sum_unmatched_measurements_ / divisor,
                interval_sum_average_match_distance_m_ / divisor,
                interval_sum_max_match_distance_m_ / divisor,
                interval_sum_camera_lidar_skew_ms_ / divisor,
                interval_sum_frame_total_ms_ / divisor,
                avg_process_cpu_percent,
                effective_output_rate_hz,
                interval_sum_rss_mb_ / divisor,
                interval_sum_peak_rss_mb_ / divisor);

            interval_frame_count_ = 0;
            interval_sum_input_lidar_detections_ = 0.0;
            interval_sum_active_tracks_ = 0.0;
            interval_sum_confirmed_tracks_ = 0.0;
            interval_sum_camera_supported_tracks_ = 0.0;
            interval_sum_matches_ = 0.0;
            interval_sum_new_tracks_ = 0.0;
            interval_sum_deleted_tracks_ = 0.0;
            interval_sum_unmatched_tracks_ = 0.0;
            interval_sum_unmatched_measurements_ = 0.0;
            interval_sum_average_match_distance_m_ = 0.0;
            interval_sum_max_match_distance_m_ = 0.0;
            interval_sum_camera_lidar_skew_ms_ = 0.0;
            interval_sum_frame_total_ms_ = 0.0;
            interval_sum_rss_mb_ = 0.0;
            interval_sum_peak_rss_mb_ = 0.0;
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
                RCLCPP_WARN(
                    this->get_logger(),
                    "Failed to open CSV log file at '%s'. Disabling CSV logging.",
                    csv_log_file_path_.c_str());
                csv_logging_ = false;
                return;
            }

            csv_log_stream_
                << "timestamp_utc,dataset_sequence,interval_frames,avg_input_lidar_detections,avg_active_tracks,"
                << "avg_confirmed_tracks,avg_camera_supported_tracks,avg_matches,avg_new_tracks,avg_deleted_tracks,"
                << "avg_unmatched_tracks,avg_unmatched_measurements,avg_match_distance_m,avg_max_match_distance_m,"
                << "avg_camera_lidar_skew_ms,avg_frame_time_ms,avg_process_cpu_percent,effective_output_rate_hz,"
                << "avg_rss_mb,avg_peak_rss_mb,total_received_frames,total_processed_frames\n";
            csv_log_stream_.flush();

            RCLCPP_INFO(
                this->get_logger(),
                "CSV logging enabled. Writing interval metrics to '%s'.",
                csv_log_file_path_.c_str());
        }
        catch (const std::exception &exception)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Failed to initialize CSV logging: %s. Disabling CSV logging.",
                exception.what());
            csv_logging_ = false;
        }
    }

    void write_csv_interval_metrics(
        const int interval_frames,
        const double avg_input_lidar_detections,
        const double avg_active_tracks,
        const double avg_confirmed_tracks,
        const double avg_camera_supported_tracks,
        const double avg_matches,
        const double avg_new_tracks,
        const double avg_deleted_tracks,
        const double avg_unmatched_tracks,
        const double avg_unmatched_measurements,
        const double avg_match_distance_m,
        const double avg_max_match_distance_m,
        const double avg_camera_lidar_skew_ms,
        const double avg_frame_time_ms,
        const double avg_process_cpu_percent,
        const double effective_output_rate_hz,
        const double avg_rss_mb,
        const double avg_peak_rss_mb)
    {
        if (!csv_logging_ || !csv_log_stream_.is_open())
        {
            return;
        }

        csv_log_stream_ << current_utc_timestamp("%Y-%m-%dT%H:%M:%SZ") << ','
                        << dataset_sequence_ << ','
                        << interval_frames << ','
                        << std::fixed << std::setprecision(2)
                        << avg_input_lidar_detections << ','
                        << avg_active_tracks << ','
                        << avg_confirmed_tracks << ','
                        << avg_camera_supported_tracks << ','
                        << avg_matches << ','
                        << avg_new_tracks << ','
                        << avg_deleted_tracks << ','
                        << avg_unmatched_tracks << ','
                        << avg_unmatched_measurements << ','
                        << avg_match_distance_m << ','
                        << avg_max_match_distance_m << ','
                        << avg_camera_lidar_skew_ms << ','
                        << avg_frame_time_ms << ','
                        << avg_process_cpu_percent << ','
                        << effective_output_rate_hz << ','
                        << avg_rss_mb << ','
                        << avg_peak_rss_mb << ','
                        << total_received_frames_ << ','
                        << total_processed_frames_ << '\n';
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
            if (!std::isalnum(static_cast<unsigned char>(character)) &&
                character != '-' &&
                character != '_')
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

    auto_stack_msgs::msg::TrackedObjectArray build_tracked_objects_from_tracks(
        const std_msgs::msg::Header &header,
        const tf2::Transform &tracking_to_output_tf) const
    {
        auto_stack_msgs::msg::TrackedObjectArray tracked_objects_msg;
        tracked_objects_msg.header = header;

        for (const auto &track : fused_tracks_)
        {
            if (!track.is_confirmed)
            {
                continue;
            }

            auto_stack_msgs::msg::TrackedObject tracked_object;
            tracked_object.header = header;
            tracked_object.track_id = track.track_id;
            const auto output_pose = transform_track_for_output(track, tracking_to_output_tf);
            tracked_object.pose.position.x = output_pose.center.x();
            tracked_object.pose.position.y = output_pose.center.y();
            tracked_object.pose.position.z = output_pose.center.z();
            tracked_object.pose.orientation = to_geometry_quaternion(output_pose.orientation);
            tracked_object.dimensions.x = track.track_state[kLengthIndex];
            tracked_object.dimensions.y = track.track_state[kWidthIndex];
            tracked_object.dimensions.z = track.height_m;
            tracked_object.velocity.x = output_pose.velocity.x();
            tracked_object.velocity.y = output_pose.velocity.y();
            tracked_object.velocity.z = output_pose.velocity.z();
            tracked_object.existence_probability = track.existence_probability;
            tracked_object.classification = track.classification;
            tracked_object.from_lidar = true;
            tracked_object.from_camera = track.camera_supported_this_frame;
            tracked_objects_msg.objects.push_back(tracked_object);
        }

        return tracked_objects_msg;
    }

    void process_lidar_frame(const vision_msgs::msg::Detection3DArray &lidar_detections)
    {
        const auto frame_start_time = std::chrono::steady_clock::now();
        TrackerFrameMetrics metrics;
        const auto lidar_to_tracking_tf =
            lookup_transform(tracking_frame_, lidar_detections.header.frame_id, lidar_detections.header.stamp);
        if (!lidar_to_tracking_tf)
        {
            return;
        }

        const auto tracking_to_output_tf = lidar_to_tracking_tf->inverse();
        const auto camera_info = get_latest_camera_info();
        std::optional<tf2::Transform> tracking_to_camera_tf;
        if (camera_info)
        {
            tracking_to_camera_tf = lookup_transform(
                camera_info->header.frame_id,
                tracking_frame_,
                lidar_detections.header.stamp);
        }

        const auto lidar_measurements = make_lidar_measurements(lidar_detections, *lidar_to_tracking_tf);
        metrics.input_lidar_detections = lidar_measurements.size();
        metrics.tracks_before_prediction = fused_tracks_.size();

        for (auto &track : fused_tracks_)
        {
            prepare_track_for_new_lidar_frame(track);
            predict_track(track, lidar_detections.header.stamp);
        }

        const auto association_result =
            associate_lidar_measurements_to_tracks(fused_tracks_, lidar_measurements);
        metrics.matched_pairs = association_result.matches.size();
        metrics.unmatched_tracks = association_result.unmatched_track_indices.size();
        metrics.unmatched_measurements = association_result.unmatched_measurement_indices.size();

        double accumulated_match_distance_m = 0.0;
        for (const auto &association_match : association_result.matches)
        {
            accumulated_match_distance_m += association_match.planar_distance_m;
            metrics.max_match_distance_m =
                std::max(metrics.max_match_distance_m, association_match.planar_distance_m);
        }
        if (!association_result.matches.empty())
        {
            metrics.average_match_distance_m =
                accumulated_match_distance_m / static_cast<double>(association_result.matches.size());
        }

        for (const auto &association_match : association_result.matches)
        {
            auto &matched_track = fused_tracks_[association_match.track_index];
            const auto &matched_measurement = lidar_measurements[association_match.measurement_index];
            update_track_with_lidar_measurement(matched_track, matched_measurement);
            matched_track.age_in_updates += 1;
            refresh_track_confirmation_state(matched_track);
        }

        update_tracks_with_camera_stereo_measurements(lidar_detections.header.stamp);

        for (const auto unmatched_track_index : association_result.unmatched_track_indices)
        {
            auto &unmatched_track = fused_tracks_[unmatched_track_index];
            if (unmatched_track.stereo_supported_this_frame)
            {
                continue;
            }
            advance_unmatched_track(unmatched_track);
            refresh_track_confirmation_state(unmatched_track);
        }

        const std::size_t track_count_before_prune = fused_tracks_.size();
        prune_deleted_tracks();
        enforce_track_capacity();
        metrics.deleted_tracks = track_count_before_prune - fused_tracks_.size();

        const std::size_t available_track_slots =
            fused_tracks_.size() >= static_cast<std::size_t>(max_active_tracks_)
                ? 0U
                : static_cast<std::size_t>(max_active_tracks_) - fused_tracks_.size();
        const std::size_t tracks_to_create =
            std::min(association_result.unmatched_measurement_indices.size(), available_track_slots);
        for (std::size_t creation_index = 0; creation_index < tracks_to_create; ++creation_index)
        {
            const auto unmatched_measurement_index =
                association_result.unmatched_measurement_indices[creation_index];
            auto new_track = initialize_track_from_lidar_measurement(
                lidar_measurements[unmatched_measurement_index]);
            refresh_track_confirmation_state(new_track);
            fused_tracks_.push_back(new_track);
        }
        metrics.new_tracks_created = tracks_to_create;
        if (tracks_to_create < association_result.unmatched_measurement_indices.size())
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "Track capacity reached. Skipped %zu new track candidates because max_active_tracks=%d.",
                association_result.unmatched_measurement_indices.size() - tracks_to_create,
                max_active_tracks_);
        }

        if (tracking_to_camera_tf)
        {
            validate_tracks_with_camera(lidar_detections.header, *tracking_to_camera_tf, metrics);
        }
        metrics.confirmed_tracks_after_update = count_confirmed_tracks();

        const auto tracked_objects_msg = build_tracked_objects_from_tracks(
            lidar_detections.header,
            tracking_to_output_tf);
        tracked_objects_publisher_->publish(tracked_objects_msg);
        publish_tracked_object_markers(lidar_detections.header, tracking_to_output_tf);
        if (tracking_to_camera_tf)
        {
            publish_fusion_overlay(lidar_detections.header, *tracking_to_camera_tf);
        }

        const auto frame_end_time = std::chrono::steady_clock::now();
        metrics.processing_time_ms =
            std::chrono::duration<double, std::milli>(frame_end_time - frame_start_time).count();
        total_processed_frames_ += 1U;
        log_tracker_frame_metrics(metrics);
    }

    void lidar_detections_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
    {
        latest_lidar_detections_ = msg;
        if (msg)
        {
            total_received_frames_ += 1U;
            process_lidar_frame(*msg);
        }
        RCLCPP_DEBUG(
            this->get_logger(),
            "TrackingBasedFusion received %zu lidar detections.",
            msg ? msg->detections.size() : 0U);
    }

    void camera_detections_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(camera_mutex_);
            camera_detections_buffer_.push_back(msg);
            while (static_cast<int>(camera_detections_buffer_.size()) > camera_history_size_)
            {
                camera_detections_buffer_.pop_front();
            }
        }
        RCLCPP_DEBUG(
            this->get_logger(),
            "TrackingBasedFusion received %zu camera detections.",
            msg ? msg->detections.size() : 0U);
    }

    void camera_stereo_detections_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(camera_mutex_);
            camera_stereo_detections_buffer_.push_back(msg);
            while (static_cast<int>(camera_stereo_detections_buffer_.size()) > camera_history_size_)
            {
                camera_stereo_detections_buffer_.pop_front();
            }
        }
        RCLCPP_DEBUG(
            this->get_logger(),
            "TrackingBasedFusion received %zu stereo camera detections.",
            msg ? msg->detections.size() : 0U);
    }

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        latest_camera_info_ = msg;
        RCLCPP_DEBUG(this->get_logger(), "TrackingBasedFusion received camera info.");
    }

    void camera_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(camera_mutex_);
            camera_image_buffer_.push_back(msg);
            while (static_cast<int>(camera_image_buffer_.size()) > camera_history_size_)
            {
                camera_image_buffer_.pop_front();
            }
        }
        RCLCPP_DEBUG(this->get_logger(), "TrackingBasedFusion received a camera image.");
    }

    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr lidar_detections_subscription_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr camera_detections_subscription_;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr camera_stereo_detections_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_image_subscription_;

    rclcpp::Publisher<auto_stack_msgs::msg::TrackedObjectArray>::SharedPtr tracked_objects_publisher_;
    rclcpp::Publisher<auto_stack_msgs::msg::DecisionState>::SharedPtr decision_state_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fusion_overlay_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tracked_object_markers_publisher_;

    vision_msgs::msg::Detection3DArray::SharedPtr latest_lidar_detections_;
    sensor_msgs::msg::CameraInfo::SharedPtr latest_camera_info_;
    std::deque<vision_msgs::msg::Detection2DArray::SharedPtr> camera_detections_buffer_;
    std::deque<vision_msgs::msg::Detection3DArray::SharedPtr> camera_stereo_detections_buffer_;
    std::deque<sensor_msgs::msg::Image::SharedPtr> camera_image_buffer_;
    mutable std::mutex camera_mutex_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::vector<FusedTrack> fused_tracks_;
    std::uint32_t next_track_id_{1};
    std::size_t tracker_frame_count_{0};
    std::size_t cumulative_matches_{0};
    std::size_t cumulative_new_tracks_{0};
    std::size_t cumulative_deleted_tracks_{0};
    double cumulative_processing_time_ms_{0.0};
    int interval_frame_count_{0};
    double interval_sum_input_lidar_detections_{0.0};
    double interval_sum_active_tracks_{0.0};
    double interval_sum_confirmed_tracks_{0.0};
    double interval_sum_camera_supported_tracks_{0.0};
    double interval_sum_matches_{0.0};
    double interval_sum_new_tracks_{0.0};
    double interval_sum_deleted_tracks_{0.0};
    double interval_sum_unmatched_tracks_{0.0};
    double interval_sum_unmatched_measurements_{0.0};
    double interval_sum_average_match_distance_m_{0.0};
    double interval_sum_max_match_distance_m_{0.0};
    double interval_sum_camera_lidar_skew_ms_{0.0};
    double interval_sum_frame_total_ms_{0.0};
    double interval_sum_rss_mb_{0.0};
    double interval_sum_peak_rss_mb_{0.0};
    std::chrono::steady_clock::time_point interval_resource_window_start_;
    double interval_resource_window_cpu_ms_{0.0};
    std::uint64_t total_received_frames_{0};
    std::uint64_t total_processed_frames_{0};
    double lidar_association_distance_gate_m_{2.0};
    double camera_stereo_association_distance_gate_m_{3.0};
    double tracked_marker_lifetime_sec_{1.0};
    double initial_existence_probability_{0.55};
    double existence_probability_hit_gain_{0.20};
    double existence_probability_miss_decay_{0.15};
    double track_confirmation_threshold_{0.75};
    double track_deletion_threshold_{0.20};
    double process_noise_position_variance_per_s2_{0.25};
    double process_noise_velocity_variance_per_s_{1.0};
    double process_noise_yaw_variance_per_s_{0.04};
    double process_noise_size_variance_per_s_{0.05};
    double lidar_measurement_noise_position_variance_{0.25};
    double lidar_measurement_noise_yaw_variance_{0.09};
    double lidar_measurement_noise_size_variance_{0.5};
    double camera_sync_tolerance_ms_{200.0};
    double match_iou_threshold_{0.10};
    double max_match_center_distance_px_{160.0};
    double tf_lookup_timeout_ms_{100.0};
    double min_projection_depth_m_{0.10};
    int camera_history_size_{10};
    int max_active_tracks_{128};
    int profiling_interval_frames_{60};
    bool csv_logging_{false};
    bool publish_overlay_image_{true};
    std::string csv_log_dir_{"csv_logs/fusion_core"};
    std::string csv_log_file_path_;
    std::string dataset_sequence_{"unknown"};
    std::string tracking_frame_{"map"};
    std::ofstream csv_log_stream_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrackingBasedFusion>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
