#include <rclcpp/rclcpp.hpp>

#include <auto_stack_msgs/msg/decision_state.hpp>
#include <auto_stack_msgs/msg/tracked_object_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

/*
 * Tracking-based fusion model summary
 *
 * Internal EKF track state lives in the LiDAR / vehicle planar frame:
 *
 *   x = [px, py, vx, vy, length, width]^T
 *
 * where:
 *   px, py   -> object center position in meters
 *   vx, vy   -> planar velocity in meters / second
 *   length   -> object extent along the forward axis
 *   width    -> object extent along the lateral axis
 *
 * We use a constant-velocity motion model over timestep dt:
 *
 *   px' = px + vx * dt
 *   py' = py + vy * dt
 *   vx' = vx
 *   vy' = vy
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
 *   z_lidar = [px, py, length, width]^T
 *   z_lidar = H_lidar * x + v_lidar
 *
 * Camera provides a nonlinear image-space measurement:
 *
 *   z_camera = [u, v, bbox_width_px, bbox_height_px]^T
 *   z_camera = h_camera(x) + v_camera
 *
 * where h_camera(x) projects the object hypothesis into the image using
 * camera intrinsics and the LiDAR-to-camera transform.
 *
 * We intentionally keep center_z and height outside the EKF state for now.
 * They remain track metadata updated mainly from LiDAR, while the EKF focuses
 * on planar motion and footprint estimation.
 */

class TrackingBasedFusion final : public rclcpp::Node
{
public:
    TrackingBasedFusion()
        : rclcpp::Node("tracking_based_fusion")
    {
        const auto lidar_detections_topic =
            this->declare_parameter<std::string>("lidar_detections_topic", "lidar_detections");
        const auto camera_detections_topic =
            this->declare_parameter<std::string>("camera_detections_topic", "object_detections");
        const auto camera_info_topic =
            this->declare_parameter<std::string>("camera_info_topic", "p2_camera_info");
        const auto camera_image_topic =
            this->declare_parameter<std::string>("camera_image_topic", "/p2_img");
        const auto tracked_objects_topic =
            this->declare_parameter<std::string>("tracked_objects_topic", "tracked_objects");
        const auto decision_state_topic =
            this->declare_parameter<std::string>("decision_state_topic", "decision_state");
        const auto fusion_overlay_topic =
            this->declare_parameter<std::string>("fusion_overlay_topic", "fusion_overlay_image");

        lidar_detections_subscription_ =
            this->create_subscription<vision_msgs::msg::Detection3DArray>(
                lidar_detections_topic,
                10,
                std::bind(&TrackingBasedFusion::lidar_detections_callback, this, std::placeholders::_1));

        camera_detections_subscription_ =
            this->create_subscription<vision_msgs::msg::Detection2DArray>(
                camera_detections_topic,
                10,
                std::bind(&TrackingBasedFusion::camera_detections_callback, this, std::placeholders::_1));

        camera_info_subscription_ =
            this->create_subscription<sensor_msgs::msg::CameraInfo>(
                camera_info_topic,
                10,
                std::bind(&TrackingBasedFusion::camera_info_callback, this, std::placeholders::_1));

        camera_image_subscription_ =
            this->create_subscription<sensor_msgs::msg::Image>(
                camera_image_topic,
                10,
                std::bind(&TrackingBasedFusion::camera_image_callback, this, std::placeholders::_1));

        tracked_objects_publisher_ =
            this->create_publisher<auto_stack_msgs::msg::TrackedObjectArray>(tracked_objects_topic, 10);
        decision_state_publisher_ =
            this->create_publisher<auto_stack_msgs::msg::DecisionState>(decision_state_topic, 10);
        fusion_overlay_publisher_ =
            this->create_publisher<sensor_msgs::msg::Image>(fusion_overlay_topic, 10);

        RCLCPP_INFO(this->get_logger(), "TrackingBasedFusion skeleton initialized.");
    }

private:
    static constexpr std::size_t kTrackStateDim{6};
    static constexpr std::size_t kLidarMeasurementDim{4};
    static constexpr std::size_t kCameraMeasurementDim{4};

    using TrackStateVector = std::array<double, kTrackStateDim>;
    using StateCovarianceMatrix = std::array<double, kTrackStateDim * kTrackStateDim>;

    struct LidarMeasurement
    {
        vision_msgs::msg::Detection3D detection;
        std::size_t detection_index{0};
        builtin_interfaces::msg::Time stamp;
        std::string frame_id;
        double center_x_m{0.0};
        double center_y_m{0.0};
        double center_z_m{0.0};
        double length_m{0.0};
        double width_m{0.0};
        double height_m{0.0};
    };

    struct CameraMeasurement
    {
        vision_msgs::msg::Detection2D detection;
        std::size_t detection_index{0};
        builtin_interfaces::msg::Time stamp;
        std::string frame_id;
        double center_u_px{0.0};
        double center_v_px{0.0};
        double bbox_width_px{0.0};
        double bbox_height_px{0.0};
        std::string class_id;
        double confidence{0.0};
    };

    struct FusedTrack
    {
        std::uint32_t track_id{0};
        TrackStateVector track_state{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        StateCovarianceMatrix state_covariance{{0.0}};
        builtin_interfaces::msg::Time last_predict_stamp;
        builtin_interfaces::msg::Time last_update_stamp;
        std::size_t age_in_updates{0};
        std::size_t lidar_hit_count{0};
        std::size_t camera_hit_count{0};
        std::size_t consecutive_missed_updates{0};
        bool is_confirmed{false};

        double center_z_m{0.0};
        double height_m{0.0};
        std::string classification{"unknown"};
        float existence_probability{0.0F};

        std::optional<std::size_t> associated_lidar_detection_index;
        std::optional<std::size_t> associated_camera_detection_index;
        double matched_iou{0.0};
    };

    std::vector<LidarMeasurement> make_lidar_measurements(
        const vision_msgs::msg::Detection3DArray &lidar_detections) const
    {
        std::vector<LidarMeasurement> lidar_measurements;
        lidar_measurements.reserve(lidar_detections.detections.size());

        for (std::size_t detection_index = 0; detection_index < lidar_detections.detections.size(); ++detection_index)
        {
            const auto &detection = lidar_detections.detections[detection_index];

            LidarMeasurement measurement;
            measurement.detection = detection;
            measurement.detection_index = detection_index;
            measurement.stamp = lidar_detections.header.stamp;
            measurement.frame_id = lidar_detections.header.frame_id;
            measurement.center_x_m = detection.bbox.center.position.x;
            measurement.center_y_m = detection.bbox.center.position.y;
            measurement.center_z_m = detection.bbox.center.position.z;
            measurement.length_m = detection.bbox.size.x;
            measurement.width_m = detection.bbox.size.y;
            measurement.height_m = detection.bbox.size.z;

            lidar_measurements.push_back(measurement);
        }

        return lidar_measurements;
    }

    std::vector<CameraMeasurement> make_camera_measurements(
        const vision_msgs::msg::Detection2DArray &camera_detections) const
    {
        std::vector<CameraMeasurement> camera_measurements;
        camera_measurements.reserve(camera_detections.detections.size());

        for (std::size_t detection_index = 0; detection_index < camera_detections.detections.size(); ++detection_index)
        {
            const auto &detection = camera_detections.detections[detection_index];

            CameraMeasurement measurement;
            measurement.detection = detection;
            measurement.detection_index = detection_index;
            measurement.stamp = camera_detections.header.stamp;
            measurement.frame_id = camera_detections.header.frame_id;
            measurement.center_u_px = detection.bbox.center.position.x;
            measurement.center_v_px = detection.bbox.center.position.y;
            measurement.bbox_width_px = detection.bbox.size_x;
            measurement.bbox_height_px = detection.bbox.size_y;

            if (!detection.results.empty())
            {
                const auto &best_hypothesis = detection.results.front().hypothesis;
                measurement.class_id = best_hypothesis.class_id;
                measurement.confidence = best_hypothesis.score;
            }

            camera_measurements.push_back(measurement);
        }

        return camera_measurements;
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

    void lidar_detections_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
    {
        latest_lidar_detections_ = msg;
        RCLCPP_DEBUG(
            this->get_logger(),
            "TrackingBasedFusion received %zu lidar detections.",
            msg ? msg->detections.size() : 0U);
    }

    void camera_detections_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
    {
        latest_camera_detections_ = msg;
        RCLCPP_DEBUG(
            this->get_logger(),
            "TrackingBasedFusion received %zu camera detections.",
            msg ? msg->detections.size() : 0U);
    }

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        latest_camera_info_ = msg;
        RCLCPP_DEBUG(this->get_logger(), "TrackingBasedFusion received camera info.");
    }

    void camera_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        latest_camera_image_ = msg;
        RCLCPP_DEBUG(this->get_logger(), "TrackingBasedFusion received a camera image.");
    }

    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr lidar_detections_subscription_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr camera_detections_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_image_subscription_;

    rclcpp::Publisher<auto_stack_msgs::msg::TrackedObjectArray>::SharedPtr tracked_objects_publisher_;
    rclcpp::Publisher<auto_stack_msgs::msg::DecisionState>::SharedPtr decision_state_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fusion_overlay_publisher_;

    vision_msgs::msg::Detection3DArray::SharedPtr latest_lidar_detections_;
    vision_msgs::msg::Detection2DArray::SharedPtr latest_camera_detections_;
    sensor_msgs::msg::CameraInfo::SharedPtr latest_camera_info_;
    sensor_msgs::msg::Image::SharedPtr latest_camera_image_;

    std::vector<FusedTrack> fused_tracks_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrackingBasedFusion>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
