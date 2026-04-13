#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
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
#include <utility>
#include <vector>

#include <auto_stack_msgs/msg/decision_state.hpp>
#include <auto_stack_msgs/msg/tracked_object.hpp>
#include <auto_stack_msgs/msg/tracked_object_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

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
        const auto end_time = std::chrono::steady_clock::now();
        duration_ms_ = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                           end_time - start_time_)
                           .count();
    }
};

class FusionCore : public rclcpp::Node
{
public:
    FusionCore()
        : Node("fusion_core"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        this->declare_parameter<std::string>("lidar_detections_topic", "lidar_detections");
        this->declare_parameter<std::string>("camera_detections_topic", "object_detections");
        this->declare_parameter<std::string>("camera_info_topic", "p2_camera_info");
        this->declare_parameter<std::string>("tracked_objects_topic", "tracked_objects");
        this->declare_parameter<std::string>("decision_state_topic", "decision_state");
        this->declare_parameter<double>("camera_sync_tolerance_ms", 100.0);
        this->declare_parameter<double>("match_iou_threshold", 0.10);
        this->declare_parameter<double>("max_match_center_distance_px", 160.0);
        this->declare_parameter<double>("tf_lookup_timeout_ms", 100.0);
        this->declare_parameter<double>("min_projection_depth_m", 0.10);
        this->declare_parameter<double>("stop_distance_m", 6.0);
        this->declare_parameter<double>("slow_distance_m", 12.0);
        this->declare_parameter<double>("decision_lateral_gate_m", 2.5);
        this->declare_parameter<int>("camera_history_size", 10);
        this->declare_parameter<int>("profiling_interval_frames", 60);
        this->declare_parameter<bool>("enable_csv_logging", false);
        this->declare_parameter<std::string>("csv_log_dir", "csv_logs/fusion_core");
        this->declare_parameter<std::string>("dataset_sequence", "unknown");

        const auto lidar_detections_topic = this->get_parameter("lidar_detections_topic").as_string();
        const auto camera_detections_topic = this->get_parameter("camera_detections_topic").as_string();
        const auto camera_info_topic = this->get_parameter("camera_info_topic").as_string();
        const auto tracked_objects_topic = this->get_parameter("tracked_objects_topic").as_string();
        const auto decision_state_topic = this->get_parameter("decision_state_topic").as_string();

        camera_sync_tolerance_ms_ = this->get_parameter("camera_sync_tolerance_ms").as_double();
        match_iou_threshold_ = this->get_parameter("match_iou_threshold").as_double();
        max_match_center_distance_px_ = this->get_parameter("max_match_center_distance_px").as_double();
        tf_lookup_timeout_ms_ = this->get_parameter("tf_lookup_timeout_ms").as_double();
        min_projection_depth_m_ = this->get_parameter("min_projection_depth_m").as_double();
        stop_distance_m_ = this->get_parameter("stop_distance_m").as_double();
        slow_distance_m_ = this->get_parameter("slow_distance_m").as_double();
        decision_lateral_gate_m_ = this->get_parameter("decision_lateral_gate_m").as_double();
        camera_history_size_ = std::max(1, static_cast<int>(this->get_parameter("camera_history_size").as_int()));
        profiling_interval_frames_ = static_cast<int>(this->get_parameter("profiling_interval_frames").as_int());
        csv_logging_ = this->get_parameter("enable_csv_logging").as_bool();
        csv_log_dir_ = this->get_parameter("csv_log_dir").as_string();
        dataset_sequence_ = this->get_parameter("dataset_sequence").as_string();

        // Keep the camera side buffered so LiDAR can drive the frame-level fusion pass.
        camera_detections_subscription_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            camera_detections_topic,
            10,
            std::bind(&FusionCore::camera_detections_callback, this, std::placeholders::_1));

        camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic,
            10,
            std::bind(&FusionCore::camera_info_callback, this, std::placeholders::_1));

        lidar_detections_subscription_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
            lidar_detections_topic,
            10,
            std::bind(&FusionCore::lidar_detections_callback, this, std::placeholders::_1));

        tracked_objects_publisher_ =
            this->create_publisher<auto_stack_msgs::msg::TrackedObjectArray>(tracked_objects_topic, 10);
        decision_state_publisher_ =
            this->create_publisher<auto_stack_msgs::msg::DecisionState>(decision_state_topic, 10);

        initialize_csv_logging();
        RCLCPP_INFO(this->get_logger(), "FusionCore node has been initialized.");
    }

private:
    struct FusionProcessingMetrics
    {
        double buffer_age_ms{0.0};
        double tf_lookup_time_ms{0.0};
        double projection_time_ms{0.0};
        double association_time_ms{0.0};
        double decision_time_ms{0.0};
        double publish_time_ms{0.0};
        double frame_total_time_ms{0.0};
        double camera_lidar_skew_ms{0.0};
        double accepted_match_iou{0.0};
        std::size_t input_lidar_detections{0};
        std::size_t input_camera_detections{0};
        std::size_t matched_detections{0};
        std::size_t unmatched_lidar_detections{0};
        std::size_t unmatched_camera_detections{0};
        std::size_t output_tracked_objects{0};
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

    void camera_detections_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        std::lock_guard<std::mutex> lock(camera_mutex_);
        camera_detections_buffer_.push_back(msg);
        while (static_cast<int>(camera_detections_buffer_.size()) > camera_history_size_)
        {
            camera_detections_buffer_.pop_front();
        }
    }

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        std::lock_guard<std::mutex> lock(camera_mutex_);
        latest_camera_info_ = msg;
    }

    void lidar_detections_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        FusionProcessingMetrics metrics;
        total_received_frames_++;

        auto tracked_objects_msg = auto_stack_msgs::msg::TrackedObjectArray();
        auto decision_state_msg = auto_stack_msgs::msg::DecisionState();
        {
            ScopedTimer total_frame_timer(metrics.frame_total_time_ms);
            tracked_objects_msg = build_tracked_objects(*msg, metrics);
            {
                ScopedTimer decision_timer(metrics.decision_time_ms);
                decision_state_msg = build_decision_state(tracked_objects_msg);
            }
            {
                ScopedTimer publish_timer(metrics.publish_time_ms);
                tracked_objects_publisher_->publish(tracked_objects_msg);
                decision_state_publisher_->publish(decision_state_msg);
            }
        }

        metrics.output_tracked_objects = tracked_objects_msg.objects.size();
        total_processed_frames_++;
        update_profiling_metrics(metrics);

        RCLCPP_DEBUG(
            this->get_logger(),
            "Fused %zu lidar detections into %zu tracked objects",
            msg->detections.size(),
            tracked_objects_msg.objects.size());
    }

    auto_stack_msgs::msg::TrackedObjectArray build_tracked_objects(
        const vision_msgs::msg::Detection3DArray &lidar_detections,
        FusionProcessingMetrics &metrics)
    {
        auto_stack_msgs::msg::TrackedObjectArray tracked_objects_msg;
        tracked_objects_msg.header = lidar_detections.header;
        tracked_objects_msg.objects.reserve(lidar_detections.detections.size());
        metrics.input_lidar_detections = lidar_detections.detections.size();

        const auto camera_selection = find_nearest_camera_detections(lidar_detections.header.stamp);
        auto camera_detections = camera_selection.detections;
        auto camera_info = get_latest_camera_info();
        metrics.camera_lidar_skew_ms = camera_selection.skew_ms;
        std::vector<bool> camera_detection_used;
        if (camera_detections)
        {
            camera_detection_used.assign(camera_detections->detections.size(), false);
            metrics.input_camera_detections = camera_detections->detections.size();
        }

        double accumulated_tf_lookup_time_ms = 0.0;
        double accumulated_projection_time_ms = 0.0;
        double accumulated_association_time_ms = 0.0;
        double accumulated_match_iou = 0.0;

        for (std::size_t lidar_index = 0; lidar_index < lidar_detections.detections.size(); ++lidar_index)
        {
            const auto &lidar_detection = lidar_detections.detections[lidar_index];
            auto tracked_object = make_lidar_backed_object(lidar_detection, lidar_detections.header, lidar_index);

            if (camera_detections && camera_info)
            {
                double tf_lookup_time_ms = 0.0;
                double projection_time_ms = 0.0;
                const auto projected_roi =
                    project_lidar_box_to_image(lidar_detection, *camera_info, tf_lookup_time_ms, projection_time_ms);
                accumulated_tf_lookup_time_ms += tf_lookup_time_ms;
                accumulated_projection_time_ms += projection_time_ms;
                if (projected_roi)
                {
                    std::optional<CameraMatch> match;
                    {
                        double association_time_ms = 0.0;
                        ScopedTimer association_timer(association_time_ms);
                        match = find_best_camera_match(
                            *projected_roi,
                            *camera_detections,
                            camera_detection_used);
                        accumulated_association_time_ms += association_time_ms;
                    }

                    if (match)
                    {
                        tracked_object.classification = class_id_to_label(match->class_id);
                        tracked_object.existence_probability =
                            static_cast<float>(std::clamp(0.50 + 0.50 * match->score, 0.0, 1.0));
                        tracked_object.from_camera = true;
                        camera_detection_used[match->detection_index] = true;
                        accumulated_match_iou += match->iou;
                        metrics.matched_detections++;
                    }
                }
            }

            tracked_objects_msg.objects.push_back(tracked_object);
        }

        metrics.tf_lookup_time_ms = accumulated_tf_lookup_time_ms;
        metrics.projection_time_ms = accumulated_projection_time_ms;
        metrics.association_time_ms = accumulated_association_time_ms;
        metrics.unmatched_lidar_detections = metrics.input_lidar_detections - metrics.matched_detections;
        metrics.unmatched_camera_detections = metrics.input_camera_detections - metrics.matched_detections;
        if (metrics.matched_detections > 0U)
        {
            metrics.accepted_match_iou = accumulated_match_iou / static_cast<double>(metrics.matched_detections);
        }

        return tracked_objects_msg;
    }

    auto_stack_msgs::msg::TrackedObject make_lidar_backed_object(
        const vision_msgs::msg::Detection3D &lidar_detection,
        const std_msgs::msg::Header &header,
        const std::size_t lidar_index) const
    {
        auto_stack_msgs::msg::TrackedObject tracked_object;
        tracked_object.header = header;
        // Pass 1 keeps ids frame-local until a real tracker lands.
        tracked_object.track_id = static_cast<std::uint32_t>(lidar_index);
        tracked_object.pose = lidar_detection.bbox.center;
        tracked_object.dimensions.x = lidar_detection.bbox.size.x;
        tracked_object.dimensions.y = lidar_detection.bbox.size.y;
        tracked_object.dimensions.z = lidar_detection.bbox.size.z;
        tracked_object.velocity.x = 0.0;
        tracked_object.velocity.y = 0.0;
        tracked_object.velocity.z = 0.0;
        tracked_object.existence_probability = 0.55F;
        tracked_object.classification = "unknown";
        tracked_object.from_lidar = true;
        tracked_object.from_camera = false;
        return tracked_object;
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

    std::shared_ptr<sensor_msgs::msg::CameraInfo> get_latest_camera_info()
    {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        return latest_camera_info_;
    }

    std::optional<ImageRoi> project_lidar_box_to_image(
        const vision_msgs::msg::Detection3D &lidar_detection,
        const sensor_msgs::msg::CameraInfo &camera_info,
        double &tf_lookup_time_ms,
        double &projection_time_ms)
    {
        if (camera_info.header.frame_id.empty() || camera_info.width == 0U || camera_info.height == 0U)
        {
            return std::nullopt;
        }

        geometry_msgs::msg::TransformStamped lidar_to_camera;
        {
            try
            {
                ScopedTimer tf_lookup_timer(tf_lookup_time_ms);
                lidar_to_camera = tf_buffer_.lookupTransform(
                    camera_info.header.frame_id,
                    lidar_detection.header.frame_id,
                    lidar_detection.header.stamp,
                    rclcpp::Duration::from_seconds(tf_lookup_timeout_ms_ / 1000.0));
            }
            catch (const tf2::TransformException &exception)
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "TF lookup failed while projecting lidar boxes: %s",
                    exception.what());
                return std::nullopt;
            }
        }

        tf2::Transform lidar_to_camera_tf;
        tf2::fromMsg(lidar_to_camera.transform, lidar_to_camera_tf);

        {
            ScopedTimer projection_timer(projection_time_ms);
            const auto corners = build_box_corners(lidar_detection);
            double min_u = std::numeric_limits<double>::max();
            double min_v = std::numeric_limits<double>::max();
            double max_u = std::numeric_limits<double>::lowest();
            double max_v = std::numeric_limits<double>::lowest();
            std::size_t valid_corner_count = 0;

            for (const auto &corner : corners)
            {
                const auto camera_point = lidar_to_camera_tf * corner;
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

            if (valid_corner_count < 2)
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

    auto_stack_msgs::msg::DecisionState build_decision_state(
        const auto_stack_msgs::msg::TrackedObjectArray &tracked_objects_msg) const
    {
        auto_stack_msgs::msg::DecisionState decision_state_msg;
        decision_state_msg.header = tracked_objects_msg.header;

        double nearest_forward_obstacle_m = std::numeric_limits<double>::infinity();

        // Decision logic stays intentionally simple for v1: just react to the closest object ahead.
        for (const auto &tracked_object : tracked_objects_msg.objects)
        {
            if (!tracked_object.from_lidar)
            {
                continue;
            }

            const double lateral_offset_m = std::abs(tracked_object.pose.position.y);
            if (lateral_offset_m > decision_lateral_gate_m_)
            {
                continue;
            }

            const double front_face_distance_m =
                tracked_object.pose.position.x - (0.5 * tracked_object.dimensions.x);
            if (front_face_distance_m <= 0.0)
            {
                continue;
            }

            nearest_forward_obstacle_m =
                std::min(nearest_forward_obstacle_m, front_face_distance_m);
        }

        if (!std::isfinite(nearest_forward_obstacle_m))
        {
            decision_state_msg.state = auto_stack_msgs::msg::DecisionState::GO;
            decision_state_msg.nearest_obstacle_distance_m = std::numeric_limits<float>::infinity();
            decision_state_msg.reason = "no forward lidar obstacles";
            return decision_state_msg;
        }

        decision_state_msg.nearest_obstacle_distance_m =
            static_cast<float>(nearest_forward_obstacle_m);

        if (nearest_forward_obstacle_m <= stop_distance_m_)
        {
            decision_state_msg.state = auto_stack_msgs::msg::DecisionState::STOP;
            decision_state_msg.reason = "nearest obstacle inside stop distance";
        }
        else if (nearest_forward_obstacle_m <= slow_distance_m_)
        {
            decision_state_msg.state = auto_stack_msgs::msg::DecisionState::SLOW;
            decision_state_msg.reason = "nearest obstacle inside slow distance";
        }
        else
        {
            decision_state_msg.state = auto_stack_msgs::msg::DecisionState::GO;
            decision_state_msg.reason = "forward path is clear";
        }

        return decision_state_msg;
    }

    static std::array<tf2::Vector3, 8> build_box_corners(const vision_msgs::msg::Detection3D &detection)
    {
        const double half_x = detection.bbox.size.x * 0.5;
        const double half_y = detection.bbox.size.y * 0.5;
        const double half_z = detection.bbox.size.z * 0.5;

        const auto &center = detection.bbox.center.position;

        return {
            tf2::Vector3(center.x - half_x, center.y - half_y, center.z - half_z),
            tf2::Vector3(center.x - half_x, center.y - half_y, center.z + half_z),
            tf2::Vector3(center.x - half_x, center.y + half_y, center.z - half_z),
            tf2::Vector3(center.x - half_x, center.y + half_y, center.z + half_z),
            tf2::Vector3(center.x + half_x, center.y - half_y, center.z - half_z),
            tf2::Vector3(center.x + half_x, center.y - half_y, center.z + half_z),
            tf2::Vector3(center.x + half_x, center.y + half_y, center.z - half_z),
            tf2::Vector3(center.x + half_x, center.y + half_y, center.z + half_z)};
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

    static ImageRoi detection_2d_to_roi(const vision_msgs::msg::Detection2D &detection)
    {
        ImageRoi roi;
        roi.min_x = detection.bbox.center.position.x - (0.5 * detection.bbox.size_x);
        roi.max_x = detection.bbox.center.position.x + (0.5 * detection.bbox.size_x);
        roi.min_y = detection.bbox.center.position.y - (0.5 * detection.bbox.size_y);
        roi.max_y = detection.bbox.center.position.y + (0.5 * detection.bbox.size_y);
        return roi;
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

        const double lhs_area = lhs.width() * lhs.height();
        const double rhs_area = rhs.width() * rhs.height();
        const double union_area = lhs_area + rhs_area - intersection_area;

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

    void update_profiling_metrics(const FusionProcessingMetrics &metrics)
    {
        interval_sum_buffer_age_ms_ += metrics.buffer_age_ms;
        interval_sum_tf_lookup_ms_ += metrics.tf_lookup_time_ms;
        interval_sum_projection_ms_ += metrics.projection_time_ms;
        interval_sum_association_ms_ += metrics.association_time_ms;
        interval_sum_decision_ms_ += metrics.decision_time_ms;
        interval_sum_publish_ms_ += metrics.publish_time_ms;
        interval_sum_frame_total_ms_ += metrics.frame_total_time_ms;
        interval_sum_camera_lidar_skew_ms_ += metrics.camera_lidar_skew_ms;
        interval_sum_accepted_match_iou_ += metrics.accepted_match_iou;
        interval_sum_input_lidar_detections_ += metrics.input_lidar_detections;
        interval_sum_input_camera_detections_ += metrics.input_camera_detections;
        interval_sum_matched_detections_ += metrics.matched_detections;
        interval_sum_unmatched_lidar_detections_ += metrics.unmatched_lidar_detections;
        interval_sum_unmatched_camera_detections_ += metrics.unmatched_camera_detections;
        interval_sum_output_tracked_objects_ += metrics.output_tracked_objects;
        interval_frame_count_++;

        if (interval_frame_count_ >= profiling_interval_frames_)
        {
            write_csv_interval_metrics(
                interval_frame_count_,
                interval_sum_buffer_age_ms_ / interval_frame_count_,
                interval_sum_tf_lookup_ms_ / interval_frame_count_,
                interval_sum_projection_ms_ / interval_frame_count_,
                interval_sum_association_ms_ / interval_frame_count_,
                interval_sum_decision_ms_ / interval_frame_count_,
                interval_sum_publish_ms_ / interval_frame_count_,
                interval_sum_frame_total_ms_ / interval_frame_count_,
                interval_sum_camera_lidar_skew_ms_ / interval_frame_count_,
                interval_sum_accepted_match_iou_ / interval_frame_count_,
                interval_sum_input_lidar_detections_ / interval_frame_count_,
                interval_sum_input_camera_detections_ / interval_frame_count_,
                interval_sum_matched_detections_ / interval_frame_count_,
                interval_sum_unmatched_lidar_detections_ / interval_frame_count_,
                interval_sum_unmatched_camera_detections_ / interval_frame_count_,
                interval_sum_output_tracked_objects_ / interval_frame_count_);

            interval_frame_count_ = 0;
            interval_sum_buffer_age_ms_ = 0.0;
            interval_sum_tf_lookup_ms_ = 0.0;
            interval_sum_projection_ms_ = 0.0;
            interval_sum_association_ms_ = 0.0;
            interval_sum_decision_ms_ = 0.0;
            interval_sum_publish_ms_ = 0.0;
            interval_sum_frame_total_ms_ = 0.0;
            interval_sum_camera_lidar_skew_ms_ = 0.0;
            interval_sum_accepted_match_iou_ = 0.0;
            interval_sum_input_lidar_detections_ = 0.0;
            interval_sum_input_camera_detections_ = 0.0;
            interval_sum_matched_detections_ = 0.0;
            interval_sum_unmatched_lidar_detections_ = 0.0;
            interval_sum_unmatched_camera_detections_ = 0.0;
            interval_sum_output_tracked_objects_ = 0.0;
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

            csv_log_stream_ << "timestamp_utc,dataset_sequence,interval_frames,avg_buffer_age_ms,avg_tf_lookup_time_ms,avg_projection_time_ms,avg_association_time_ms,avg_decision_time_ms,avg_publish_time_ms,avg_frame_total_time_ms,avg_camera_lidar_skew_ms,avg_accepted_match_iou,avg_input_lidar_detections,avg_input_camera_detections,avg_matched_detections,avg_unmatched_lidar_detections,avg_unmatched_camera_detections,avg_output_tracked_objects,total_received_frames,total_processed_frames,total_overwritten_frames\n";
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
        double avg_tf_lookup_ms,
        double avg_projection_ms,
        double avg_association_ms,
        double avg_decision_ms,
        double avg_publish_ms,
        double avg_frame_total_ms,
        double avg_camera_lidar_skew_ms,
        double avg_accepted_match_iou,
        double avg_input_lidar_detections,
        double avg_input_camera_detections,
        double avg_matched_detections,
        double avg_unmatched_lidar_detections,
        double avg_unmatched_camera_detections,
        double avg_output_tracked_objects)
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
                        << avg_tf_lookup_ms << ','
                        << avg_projection_ms << ','
                        << avg_association_ms << ','
                        << avg_decision_ms << ','
                        << avg_publish_ms << ','
                        << avg_frame_total_ms << ','
                        << avg_camera_lidar_skew_ms << ','
                        << avg_accepted_match_iou << ','
                        << avg_input_lidar_detections << ','
                        << avg_input_camera_detections << ','
                        << avg_matched_detections << ','
                        << avg_unmatched_lidar_detections << ','
                        << avg_unmatched_camera_detections << ','
                        << avg_output_tracked_objects << ','
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

    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr lidar_detections_subscription_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr camera_detections_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
    rclcpp::Publisher<auto_stack_msgs::msg::TrackedObjectArray>::SharedPtr tracked_objects_publisher_;
    rclcpp::Publisher<auto_stack_msgs::msg::DecisionState>::SharedPtr decision_state_publisher_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::mutex camera_mutex_;
    std::deque<vision_msgs::msg::Detection2DArray::SharedPtr> camera_detections_buffer_;
    sensor_msgs::msg::CameraInfo::SharedPtr latest_camera_info_;

    double camera_sync_tolerance_ms_{100.0};
    double match_iou_threshold_{0.10};
    double max_match_center_distance_px_{160.0};
    double tf_lookup_timeout_ms_{100.0};
    double min_projection_depth_m_{0.10};
    double stop_distance_m_{6.0};
    double slow_distance_m_{12.0};
    double decision_lateral_gate_m_{2.5};
    int camera_history_size_{10};
    int interval_frame_count_{0};
    double interval_sum_buffer_age_ms_{0.0};
    double interval_sum_tf_lookup_ms_{0.0};
    double interval_sum_projection_ms_{0.0};
    double interval_sum_association_ms_{0.0};
    double interval_sum_decision_ms_{0.0};
    double interval_sum_publish_ms_{0.0};
    double interval_sum_frame_total_ms_{0.0};
    double interval_sum_camera_lidar_skew_ms_{0.0};
    double interval_sum_accepted_match_iou_{0.0};
    double interval_sum_input_lidar_detections_{0.0};
    double interval_sum_input_camera_detections_{0.0};
    double interval_sum_matched_detections_{0.0};
    double interval_sum_unmatched_lidar_detections_{0.0};
    double interval_sum_unmatched_camera_detections_{0.0};
    double interval_sum_output_tracked_objects_{0.0};
    std::uint64_t total_received_frames_{0};
    std::uint64_t total_processed_frames_{0};
    std::uint64_t total_overwritten_frames_{0};
    int profiling_interval_frames_{60};
    bool csv_logging_{false};
    std::string csv_log_dir_{"csv_logs/fusion_core"};
    std::string csv_log_file_path_;
    std::string dataset_sequence_{"unknown"};
    std::ofstream csv_log_stream_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FusionCore>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
