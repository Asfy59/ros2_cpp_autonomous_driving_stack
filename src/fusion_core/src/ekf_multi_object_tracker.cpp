#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>

#include <auto_stack_msgs/msg/tracked_object.hpp>
#include <auto_stack_msgs/msg/tracked_object_array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace
{
double age_ms_from_now(
    const std::chrono::steady_clock::time_point &received_steady,
    const std::chrono::steady_clock::time_point &now_steady)
{
    return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
               now_steady - received_steady)
        .count();
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion &orientation_msg)
{
    const double siny_cosp =
        2.0 * ((orientation_msg.w * orientation_msg.z) + (orientation_msg.x * orientation_msg.y));
    const double cosy_cosp =
        1.0 - (2.0 * ((orientation_msg.y * orientation_msg.y) + (orientation_msg.z * orientation_msg.z)));
    return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion quaternion_from_yaw(const double yaw_rad)
{
    geometry_msgs::msg::Quaternion orientation_msg;
    orientation_msg.x = 0.0;
    orientation_msg.y = 0.0;
    orientation_msg.z = std::sin(0.5 * yaw_rad);
    orientation_msg.w = std::cos(0.5 * yaw_rad);
    return orientation_msg;
}

double normalize_angle(const double angle_rad)
{
    constexpr double kTwoPi = 2.0 * M_PI;
    return std::remainder(angle_rad, kTwoPi);
}

double blend_yaw(const double current_yaw_rad, const double measured_yaw_rad, const double alpha)
{
    const double wrapped_delta = normalize_angle(measured_yaw_rad - current_yaw_rad);
    return normalize_angle(current_yaw_rad + (alpha * wrapped_delta));
}

std::string class_id_to_label(const std::string &class_id)
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
    if (class_id.empty())
    {
        return "unknown";
    }
    return "class_" + class_id;
}
}  // namespace

class EkfMultiObjectTracker final : public rclcpp::Node
{
public:
    static constexpr std::size_t kStateDim{6};
    static constexpr std::size_t kSemanticQosDepth{5};

    EkfMultiObjectTracker()
        : rclcpp::Node("ekf_multi_object_tracker"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        declare_parameter<std::string>("tracking_frame", "map");
        declare_parameter<std::string>("lidar_detections_topic", "lidar_detections");
        declare_parameter<std::string>("stereo_detections_topic", "camera_stereo_detections");
        declare_parameter<std::string>("tracked_objects_topic", "tracked_objects");
        declare_parameter<std::string>("tracked_markers_topic", "tracked_markers");
        declare_parameter<double>("tracker_rate_hz", 10.0);
        declare_parameter<int>("lidar_buffer_size", 10);
        declare_parameter<int>("stereo_buffer_size", 10);
        declare_parameter<double>("max_measurement_age_ms", 200.0);
        declare_parameter<double>("tf_lookup_timeout_ms", 50.0);
        declare_parameter<bool>("publish_debug_markers", true);
        declare_parameter<double>("marker_lifetime_sec", 0.0);
        declare_parameter<double>("yaw_update_alpha", 0.25);
        declare_parameter<double>("stereo_birth_default_length_m", 1.0);
        declare_parameter<double>("stereo_birth_default_width_m", 1.0);

        declare_parameter<double>("sigma_a_x", 1.0);
        declare_parameter<double>("sigma_a_y", 1.0);
        declare_parameter<double>("sigma_length", 0.3);
        declare_parameter<double>("sigma_width", 0.3);

        declare_parameter<double>("sigma_lidar_px", 0.3);
        declare_parameter<double>("sigma_lidar_py", 0.3);
        declare_parameter<double>("sigma_lidar_length", 0.5);
        declare_parameter<double>("sigma_lidar_width", 0.5);
        declare_parameter<double>("sigma_stereo_px", 0.7);
        declare_parameter<double>("sigma_stereo_py", 0.7);

        declare_parameter<double>("sigma_birth_lidar_px", 0.3);
        declare_parameter<double>("sigma_birth_lidar_py", 0.3);
        declare_parameter<double>("sigma_birth_lidar_vx", 2.0);
        declare_parameter<double>("sigma_birth_lidar_vy", 2.0);
        declare_parameter<double>("sigma_birth_lidar_length", 0.6);
        declare_parameter<double>("sigma_birth_lidar_width", 0.6);
        declare_parameter<double>("sigma_birth_stereo_px", 0.8);
        declare_parameter<double>("sigma_birth_stereo_py", 0.8);
        declare_parameter<double>("sigma_birth_stereo_vx", 3.0);
        declare_parameter<double>("sigma_birth_stereo_vy", 3.0);
        declare_parameter<double>("sigma_birth_stereo_length", 1.5);
        declare_parameter<double>("sigma_birth_stereo_width", 1.5);

        declare_parameter<double>("lidar_gate_mahalanobis_sq", 9.0);
        declare_parameter<double>("stereo_gate_mahalanobis_sq", 9.0);

        declare_parameter<double>("p_init_lidar", 0.35);
        declare_parameter<double>("p_init_stereo", 0.20);
        declare_parameter<double>("lidar_hit_gain", 0.15);
        declare_parameter<double>("stereo_hit_gain", 0.05);
        declare_parameter<double>("miss_decay", 0.15);
        declare_parameter<double>("confirmation_threshold", 0.75);
        declare_parameter<double>("deletion_threshold", 0.20);

        config_.tracking_frame = get_parameter("tracking_frame").as_string();
        config_.tracker_rate_hz = std::max(1.0, get_parameter("tracker_rate_hz").as_double());
        config_.lidar_buffer_size =
            std::max(1, static_cast<int>(get_parameter("lidar_buffer_size").as_int()));
        config_.stereo_buffer_size =
            std::max(1, static_cast<int>(get_parameter("stereo_buffer_size").as_int()));
        config_.max_measurement_age_ms =
            std::max(0.0, get_parameter("max_measurement_age_ms").as_double());
        config_.tf_lookup_timeout_ms =
            std::max(0.0, get_parameter("tf_lookup_timeout_ms").as_double());
        config_.publish_debug_markers = get_parameter("publish_debug_markers").as_bool();
        config_.marker_lifetime_sec =
            std::max(0.0, get_parameter("marker_lifetime_sec").as_double());
        config_.yaw_update_alpha =
            std::clamp(get_parameter("yaw_update_alpha").as_double(), 0.0, 1.0);
        config_.stereo_birth_default_length_m =
            std::max(0.1, get_parameter("stereo_birth_default_length_m").as_double());
        config_.stereo_birth_default_width_m =
            std::max(0.1, get_parameter("stereo_birth_default_width_m").as_double());

        config_.sigma_a_x = std::max(0.0, get_parameter("sigma_a_x").as_double());
        config_.sigma_a_y = std::max(0.0, get_parameter("sigma_a_y").as_double());
        config_.sigma_length = std::max(0.0, get_parameter("sigma_length").as_double());
        config_.sigma_width = std::max(0.0, get_parameter("sigma_width").as_double());

        config_.sigma_lidar_px = std::max(1e-6, get_parameter("sigma_lidar_px").as_double());
        config_.sigma_lidar_py = std::max(1e-6, get_parameter("sigma_lidar_py").as_double());
        config_.sigma_lidar_length =
            std::max(1e-6, get_parameter("sigma_lidar_length").as_double());
        config_.sigma_lidar_width =
            std::max(1e-6, get_parameter("sigma_lidar_width").as_double());
        config_.sigma_stereo_px = std::max(1e-6, get_parameter("sigma_stereo_px").as_double());
        config_.sigma_stereo_py = std::max(1e-6, get_parameter("sigma_stereo_py").as_double());

        config_.sigma_birth_lidar_px =
            std::max(1e-6, get_parameter("sigma_birth_lidar_px").as_double());
        config_.sigma_birth_lidar_py =
            std::max(1e-6, get_parameter("sigma_birth_lidar_py").as_double());
        config_.sigma_birth_lidar_vx =
            std::max(1e-6, get_parameter("sigma_birth_lidar_vx").as_double());
        config_.sigma_birth_lidar_vy =
            std::max(1e-6, get_parameter("sigma_birth_lidar_vy").as_double());
        config_.sigma_birth_lidar_length =
            std::max(1e-6, get_parameter("sigma_birth_lidar_length").as_double());
        config_.sigma_birth_lidar_width =
            std::max(1e-6, get_parameter("sigma_birth_lidar_width").as_double());
        config_.sigma_birth_stereo_px =
            std::max(1e-6, get_parameter("sigma_birth_stereo_px").as_double());
        config_.sigma_birth_stereo_py =
            std::max(1e-6, get_parameter("sigma_birth_stereo_py").as_double());
        config_.sigma_birth_stereo_vx =
            std::max(1e-6, get_parameter("sigma_birth_stereo_vx").as_double());
        config_.sigma_birth_stereo_vy =
            std::max(1e-6, get_parameter("sigma_birth_stereo_vy").as_double());
        config_.sigma_birth_stereo_length =
            std::max(1e-6, get_parameter("sigma_birth_stereo_length").as_double());
        config_.sigma_birth_stereo_width =
            std::max(1e-6, get_parameter("sigma_birth_stereo_width").as_double());

        config_.lidar_gate_mahalanobis_sq =
            std::max(0.1, get_parameter("lidar_gate_mahalanobis_sq").as_double());
        config_.stereo_gate_mahalanobis_sq =
            std::max(0.1, get_parameter("stereo_gate_mahalanobis_sq").as_double());

        config_.p_init_lidar = std::clamp(get_parameter("p_init_lidar").as_double(), 0.0, 1.0);
        config_.p_init_stereo = std::clamp(get_parameter("p_init_stereo").as_double(), 0.0, 1.0);
        config_.lidar_hit_gain = std::clamp(get_parameter("lidar_hit_gain").as_double(), 0.0, 1.0);
        config_.stereo_hit_gain =
            std::clamp(get_parameter("stereo_hit_gain").as_double(), 0.0, 1.0);
        config_.miss_decay = std::clamp(get_parameter("miss_decay").as_double(), 0.0, 1.0);
        config_.confirmation_threshold =
            std::clamp(get_parameter("confirmation_threshold").as_double(), 0.0, 1.0);
        config_.deletion_threshold =
            std::clamp(get_parameter("deletion_threshold").as_double(), 0.0, config_.confirmation_threshold);

        const auto semantic_qos =
            rclcpp::QoS(rclcpp::KeepLast(kSemanticQosDepth)).reliable().durability_volatile();
        const auto lidar_topic = get_parameter("lidar_detections_topic").as_string();
        const auto stereo_topic = get_parameter("stereo_detections_topic").as_string();
        const auto tracked_objects_topic = get_parameter("tracked_objects_topic").as_string();
        const auto tracked_markers_topic = get_parameter("tracked_markers_topic").as_string();

        lidar_subscription_ =
            create_subscription<vision_msgs::msg::Detection3DArray>(
                lidar_topic,
                semantic_qos,
                std::bind(&EkfMultiObjectTracker::lidar_callback, this, std::placeholders::_1));
        stereo_subscription_ =
            create_subscription<vision_msgs::msg::Detection3DArray>(
                stereo_topic,
                semantic_qos,
                std::bind(&EkfMultiObjectTracker::stereo_callback, this, std::placeholders::_1));

        tracked_objects_publisher_ =
            create_publisher<auto_stack_msgs::msg::TrackedObjectArray>(
                tracked_objects_topic,
                semantic_qos);

        if (config_.publish_debug_markers)
        {
            tracked_markers_publisher_ =
                create_publisher<visualization_msgs::msg::MarkerArray>(
                    tracked_markers_topic,
                    semantic_qos);
        }

        tracker_timer_ =
            create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(1.0 / config_.tracker_rate_hz)),
                std::bind(&EkfMultiObjectTracker::run_tracker_iteration, this));

        RCLCPP_INFO(
            get_logger(),
            "ekf_multi_object_tracker initialized. frame=%s rate=%.1fHz lidar_buffer=%d stereo_buffer=%d",
            config_.tracking_frame.c_str(),
            config_.tracker_rate_hz,
            config_.lidar_buffer_size,
            config_.stereo_buffer_size);
    }

private:
    using StateVector = Eigen::Matrix<double, kStateDim, 1>;
    using CovarianceMatrix = Eigen::Matrix<double, kStateDim, kStateDim>;
    using LidarMeasurementVector = Eigen::Matrix<double, 4, 1>;
    using LidarMeasurementMatrix = Eigen::Matrix<double, 4, kStateDim>;
    using LidarMeasurementCovariance = Eigen::Matrix<double, 4, 4>;
    using LidarKalmanGain = Eigen::Matrix<double, kStateDim, 4>;
    using StereoMeasurementVector = Eigen::Matrix<double, 2, 1>;
    using StereoMeasurementMatrix = Eigen::Matrix<double, 2, kStateDim>;
    using StereoMeasurementCovariance = Eigen::Matrix<double, 2, 2>;
    using StereoKalmanGain = Eigen::Matrix<double, kStateDim, 2>;

    struct TrackerConfig
    {
        std::string tracking_frame{"map"};
        double tracker_rate_hz{10.0};
        int lidar_buffer_size{10};
        int stereo_buffer_size{10};
        double max_measurement_age_ms{200.0};
        double tf_lookup_timeout_ms{50.0};
        bool publish_debug_markers{true};
        double marker_lifetime_sec{0.0};
        double yaw_update_alpha{0.25};
        double stereo_birth_default_length_m{1.0};
        double stereo_birth_default_width_m{1.0};

        double sigma_a_x{1.0};
        double sigma_a_y{1.0};
        double sigma_length{0.3};
        double sigma_width{0.3};

        double sigma_lidar_px{0.3};
        double sigma_lidar_py{0.3};
        double sigma_lidar_length{0.5};
        double sigma_lidar_width{0.5};
        double sigma_stereo_px{0.7};
        double sigma_stereo_py{0.7};

        double sigma_birth_lidar_px{0.3};
        double sigma_birth_lidar_py{0.3};
        double sigma_birth_lidar_vx{2.0};
        double sigma_birth_lidar_vy{2.0};
        double sigma_birth_lidar_length{0.6};
        double sigma_birth_lidar_width{0.6};
        double sigma_birth_stereo_px{0.8};
        double sigma_birth_stereo_py{0.8};
        double sigma_birth_stereo_vx{3.0};
        double sigma_birth_stereo_vy{3.0};
        double sigma_birth_stereo_length{1.5};
        double sigma_birth_stereo_width{1.5};

        double lidar_gate_mahalanobis_sq{9.0};
        double stereo_gate_mahalanobis_sq{9.0};

        double p_init_lidar{0.35};
        double p_init_stereo{0.20};
        double lidar_hit_gain{0.15};
        double stereo_hit_gain{0.05};
        double miss_decay{0.15};
        double confirmation_threshold{0.75};
        double deletion_threshold{0.20};
    };

    struct Track
    {
        std::uint32_t track_id{0};
        StateVector x = StateVector::Zero();
        CovarianceMatrix P = CovarianceMatrix::Zero();
        double pz_m{0.0};
        double height_m{0.0};
        double yaw_rad{0.0};
        double existence_probability{0.0};
        bool confirmed{false};
        bool lidar_supported_this_iteration{false};
        bool stereo_supported_this_iteration{false};
        std::string classification{"unknown"};
        rclcpp::Time last_predict_time{0, 0, RCL_ROS_TIME};
        rclcpp::Time last_update_time{0, 0, RCL_ROS_TIME};
    };

    struct BufferedDetectionBatch
    {
        vision_msgs::msg::Detection3DArray::SharedPtr msg;
        std::chrono::steady_clock::time_point received_steady;
    };

    struct IterationInputs
    {
        vision_msgs::msg::Detection3DArray::SharedPtr lidar_batch;
        vision_msgs::msg::Detection3DArray::SharedPtr stereo_batch;
    };

    struct LidarMeasurement
    {
        std::size_t detection_index{0};
        rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
        double px_m{0.0};
        double py_m{0.0};
        double pz_m{0.0};
        double length_m{0.0};
        double width_m{0.0};
        double height_m{0.0};
        double yaw_rad{0.0};
    };

    struct StereoMeasurement
    {
        std::size_t detection_index{0};
        rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
        double px_m{0.0};
        double py_m{0.0};
        double pz_m{0.0};
        double length_m{0.0};
        double width_m{0.0};
        double height_m{0.0};
        std::string classification{"unknown"};
        double confidence{0.0};
    };

    struct LidarAssociationResult
    {
        std::vector<std::pair<std::size_t, std::size_t>> matches;
        std::vector<std::size_t> unmatched_track_indices;
        std::vector<std::size_t> unmatched_measurement_indices;
    };

    struct StereoAssociationResult
    {
        std::vector<std::pair<std::size_t, std::size_t>> matches;
        std::vector<std::size_t> unmatched_track_indices;
        std::vector<std::size_t> unmatched_measurement_indices;
    };

    CovarianceMatrix make_transition_matrix(const double dt) const
    {
        CovarianceMatrix F = CovarianceMatrix::Identity();
        F(0, 2) = dt;
        F(1, 3) = dt;
        return F;
    }

    CovarianceMatrix make_process_noise_matrix(const double dt) const
    {
        CovarianceMatrix Q = CovarianceMatrix::Zero();

        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt2 * dt2;

        const double q_x = config_.sigma_a_x * config_.sigma_a_x;
        const double q_y = config_.sigma_a_y * config_.sigma_a_y;

        Q(0, 0) = q_x * dt4 * 0.25;
        Q(0, 2) = q_x * dt3 * 0.5;
        Q(2, 0) = q_x * dt3 * 0.5;
        Q(2, 2) = q_x * dt2;

        Q(1, 1) = q_y * dt4 * 0.25;
        Q(1, 3) = q_y * dt3 * 0.5;
        Q(3, 1) = q_y * dt3 * 0.5;
        Q(3, 3) = q_y * dt2;

        Q(4, 4) = config_.sigma_length * config_.sigma_length * dt;
        Q(5, 5) = config_.sigma_width * config_.sigma_width * dt;

        return Q;
    }

    void predict_track(Track &track, const rclcpp::Time &prediction_stamp)
    {
        if (track.last_predict_time.nanoseconds() == 0)
        {
            track.last_predict_time = prediction_stamp;
            return;
        }

        const double dt = (prediction_stamp - track.last_predict_time).seconds();
        if (dt <= 0.0)
        {
            track.last_predict_time = prediction_stamp;
            return;
        }

        const CovarianceMatrix F = make_transition_matrix(dt);
        const CovarianceMatrix Q = make_process_noise_matrix(dt);

        track.x = F * track.x;
        track.P = F * track.P * F.transpose() + Q;
        track.last_predict_time = prediction_stamp;
    }

    void predict_tracks(const rclcpp::Time &prediction_stamp)
    {
        for (auto &track : tracks_)
        {
            predict_track(track, prediction_stamp);
        }
    }

    void prepare_tracks_for_iteration()
    {
        for (auto &track : tracks_)
        {
            track.lidar_supported_this_iteration = false;
            track.stereo_supported_this_iteration = false;
        }
    }

    static LidarMeasurementMatrix make_lidar_measurement_matrix()
    {
        LidarMeasurementMatrix H = LidarMeasurementMatrix::Zero();
        H(0, 0) = 1.0;
        H(1, 1) = 1.0;
        H(2, 4) = 1.0;
        H(3, 5) = 1.0;
        return H;
    }

    LidarMeasurementCovariance make_lidar_measurement_covariance() const
    {
        LidarMeasurementCovariance R = LidarMeasurementCovariance::Zero();
        R(0, 0) = config_.sigma_lidar_px * config_.sigma_lidar_px;
        R(1, 1) = config_.sigma_lidar_py * config_.sigma_lidar_py;
        R(2, 2) = config_.sigma_lidar_length * config_.sigma_lidar_length;
        R(3, 3) = config_.sigma_lidar_width * config_.sigma_lidar_width;
        return R;
    }

    static LidarMeasurementVector make_lidar_measurement_vector(const LidarMeasurement &measurement)
    {
        LidarMeasurementVector z;
        z << measurement.px_m, measurement.py_m, measurement.length_m, measurement.width_m;
        return z;
    }

    static StereoMeasurementMatrix make_stereo_measurement_matrix()
    {
        StereoMeasurementMatrix H = StereoMeasurementMatrix::Zero();
        H(0, 0) = 1.0;
        H(1, 1) = 1.0;
        return H;
    }

    StereoMeasurementCovariance make_stereo_measurement_covariance() const
    {
        StereoMeasurementCovariance R = StereoMeasurementCovariance::Zero();
        R(0, 0) = config_.sigma_stereo_px * config_.sigma_stereo_px;
        R(1, 1) = config_.sigma_stereo_py * config_.sigma_stereo_py;
        return R;
    }

    static StereoMeasurementVector make_stereo_measurement_vector(const StereoMeasurement &measurement)
    {
        StereoMeasurementVector z;
        z << measurement.px_m, measurement.py_m;
        return z;
    }

    static std::vector<int> solve_assignment_hungarian(const std::vector<std::vector<double>> &cost_matrix)
    {
        const int n = static_cast<int>(cost_matrix.size());
        if (n == 0)
        {
            return {};
        }

        const double kInfinity = std::numeric_limits<double>::infinity();
        std::vector<double> u(n + 1, 0.0);
        std::vector<double> v(n + 1, 0.0);
        std::vector<int> p(n + 1, 0);
        std::vector<int> way(n + 1, 0);

        for (int i = 1; i <= n; ++i)
        {
            p[0] = i;
            int j0 = 0;
            std::vector<double> minv(n + 1, kInfinity);
            std::vector<bool> used(n + 1, false);

            do
            {
                used[j0] = true;
                const int i0 = p[j0];
                double delta = kInfinity;
                int j1 = 0;

                for (int j = 1; j <= n; ++j)
                {
                    if (used[j])
                    {
                        continue;
                    }

                    const double cur = cost_matrix[static_cast<std::size_t>(i0 - 1)]
                                                  [static_cast<std::size_t>(j - 1)] -
                                      u[static_cast<std::size_t>(i0)] - v[static_cast<std::size_t>(j)];
                    if (cur < minv[static_cast<std::size_t>(j)])
                    {
                        minv[static_cast<std::size_t>(j)] = cur;
                        way[static_cast<std::size_t>(j)] = j0;
                    }
                    if (minv[static_cast<std::size_t>(j)] < delta)
                    {
                        delta = minv[static_cast<std::size_t>(j)];
                        j1 = j;
                    }
                }

                for (int j = 0; j <= n; ++j)
                {
                    if (used[static_cast<std::size_t>(j)])
                    {
                        u[static_cast<std::size_t>(p[static_cast<std::size_t>(j)])] += delta;
                        v[static_cast<std::size_t>(j)] -= delta;
                    }
                    else
                    {
                        minv[static_cast<std::size_t>(j)] -= delta;
                    }
                }
                j0 = j1;
            } while (p[static_cast<std::size_t>(j0)] != 0);

            do
            {
                const int j1 = way[static_cast<std::size_t>(j0)];
                p[static_cast<std::size_t>(j0)] = p[static_cast<std::size_t>(j1)];
                j0 = j1;
            } while (j0 != 0);
        }

        std::vector<int> assignment(static_cast<std::size_t>(n), -1);
        for (int j = 1; j <= n; ++j)
        {
            if (p[static_cast<std::size_t>(j)] > 0)
            {
                assignment[static_cast<std::size_t>(p[static_cast<std::size_t>(j)] - 1)] = j - 1;
            }
        }
        return assignment;
    }

    LidarAssociationResult associate_lidar_measurements_to_tracks(
        const std::vector<LidarMeasurement> &lidar_measurements) const
    {
        LidarAssociationResult result;

        const std::size_t track_count = tracks_.size();
        const std::size_t measurement_count = lidar_measurements.size();

        if (track_count == 0)
        {
            result.unmatched_measurement_indices.resize(measurement_count);
            for (std::size_t i = 0; i < measurement_count; ++i)
            {
                result.unmatched_measurement_indices[i] = i;
            }
            return result;
        }
        if (measurement_count == 0)
        {
            result.unmatched_track_indices.resize(track_count);
            for (std::size_t i = 0; i < track_count; ++i)
            {
                result.unmatched_track_indices[i] = i;
            }
            return result;
        }

        const LidarMeasurementMatrix H = make_lidar_measurement_matrix();
        const LidarMeasurementCovariance R = make_lidar_measurement_covariance();
        const double dummy_cost = config_.lidar_gate_mahalanobis_sq + 1.0;
        const double invalid_cost = 1e9;
        const std::size_t square_size = track_count + measurement_count;
        std::vector<std::vector<double>> cost_matrix(
            square_size,
            std::vector<double>(square_size, 0.0));

        for (std::size_t track_index = 0; track_index < track_count; ++track_index)
        {
            const auto &track = tracks_[track_index];
            for (std::size_t measurement_index = 0; measurement_index < measurement_count; ++measurement_index)
            {
                const auto z = make_lidar_measurement_vector(lidar_measurements[measurement_index]);
                const auto innovation = z - (H * track.x);
                const auto innovation_covariance = H * track.P * H.transpose() + R;
                const Eigen::LDLT<LidarMeasurementCovariance> solver(innovation_covariance);
                double mahalanobis_sq = invalid_cost;

                if (solver.info() == Eigen::Success)
                {
                    const auto solved_innovation = solver.solve(innovation);
                    mahalanobis_sq = innovation.dot(solved_innovation);
                }

                cost_matrix[track_index][measurement_index] =
                    (std::isfinite(mahalanobis_sq) &&
                     mahalanobis_sq <= config_.lidar_gate_mahalanobis_sq)
                        ? mahalanobis_sq
                        : invalid_cost;
            }

            for (std::size_t dummy_col = 0; dummy_col < track_count; ++dummy_col)
            {
                cost_matrix[track_index][measurement_count + dummy_col] = dummy_cost;
            }
        }

        for (std::size_t dummy_row = 0; dummy_row < measurement_count; ++dummy_row)
        {
            for (std::size_t measurement_index = 0; measurement_index < measurement_count; ++measurement_index)
            {
                cost_matrix[track_count + dummy_row][measurement_index] = dummy_cost;
            }
        }

        const auto assignment = solve_assignment_hungarian(cost_matrix);
        std::vector<bool> matched_measurements(measurement_count, false);

        for (std::size_t track_index = 0; track_index < track_count; ++track_index)
        {
            const int assigned_col = assignment[track_index];
            if (assigned_col >= 0 &&
                static_cast<std::size_t>(assigned_col) < measurement_count &&
                cost_matrix[track_index][static_cast<std::size_t>(assigned_col)] < invalid_cost)
            {
                result.matches.emplace_back(track_index, static_cast<std::size_t>(assigned_col));
                matched_measurements[static_cast<std::size_t>(assigned_col)] = true;
            }
            else
            {
                result.unmatched_track_indices.push_back(track_index);
            }
        }

        for (std::size_t measurement_index = 0; measurement_index < measurement_count; ++measurement_index)
        {
            if (!matched_measurements[measurement_index])
            {
                result.unmatched_measurement_indices.push_back(measurement_index);
            }
        }

        return result;
    }

    StereoAssociationResult associate_stereo_measurements_to_tracks(
        const std::vector<StereoMeasurement> &stereo_measurements) const
    {
        StereoAssociationResult result;

        const std::size_t track_count = tracks_.size();
        const std::size_t measurement_count = stereo_measurements.size();

        if (track_count == 0)
        {
            result.unmatched_measurement_indices.resize(measurement_count);
            for (std::size_t i = 0; i < measurement_count; ++i)
            {
                result.unmatched_measurement_indices[i] = i;
            }
            return result;
        }
        if (measurement_count == 0)
        {
            result.unmatched_track_indices.resize(track_count);
            for (std::size_t i = 0; i < track_count; ++i)
            {
                result.unmatched_track_indices[i] = i;
            }
            return result;
        }

        const StereoMeasurementMatrix H = make_stereo_measurement_matrix();
        const StereoMeasurementCovariance R = make_stereo_measurement_covariance();
        const double dummy_cost = config_.stereo_gate_mahalanobis_sq + 1.0;
        const double invalid_cost = 1e9;
        const std::size_t square_size = track_count + measurement_count;
        std::vector<std::vector<double>> cost_matrix(
            square_size,
            std::vector<double>(square_size, 0.0));

        for (std::size_t track_index = 0; track_index < track_count; ++track_index)
        {
            const auto &track = tracks_[track_index];
            for (std::size_t measurement_index = 0; measurement_index < measurement_count; ++measurement_index)
            {
                const auto z = make_stereo_measurement_vector(stereo_measurements[measurement_index]);
                const auto innovation = z - (H * track.x);
                const auto innovation_covariance = H * track.P * H.transpose() + R;
                const Eigen::LDLT<StereoMeasurementCovariance> solver(innovation_covariance);
                double mahalanobis_sq = invalid_cost;

                if (solver.info() == Eigen::Success)
                {
                    const auto solved_innovation = solver.solve(innovation);
                    mahalanobis_sq = innovation.dot(solved_innovation);
                }

                cost_matrix[track_index][measurement_index] =
                    (std::isfinite(mahalanobis_sq) &&
                     mahalanobis_sq <= config_.stereo_gate_mahalanobis_sq)
                        ? mahalanobis_sq
                        : invalid_cost;
            }

            for (std::size_t dummy_col = 0; dummy_col < track_count; ++dummy_col)
            {
                cost_matrix[track_index][measurement_count + dummy_col] = dummy_cost;
            }
        }

        for (std::size_t dummy_row = 0; dummy_row < measurement_count; ++dummy_row)
        {
            for (std::size_t measurement_index = 0; measurement_index < measurement_count; ++measurement_index)
            {
                cost_matrix[track_count + dummy_row][measurement_index] = dummy_cost;
            }
        }

        const auto assignment = solve_assignment_hungarian(cost_matrix);
        std::vector<bool> matched_measurements(measurement_count, false);

        for (std::size_t track_index = 0; track_index < track_count; ++track_index)
        {
            const int assigned_col = assignment[track_index];
            if (assigned_col >= 0 &&
                static_cast<std::size_t>(assigned_col) < measurement_count &&
                cost_matrix[track_index][static_cast<std::size_t>(assigned_col)] < invalid_cost)
            {
                result.matches.emplace_back(track_index, static_cast<std::size_t>(assigned_col));
                matched_measurements[static_cast<std::size_t>(assigned_col)] = true;
            }
            else
            {
                result.unmatched_track_indices.push_back(track_index);
            }
        }

        for (std::size_t measurement_index = 0; measurement_index < measurement_count; ++measurement_index)
        {
            if (!matched_measurements[measurement_index])
            {
                result.unmatched_measurement_indices.push_back(measurement_index);
            }
        }

        return result;
    }

    void apply_lidar_update(Track &track, const LidarMeasurement &measurement)
    {
        const LidarMeasurementMatrix H = make_lidar_measurement_matrix();
        const LidarMeasurementCovariance R = make_lidar_measurement_covariance();
        const LidarMeasurementVector z = make_lidar_measurement_vector(measurement);
        const LidarMeasurementVector innovation = z - (H * track.x);
        const LidarMeasurementCovariance innovation_covariance = H * track.P * H.transpose() + R;
        const Eigen::LDLT<LidarMeasurementCovariance> solver(innovation_covariance);
        if (solver.info() != Eigen::Success)
        {
            return;
        }

        const LidarKalmanGain K =
            track.P * H.transpose() * solver.solve(LidarMeasurementCovariance::Identity());
        track.x = track.x + (K * innovation);

        const CovarianceMatrix identity = CovarianceMatrix::Identity();
        const CovarianceMatrix correction = identity - (K * H);
        track.P = correction * track.P * correction.transpose() + K * R * K.transpose();
        track.P = 0.5 * (track.P + track.P.transpose());

        track.pz_m = measurement.pz_m;
        track.height_m = measurement.height_m;
        track.yaw_rad = blend_yaw(track.yaw_rad, measurement.yaw_rad, config_.yaw_update_alpha);
        track.existence_probability =
            std::clamp(track.existence_probability + config_.lidar_hit_gain, 0.0, 1.0);
        track.confirmed = track.existence_probability >= config_.confirmation_threshold;
        track.lidar_supported_this_iteration = true;
        track.last_update_time = measurement.stamp;
    }

    void apply_stereo_update(Track &track, const StereoMeasurement &measurement)
    {
        const StereoMeasurementMatrix H = make_stereo_measurement_matrix();
        const StereoMeasurementCovariance R = make_stereo_measurement_covariance();
        const StereoMeasurementVector z = make_stereo_measurement_vector(measurement);
        const StereoMeasurementVector innovation = z - (H * track.x);
        const StereoMeasurementCovariance innovation_covariance = H * track.P * H.transpose() + R;
        const Eigen::LDLT<StereoMeasurementCovariance> solver(innovation_covariance);
        if (solver.info() != Eigen::Success)
        {
            return;
        }

        const StereoMeasurementVector solved_innovation = solver.solve(innovation);
        const double mahalanobis_sq = innovation.dot(solved_innovation);
        const double support_quality =
            std::clamp(1.0 - (mahalanobis_sq / config_.stereo_gate_mahalanobis_sq), 0.0, 1.0);
        const StereoKalmanGain K =
            track.P * H.transpose() * solver.solve(StereoMeasurementCovariance::Identity());
        track.x = track.x + (K * innovation);

        const CovarianceMatrix identity = CovarianceMatrix::Identity();
        const CovarianceMatrix correction = identity - (K * H);
        track.P = correction * track.P * correction.transpose() + K * R * K.transpose();
        track.P = 0.5 * (track.P + track.P.transpose());

        track.pz_m = measurement.pz_m;
        track.height_m = measurement.height_m;
        track.existence_probability = std::clamp(
            track.existence_probability + (config_.stereo_hit_gain * support_quality), 0.0, 1.0);
        track.confirmed = track.existence_probability >= config_.confirmation_threshold;
        track.stereo_supported_this_iteration = true;
        if (measurement.classification != "unknown")
        {
            track.classification = measurement.classification;
        }
        track.last_update_time = measurement.stamp;
    }

    void birth_unmatched_lidar_tracks(
        const std::vector<LidarMeasurement> &lidar_measurements,
        const std::vector<std::size_t> &unmatched_measurement_indices)
    {
        tracks_.reserve(tracks_.size() + unmatched_measurement_indices.size());
        for (const std::size_t measurement_index : unmatched_measurement_indices)
        {
            tracks_.push_back(
                initialize_track_from_lidar_measurement(lidar_measurements[measurement_index]));
        }
    }

    CovarianceMatrix make_lidar_birth_covariance() const
    {
        CovarianceMatrix P0 = CovarianceMatrix::Zero();
        P0(0, 0) = config_.sigma_birth_lidar_px * config_.sigma_birth_lidar_px;
        P0(1, 1) = config_.sigma_birth_lidar_py * config_.sigma_birth_lidar_py;
        P0(2, 2) = config_.sigma_birth_lidar_vx * config_.sigma_birth_lidar_vx;
        P0(3, 3) = config_.sigma_birth_lidar_vy * config_.sigma_birth_lidar_vy;
        P0(4, 4) = config_.sigma_birth_lidar_length * config_.sigma_birth_lidar_length;
        P0(5, 5) = config_.sigma_birth_lidar_width * config_.sigma_birth_lidar_width;
        return P0;
    }

    CovarianceMatrix make_stereo_birth_covariance() const
    {
        CovarianceMatrix P0 = CovarianceMatrix::Zero();
        P0(0, 0) = config_.sigma_birth_stereo_px * config_.sigma_birth_stereo_px;
        P0(1, 1) = config_.sigma_birth_stereo_py * config_.sigma_birth_stereo_py;
        P0(2, 2) = config_.sigma_birth_stereo_vx * config_.sigma_birth_stereo_vx;
        P0(3, 3) = config_.sigma_birth_stereo_vy * config_.sigma_birth_stereo_vy;
        P0(4, 4) = config_.sigma_birth_stereo_length * config_.sigma_birth_stereo_length;
        P0(5, 5) = config_.sigma_birth_stereo_width * config_.sigma_birth_stereo_width;
        return P0;
    }

    Track initialize_track_from_lidar_measurement(const LidarMeasurement &measurement)
    {
        Track track;
        track.track_id = next_track_id_++;
        track.x << measurement.px_m, measurement.py_m, 0.0, 0.0, measurement.length_m, measurement.width_m;
        track.P = make_lidar_birth_covariance();
        track.pz_m = measurement.pz_m;
        track.height_m = measurement.height_m;
        track.yaw_rad = measurement.yaw_rad;
        track.existence_probability = config_.p_init_lidar;
        track.confirmed = track.existence_probability >= config_.confirmation_threshold;
        track.lidar_supported_this_iteration = true;
        track.last_predict_time = measurement.stamp;
        track.last_update_time = measurement.stamp;
        return track;
    }

    Track initialize_track_from_stereo_measurement(const StereoMeasurement &measurement)
    {
        Track track;
        track.track_id = next_track_id_++;
        // Stereo is useful for seeding position, but LiDAR remains the primary source
        // of tracked footprint dimensions once available.
        track.x << measurement.px_m,
            measurement.py_m,
            0.0,
            0.0,
            config_.stereo_birth_default_length_m,
            config_.stereo_birth_default_width_m;
        track.P = make_stereo_birth_covariance();
        track.pz_m = measurement.pz_m;
        track.height_m = measurement.height_m;
        track.yaw_rad = 0.0;
        track.existence_probability = config_.p_init_stereo;
        track.confirmed = track.existence_probability >= config_.confirmation_threshold;
        track.stereo_supported_this_iteration = true;
        track.classification = measurement.classification;
        track.last_predict_time = measurement.stamp;
        track.last_update_time = measurement.stamp;
        return track;
    }

    void birth_tracks_from_lidar_measurements(
        const std::vector<LidarMeasurement> &lidar_measurements)
    {
        tracks_.reserve(tracks_.size() + lidar_measurements.size());
        for (const auto &measurement : lidar_measurements)
        {
            tracks_.push_back(initialize_track_from_lidar_measurement(measurement));
        }
    }

    void birth_tracks_from_stereo_measurements(
        const std::vector<StereoMeasurement> &stereo_measurements)
    {
        tracks_.reserve(tracks_.size() + stereo_measurements.size());
        for (const auto &measurement : stereo_measurements)
        {
            tracks_.push_back(initialize_track_from_stereo_measurement(measurement));
        }
    }

    std_msgs::msg::Header make_output_header(const rclcpp::Time &stamp) const
    {
        std_msgs::msg::Header header;
        header.stamp = stamp;
        header.frame_id = config_.tracking_frame;
        return header;
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
                rclcpp::Duration::from_seconds(config_.tf_lookup_timeout_ms / 1000.0));
            tf2::Transform transform;
            tf2::fromMsg(transform_stamped.transform, transform);
            return transform;
        }
        catch (const tf2::TransformException &exception)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "TF lookup failed from %s to %s: %s",
                source_frame.c_str(),
                target_frame.c_str(),
                exception.what());
            return std::nullopt;
        }
    }

    std::vector<LidarMeasurement> make_lidar_measurements(
        const vision_msgs::msg::Detection3DArray &lidar_detections,
        const tf2::Transform &lidar_to_tracking_tf)
    {
        std::vector<LidarMeasurement> lidar_measurements;
        lidar_measurements.reserve(lidar_detections.detections.size());

        for (std::size_t detection_index = 0; detection_index < lidar_detections.detections.size(); ++detection_index)
        {
            const auto &detection = lidar_detections.detections[detection_index];
            const tf2::Vector3 raw_center(
                detection.bbox.center.position.x,
                detection.bbox.center.position.y,
                detection.bbox.center.position.z);
            const tf2::Vector3 transformed_center = lidar_to_tracking_tf * raw_center;
            tf2::Quaternion raw_orientation;
            tf2::fromMsg(detection.bbox.center.orientation, raw_orientation);
            const tf2::Quaternion transformed_orientation =
                lidar_to_tracking_tf.getRotation() * raw_orientation;

            LidarMeasurement measurement;
            measurement.detection_index = detection_index;
            measurement.stamp = lidar_detections.header.stamp;
            measurement.px_m = transformed_center.x();
            measurement.py_m = transformed_center.y();
            measurement.pz_m = transformed_center.z();
            measurement.length_m = detection.bbox.size.x;
            measurement.width_m = detection.bbox.size.y;
            measurement.height_m = detection.bbox.size.z;
            measurement.yaw_rad = yaw_from_quaternion(tf2::toMsg(transformed_orientation));
            lidar_measurements.push_back(measurement);
        }

        return lidar_measurements;
    }

    std::vector<StereoMeasurement> make_stereo_measurements(
        const vision_msgs::msg::Detection3DArray &stereo_detections,
        const tf2::Transform &stereo_to_tracking_tf)
    {
        std::vector<StereoMeasurement> stereo_measurements;
        stereo_measurements.reserve(stereo_detections.detections.size());

        for (std::size_t detection_index = 0; detection_index < stereo_detections.detections.size(); ++detection_index)
        {
            const auto &detection = stereo_detections.detections[detection_index];
            const tf2::Vector3 raw_center(
                detection.bbox.center.position.x,
                detection.bbox.center.position.y,
                detection.bbox.center.position.z);
            const tf2::Vector3 transformed_center = stereo_to_tracking_tf * raw_center;

            StereoMeasurement measurement;
            measurement.detection_index = detection_index;
            measurement.stamp = stereo_detections.header.stamp;
            measurement.px_m = transformed_center.x();
            measurement.py_m = transformed_center.y();
            measurement.pz_m = transformed_center.z();
            measurement.length_m = detection.bbox.size.x;
            measurement.width_m = detection.bbox.size.y;
            measurement.height_m = detection.bbox.size.z;

            if (!detection.results.empty())
            {
                const auto &hypothesis = detection.results.front().hypothesis;
                measurement.classification = class_id_to_label(hypothesis.class_id);
                measurement.confidence = hypothesis.score;
            }

            stereo_measurements.push_back(measurement);
        }

        return stereo_measurements;
    }

    void lidar_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        lidar_buffer_.push_back({msg, std::chrono::steady_clock::now()});
        while (static_cast<int>(lidar_buffer_.size()) > config_.lidar_buffer_size)
        {
            lidar_buffer_.pop_front();
        }
    }

    void stereo_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        stereo_buffer_.push_back({msg, std::chrono::steady_clock::now()});
        while (static_cast<int>(stereo_buffer_.size()) > config_.stereo_buffer_size)
        {
            stereo_buffer_.pop_front();
        }
    }

    void prune_stale_batches(
        std::deque<BufferedDetectionBatch> &buffer,
        const std::chrono::steady_clock::time_point &now_steady) const
    {
        while (!buffer.empty())
        {
            if (age_ms_from_now(buffer.front().received_steady, now_steady) <= config_.max_measurement_age_ms)
            {
                break;
            }
            buffer.pop_front();
        }
    }

    IterationInputs consume_latest_inputs()
    {
        IterationInputs inputs;
        const auto now_steady = std::chrono::steady_clock::now();

        std::lock_guard<std::mutex> lock(buffer_mutex_);
        prune_stale_batches(lidar_buffer_, now_steady);
        prune_stale_batches(stereo_buffer_, now_steady);

        if (!lidar_buffer_.empty())
        {
            inputs.lidar_batch = lidar_buffer_.back().msg;
            lidar_buffer_.clear();
        }
        if (!stereo_buffer_.empty())
        {
            inputs.stereo_batch = stereo_buffer_.back().msg;
            stereo_buffer_.clear();
        }

        return inputs;
    }

    void publish_debug_markers(const std_msgs::msg::Header &header)
    {
        if (!tracked_markers_publisher_)
        {
            return;
        }

        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header = header;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        for (const auto &track : tracks_)
        {
            visualization_msgs::msg::Marker box_marker;
            box_marker.header = header;
            box_marker.ns = "tracked_boxes";
            box_marker.id = static_cast<int>(track.track_id);
            box_marker.type = visualization_msgs::msg::Marker::CUBE;
            box_marker.action = visualization_msgs::msg::Marker::ADD;
            box_marker.pose.position.x = track.x(0);
            box_marker.pose.position.y = track.x(1);
            box_marker.pose.position.z = track.pz_m;
            box_marker.pose.orientation = quaternion_from_yaw(track.yaw_rad);
            box_marker.scale.x = std::max(0.05, track.x(4));
            box_marker.scale.y = std::max(0.05, track.x(5));
            box_marker.scale.z = std::max(0.05, track.height_m);
            box_marker.color.r = 0.95F;
            box_marker.color.g = 0.20F;
            box_marker.color.b = 0.20F;
            box_marker.color.a = 0.30F;
            box_marker.lifetime = rclcpp::Duration::from_seconds(config_.marker_lifetime_sec);
            marker_array.markers.push_back(box_marker);

            visualization_msgs::msg::Marker text_marker;
            text_marker.header = header;
            text_marker.ns = "tracked_labels";
            text_marker.id = static_cast<int>(track.track_id + 10000U);
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            text_marker.pose.position.x = track.x(0);
            text_marker.pose.position.y = track.x(1);
            text_marker.pose.position.z = track.pz_m + (0.6 * std::max(0.05, track.height_m));
            text_marker.scale.z = 0.45;
            text_marker.color.r = 1.0F;
            text_marker.color.g = 1.0F;
            text_marker.color.b = 1.0F;
            text_marker.color.a = 0.95F;
            text_marker.text =
                std::to_string(track.track_id) + " p=" +
                std::to_string(track.existence_probability).substr(0, 4) +
                (track.confirmed ? " C" : " T");
            text_marker.lifetime = rclcpp::Duration::from_seconds(config_.marker_lifetime_sec);
            marker_array.markers.push_back(text_marker);
        }

        tracked_markers_publisher_->publish(marker_array);
    }

    void apply_lifecycle(const bool sensor_frame_received)
    {
        if (!sensor_frame_received)
        {
            return;
        }

        for (auto &track : tracks_)
        {
            const bool supported =
                track.lidar_supported_this_iteration || track.stereo_supported_this_iteration;
            if (!supported)
            {
                track.existence_probability =
                    std::clamp(track.existence_probability - config_.miss_decay, 0.0, 1.0);
            }
            track.confirmed = track.existence_probability >= config_.confirmation_threshold;
        }

        tracks_.erase(
            std::remove_if(
                tracks_.begin(),
                tracks_.end(),
                [this](const Track &track) {
                    return track.existence_probability <= config_.deletion_threshold;
                }),
            tracks_.end());
    }

    auto_stack_msgs::msg::TrackedObjectArray build_tracked_objects_message(
        const std_msgs::msg::Header &header) const
    {
        auto_stack_msgs::msg::TrackedObjectArray tracked_objects_msg;
        tracked_objects_msg.header = header;

        for (const auto &track : tracks_)
        {
            if (!track.confirmed)
            {
                continue;
            }

            auto_stack_msgs::msg::TrackedObject tracked_object;
            tracked_object.header = tracked_objects_msg.header;
            tracked_object.track_id = track.track_id;
            tracked_object.pose.position.x = track.x(0);
            tracked_object.pose.position.y = track.x(1);
            tracked_object.pose.position.z = track.pz_m;
            tracked_object.pose.orientation = quaternion_from_yaw(track.yaw_rad);
            tracked_object.dimensions.x = track.x(4);
            tracked_object.dimensions.y = track.x(5);
            tracked_object.dimensions.z = track.height_m;
            tracked_object.velocity.x = track.x(2);
            tracked_object.velocity.y = track.x(3);
            tracked_object.velocity.z = 0.0;
            tracked_object.existence_probability =
                static_cast<float>(track.existence_probability);
            tracked_object.classification = track.classification;
            tracked_object.from_lidar = track.lidar_supported_this_iteration;
            tracked_object.from_camera = track.stereo_supported_this_iteration;
            tracked_objects_msg.objects.push_back(tracked_object);
        }

        return tracked_objects_msg;
    }

    void publish_tracked_objects(const std_msgs::msg::Header &header)
    {
        if (!tracked_objects_publisher_)
        {
            return;
        }
        tracked_objects_publisher_->publish(build_tracked_objects_message(header));
    }

    void run_tracker_iteration()
    {
        const auto iteration_stamp = now();
        const auto output_header = make_output_header(iteration_stamp);
        const auto inputs = consume_latest_inputs();
        const bool sensor_frame_received = static_cast<bool>(inputs.lidar_batch) ||
                                           static_cast<bool>(inputs.stereo_batch);
        std::vector<LidarMeasurement> lidar_measurements;
        if (inputs.lidar_batch)
        {
            const auto lidar_to_tracking_tf = lookup_transform(
                config_.tracking_frame,
                inputs.lidar_batch->header.frame_id,
                inputs.lidar_batch->header.stamp);
            if (lidar_to_tracking_tf)
            {
                lidar_measurements =
                    make_lidar_measurements(*inputs.lidar_batch, *lidar_to_tracking_tf);
            }
        }

        std::vector<StereoMeasurement> stereo_measurements;
        if (inputs.stereo_batch)
        {
            const auto stereo_to_tracking_tf = lookup_transform(
                config_.tracking_frame,
                inputs.stereo_batch->header.frame_id,
                inputs.stereo_batch->header.stamp);
            if (stereo_to_tracking_tf)
            {
                stereo_measurements =
                    make_stereo_measurements(*inputs.stereo_batch, *stereo_to_tracking_tf);
            }
        }

        prepare_tracks_for_iteration();
        predict_tracks(iteration_stamp);
        bool birthed_from_stereo_only = false;

        if (!lidar_measurements.empty())
        {
            const auto lidar_association_result =
                associate_lidar_measurements_to_tracks(lidar_measurements);
            for (const auto &[track_index, measurement_index] : lidar_association_result.matches)
            {
                apply_lidar_update(tracks_[track_index], lidar_measurements[measurement_index]);
            }
            birth_unmatched_lidar_tracks(
                lidar_measurements,
                lidar_association_result.unmatched_measurement_indices);
        }
        else if (tracks_.empty() && !stereo_measurements.empty())
        {
            birth_tracks_from_stereo_measurements(stereo_measurements);
            birthed_from_stereo_only = true;
        }

        if (!birthed_from_stereo_only && !stereo_measurements.empty() && !tracks_.empty())
        {
            const auto stereo_association_result =
                associate_stereo_measurements_to_tracks(stereo_measurements);
            for (const auto &[track_index, measurement_index] : stereo_association_result.matches)
            {
                apply_stereo_update(tracks_[track_index], stereo_measurements[measurement_index]);
            }
        }

        apply_lifecycle(sensor_frame_received);
        publish_tracked_objects(output_header);

        RCLCPP_DEBUG(
            get_logger(),
            "Tracker iteration at %.3f sec: lidar_batch=%s lidar_meas=%zu stereo_batch=%s stereo_meas=%zu tracks=%zu",
            iteration_stamp.seconds(),
            inputs.lidar_batch ? "yes" : "no",
            lidar_measurements.size(),
            inputs.stereo_batch ? "yes" : "no",
            stereo_measurements.size(),
            tracks_.size());

        // The node now runs the full V1 tracking loop: transform, predict,
        // sequential sensor updates, lifecycle, and output publishing.
        publish_debug_markers(output_header);
    }

    TrackerConfig config_{};
    std::vector<Track> tracks_{};
    std::uint32_t next_track_id_{1};

    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr lidar_subscription_;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr stereo_subscription_;
    rclcpp::Publisher<auto_stack_msgs::msg::TrackedObjectArray>::SharedPtr tracked_objects_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tracked_markers_publisher_;
    rclcpp::TimerBase::SharedPtr tracker_timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::deque<BufferedDetectionBatch> lidar_buffer_;
    std::deque<BufferedDetectionBatch> stereo_buffer_;
    std::mutex buffer_mutex_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EkfMultiObjectTracker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
