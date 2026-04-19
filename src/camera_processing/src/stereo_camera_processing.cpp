#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
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

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <camera_processing/yolo_detector.hpp>

namespace
{
struct ScopedTimer
{
    std::chrono::steady_clock::time_point start_time_;
    double &duration_ms_;

    explicit ScopedTimer(double &duration_ms)
        : start_time_(std::chrono::steady_clock::now()),
          duration_ms_(duration_ms)
    {
    }

    ~ScopedTimer()
    {
        const auto end_time = std::chrono::steady_clock::now();
        duration_ms_ = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                           end_time - start_time_)
                           .count();
    }
};

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

double time_delta_ms(
    const builtin_interfaces::msg::Time &lhs,
    const builtin_interfaces::msg::Time &rhs)
{
    return std::abs((rclcpp::Time(lhs) - rclcpp::Time(rhs)).seconds()) * 1000.0;
}

bool starts_with(const std::string &value, const std::string &prefix)
{
    return value.rfind(prefix, 0) == 0;
}

std::string normalize_dataset_sequence(std::string sequence)
{
    sequence.erase(
        std::remove_if(
            sequence.begin(),
            sequence.end(),
            [](unsigned char ch)
            {
                return std::isspace(ch) != 0;
            }),
        sequence.end());

    if (sequence.empty())
    {
        return sequence;
    }

    const bool numeric = std::all_of(
        sequence.begin(),
        sequence.end(),
        [](unsigned char ch)
        {
            return std::isdigit(ch) != 0;
        });
    if (!numeric)
    {
        return sequence;
    }

    while (sequence.size() < 2)
    {
        sequence.insert(sequence.begin(), '0');
    }
    return sequence;
}

std::string projection_label_for_camera_name(std::string camera_name)
{
    std::transform(
        camera_name.begin(),
        camera_name.end(),
        camera_name.begin(),
        [](unsigned char ch)
        {
            return static_cast<char>(std::toupper(ch));
        });
    return camera_name + ":";
}

std::string class_id_to_label(const int class_id)
{
    switch (class_id)
    {
    case 0:
        return "person";
    case 1:
        return "bicycle";
    case 2:
        return "car";
    case 3:
        return "motorcycle";
    case 5:
        return "bus";
    case 7:
        return "truck";
    default:
        return "class_" + std::to_string(class_id);
    }
}

struct ObjectDimensions
{
    double length_m{1.0};
    double width_m{1.0};
    double height_m{1.0};
};

ObjectDimensions class_dimensions_for_id(const int class_id)
{
    switch (class_id)
    {
    case 0:
        return {0.8, 0.6, 1.7};
    case 1:
        return {1.8, 0.6, 1.5};
    case 2:
        return {4.3, 1.8, 1.5};
    case 3:
        return {2.1, 0.8, 1.4};
    case 5:
        return {12.0, 2.6, 3.2};
    case 7:
        return {6.5, 2.5, 3.0};
    default:
        return {};
    }
}

double clamp_with_reference(const double value, const double reference)
{
    return std::clamp(value, 0.5 * reference, 1.5 * reference);
}
}  // namespace

class StereoCameraProcessing final : public rclcpp::Node
{
public:
    static constexpr std::size_t kSensorImageQosDepth{5};
    static constexpr std::size_t kSemanticOutputQosDepth{5};

    StereoCameraProcessing()
        : Node("camera_processing")
    {
        declare_parameter<double>("processing_rate", 10.0);
        declare_parameter<std::string>("model_path", "models/yolo/yolov8n.onnx");
        declare_parameter<bool>("publish_overlay_image", false);
        declare_parameter<bool>("publish_camera_info", false);
        declare_parameter<bool>("publish_detection_markers", false);
        declare_parameter<std::string>("detection_markers_topic", "camera_stereo_detection_markers");
        declare_parameter<int>("profiling_interval_frames", 60);
        declare_parameter<bool>("enable_csv_logging", false);
        declare_parameter<std::string>("csv_log_dir", "csv_logs/camera_processing");
        declare_parameter<std::string>("dataset_path", "");
        declare_parameter<std::string>("dataset_sequence", "unknown");
        declare_parameter<std::string>("left_camera_name", "p2");
        declare_parameter<std::string>("right_camera_name", "p3");
        declare_parameter<int>("input_queue_size", 4);
        declare_parameter<double>("max_buffer_age_ms", 300.0);
        declare_parameter<double>("stereo_sync_tolerance_ms", 50.0);
        declare_parameter<int>("stereo_num_disparities", 128);
        declare_parameter<int>("stereo_block_size", 5);
        declare_parameter<int>("stereo_uniqueness_ratio", 10);
        declare_parameter<int>("stereo_speckle_window_size", 100);
        declare_parameter<int>("stereo_speckle_range", 2);
        declare_parameter<double>("min_valid_disparity_px", 1.0);
        declare_parameter<double>("stereo_min_depth_m", 1.0);
        declare_parameter<double>("stereo_max_depth_m", 80.0);
        declare_parameter<int>("roi_sampling_border_px", 6);

        processing_rate_ = get_parameter("processing_rate").as_double();
        model_path_ = get_parameter("model_path").as_string();
        publish_overlay_image_ = get_parameter("publish_overlay_image").as_bool();
        publish_camera_info_ = get_parameter("publish_camera_info").as_bool();
        publish_detection_markers_ = get_parameter("publish_detection_markers").as_bool();
        detection_markers_topic_ = get_parameter("detection_markers_topic").as_string();
        profiling_interval_frames_ =
            static_cast<int>(get_parameter("profiling_interval_frames").as_int());
        csv_logging_ = get_parameter("enable_csv_logging").as_bool();
        csv_log_dir_ = get_parameter("csv_log_dir").as_string();
        dataset_path_ = get_parameter("dataset_path").as_string();
        dataset_sequence_ = get_parameter("dataset_sequence").as_string();
        left_camera_name_ = get_parameter("left_camera_name").as_string();
        right_camera_name_ = get_parameter("right_camera_name").as_string();
        input_queue_size_ = std::max(1, static_cast<int>(get_parameter("input_queue_size").as_int()));
        max_buffer_age_ms_ = std::max(0.0, get_parameter("max_buffer_age_ms").as_double());
        stereo_sync_tolerance_ms_ =
            std::max(0.0, get_parameter("stereo_sync_tolerance_ms").as_double());
        stereo_num_disparities_ =
            std::max(16, static_cast<int>(get_parameter("stereo_num_disparities").as_int()));
        if ((stereo_num_disparities_ % 16) != 0)
        {
            stereo_num_disparities_ += 16 - (stereo_num_disparities_ % 16);
        }
        stereo_block_size_ =
            std::max(3, static_cast<int>(get_parameter("stereo_block_size").as_int()));
        if ((stereo_block_size_ % 2) == 0)
        {
            stereo_block_size_ += 1;
        }
        stereo_uniqueness_ratio_ =
            std::max(0, static_cast<int>(get_parameter("stereo_uniqueness_ratio").as_int()));
        stereo_speckle_window_size_ =
            std::max(0, static_cast<int>(get_parameter("stereo_speckle_window_size").as_int()));
        stereo_speckle_range_ =
            std::max(0, static_cast<int>(get_parameter("stereo_speckle_range").as_int()));
        min_valid_disparity_px_ =
            std::max(0.0, get_parameter("min_valid_disparity_px").as_double());
        stereo_min_depth_m_ = std::max(0.1, get_parameter("stereo_min_depth_m").as_double());
        stereo_max_depth_m_ =
            std::max(stereo_min_depth_m_, get_parameter("stereo_max_depth_m").as_double());
        roi_sampling_border_px_ =
            std::max(0, static_cast<int>(get_parameter("roi_sampling_border_px").as_int()));

        const auto camera_image_qos =
            rclcpp::SensorDataQoS().keep_last(kSensorImageQosDepth);
        const auto semantic_output_qos =
            rclcpp::QoS(rclcpp::KeepLast(kSemanticOutputQosDepth))
                .reliable()
                .durability_volatile();

        left_image_subscription_ =
            create_subscription<sensor_msgs::msg::Image>(
                "left_camera_in",
                camera_image_qos,
                std::bind(
                    &StereoCameraProcessing::left_image_callback,
                    this,
                    std::placeholders::_1));
        right_image_subscription_ =
            create_subscription<sensor_msgs::msg::Image>(
                "right_camera_in",
                camera_image_qos,
                std::bind(
                    &StereoCameraProcessing::right_image_callback,
                    this,
                    std::placeholders::_1));

        object_bbox_publisher_ =
            create_publisher<vision_msgs::msg::Detection2DArray>(
                "object_detections",
                semantic_output_qos);
        stereo_detection_publisher_ =
            create_publisher<vision_msgs::msg::Detection3DArray>(
                "camera_stereo_detections",
                semantic_output_qos);
        if (publish_overlay_image_)
        {
            overlay_image_publisher_ =
                create_publisher<sensor_msgs::msg::Image>("overlay_image", semantic_output_qos);
        }
        if (publish_camera_info_)
        {
            left_camera_info_publisher_ =
                create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", semantic_output_qos);
            right_camera_info_publisher_ =
                create_publisher<sensor_msgs::msg::CameraInfo>("right_camera_info", semantic_output_qos);
        }
        if (publish_detection_markers_)
        {
            stereo_detection_markers_publisher_ =
                create_publisher<visualization_msgs::msg::MarkerArray>(
                    detection_markers_topic_,
                    semantic_output_qos);
        }

        yolo_detector_ = std::make_unique<YoloDetector>(model_path_);
        stereo_matcher_ = cv::StereoSGBM::create(
            0,
            stereo_num_disparities_,
            stereo_block_size_);
        stereo_matcher_->setP1(8 * stereo_block_size_ * stereo_block_size_);
        stereo_matcher_->setP2(32 * stereo_block_size_ * stereo_block_size_);
        stereo_matcher_->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
        stereo_matcher_->setUniquenessRatio(stereo_uniqueness_ratio_);
        stereo_matcher_->setSpeckleWindowSize(stereo_speckle_window_size_);
        stereo_matcher_->setSpeckleRange(stereo_speckle_range_);
        stereo_matcher_->setDisp12MaxDiff(1);

        const bool calibration_loaded = load_stereo_calibration();
        if (!calibration_loaded)
        {
            RCLCPP_WARN(
                get_logger(),
                "Stereo calibration was not loaded. 2D detections will still publish, but 3D stereo detections will be empty.");
        }

        processing_timer_ = create_wall_timer(
            processing_rate_ > 0.0
                ? std::chrono::milliseconds(static_cast<int>(1000.0 / processing_rate_))
                : std::chrono::milliseconds(100),
            std::bind(&StereoCameraProcessing::process_latest_stereo_pair, this));
        initialize_csv_logging();
        reset_resource_window();

        RCLCPP_INFO(
            get_logger(),
            "Stereo camera processing initialized for %s/%s.",
            left_camera_name_.c_str(),
            right_camera_name_.c_str());
    }

private:
    struct FrameProcessingMetrics
    {
        double buffer_age_ms{0.0};
        double stereo_skew_ms{0.0};
        double conversion_time_ms{0.0};
        double inference_time_ms{0.0};
        double disparity_time_ms{0.0};
        double publish_time_ms{0.0};
        double overlay_time_ms{0.0};
        double frame_total_time_ms{0.0};
        double rss_mb{0.0};
        double peak_rss_mb{0.0};
        std::size_t num_2d_detections{0};
        std::size_t num_3d_detections{0};
    };

    struct BufferedImage
    {
        sensor_msgs::msg::Image::SharedPtr msg;
        std::chrono::steady_clock::time_point received_steady;
    };

    struct StereoPair
    {
        sensor_msgs::msg::Image::SharedPtr left_image;
        sensor_msgs::msg::Image::SharedPtr right_image;
        std::chrono::steady_clock::time_point left_received_steady;
        std::chrono::steady_clock::time_point right_received_steady;
        double skew_ms{0.0};
    };

    struct StereoCalibration
    {
        bool valid{false};
        std::array<double, 12> left_projection{};
        std::array<double, 12> right_projection{};
        sensor_msgs::msg::CameraInfo left_camera_info;
        sensor_msgs::msg::CameraInfo right_camera_info;
        double left_fx{0.0};
        double left_fy{0.0};
        double left_cx{0.0};
        double left_cy{0.0};
        double baseline_m{0.0};
    };

    double processing_rate_{10.0};
    std::string model_path_;
    bool publish_overlay_image_{false};
    bool publish_camera_info_{false};
    bool publish_detection_markers_{false};
    std::string dataset_path_;
    std::string dataset_sequence_{"unknown"};
    std::string detection_markers_topic_{"camera_stereo_detection_markers"};
    std::string left_camera_name_{"p2"};
    std::string right_camera_name_{"p3"};
    int input_queue_size_{4};
    double max_buffer_age_ms_{300.0};
    double stereo_sync_tolerance_ms_{50.0};
    int stereo_num_disparities_{128};
    int stereo_block_size_{5};
    int stereo_uniqueness_ratio_{10};
    int stereo_speckle_window_size_{100};
    int stereo_speckle_range_{2};
    double min_valid_disparity_px_{1.0};
    double stereo_min_depth_m_{1.0};
    double stereo_max_depth_m_{80.0};
    int roi_sampling_border_px_{6};
    int profiling_interval_frames_{60};
    bool csv_logging_{false};
    std::string csv_log_dir_{"csv_logs/camera_processing"};
    std::string csv_log_file_path_;
    int interval_frame_count_{0};
    double interval_sum_buffer_age_ms_{0.0};
    double interval_sum_stereo_skew_ms_{0.0};
    double interval_sum_conversion_ms_{0.0};
    double interval_sum_inference_ms_{0.0};
    double interval_sum_disparity_ms_{0.0};
    double interval_sum_publish_ms_{0.0};
    double interval_sum_overlay_ms_{0.0};
    double interval_sum_frame_total_ms_{0.0};
    double interval_sum_rss_mb_{0.0};
    double interval_sum_peak_rss_mb_{0.0};
    double interval_sum_num_2d_detections_{0.0};
    double interval_sum_num_3d_detections_{0.0};
    std::chrono::steady_clock::time_point interval_resource_window_start_;
    double interval_resource_window_cpu_ms_{0.0};
    std::ofstream csv_log_stream_;
    std::uint64_t total_received_left_frames_{0};
    std::uint64_t total_received_right_frames_{0};
    std::uint64_t total_processed_pairs_{0};
    std::uint64_t total_overwritten_left_frames_{0};
    std::uint64_t total_overwritten_right_frames_{0};
    std::uint64_t total_sync_drop_frames_{0};

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_image_subscription_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr object_bbox_publisher_;
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr stereo_detection_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_camera_info_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_camera_info_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr stereo_detection_markers_publisher_;
    rclcpp::TimerBase::SharedPtr processing_timer_;

    std::mutex image_stream_mutex_;
    std::deque<BufferedImage> left_image_queue_;
    std::deque<BufferedImage> right_image_queue_;

    std::unique_ptr<YoloDetector> yolo_detector_;
    cv::Ptr<cv::StereoSGBM> stereo_matcher_;
    StereoCalibration stereo_calibration_;

    void left_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        total_received_left_frames_ += 1;
        push_image(left_image_queue_, std::move(msg));
    }

    void right_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        total_received_right_frames_ += 1;
        push_image(right_image_queue_, std::move(msg));
    }

    void push_image(
        std::deque<BufferedImage> &queue,
        sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(image_stream_mutex_);
        if (static_cast<int>(queue.size()) >= input_queue_size_)
        {
            queue.pop_front();
            if (&queue == &left_image_queue_)
            {
                total_overwritten_left_frames_ += 1;
            }
            else if (&queue == &right_image_queue_)
            {
                total_overwritten_right_frames_ += 1;
            }
        }
        queue.push_back(BufferedImage{std::move(msg), std::chrono::steady_clock::now()});
    }

    void process_latest_stereo_pair()
    {
        FrameProcessingMetrics metrics;
        {
            ScopedTimer total_frame_timer(metrics.frame_total_time_ms);
            const auto stereo_pair = try_pop_synced_pair();
            if (!stereo_pair)
            {
                return;
            }

            const auto now_steady = std::chrono::steady_clock::now();
            const double left_age_ms =
                std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                    now_steady - stereo_pair->left_received_steady)
                    .count();
            const double right_age_ms =
                std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                    now_steady - stereo_pair->right_received_steady)
                    .count();
            metrics.buffer_age_ms = std::max(left_age_ms, right_age_ms);
            metrics.stereo_skew_ms = stereo_pair->skew_ms;

            cv::Mat left_image;
            cv::Mat right_image;
            try
            {
                ScopedTimer conversion_timer(metrics.conversion_time_ms);
                left_image = cv_bridge::toCvCopy(
                                 stereo_pair->left_image,
                                 sensor_msgs::image_encodings::BGR8)
                                 ->image;
                right_image = cv_bridge::toCvCopy(
                                  stereo_pair->right_image,
                                  sensor_msgs::image_encodings::BGR8)
                                  ->image;
            }
            catch (const cv_bridge::Exception &exception)
            {
                RCLCPP_ERROR(get_logger(), "cv_bridge conversion failed: %s", exception.what());
                return;
            }

            if (left_image.empty() || right_image.empty())
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Stereo frame conversion produced an empty image.");
                return;
            }

            if (left_image.size() != right_image.size())
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Left/right stereo image sizes do not match.");
                return;
            }

            std::vector<YoloDetectionResult> detections;
            {
                ScopedTimer inference_timer(metrics.inference_time_ms);
                detections = yolo_detector_->infer(left_image);
            }
            metrics.num_2d_detections = detections.size();

            cv::Mat disparity_pixels;
            if (stereo_calibration_.valid)
            {
                ScopedTimer disparity_timer(metrics.disparity_time_ms);
                disparity_pixels = compute_disparity_map(left_image, right_image);
            }

            {
                ScopedTimer publish_timer(metrics.publish_time_ms);
                publish_object_detections(detections, stereo_pair->left_image->header);
                publish_camera_info_messages(*stereo_pair->left_image, *stereo_pair->right_image);
                metrics.num_3d_detections = publish_stereo_detections(
                    detections,
                    stereo_pair->left_image->header,
                    disparity_pixels);
            }

            if (publish_overlay_image_ && overlay_image_publisher_)
            {
                ScopedTimer overlay_timer(metrics.overlay_time_ms);
                publish_overlay_image(
                    left_image,
                    detections,
                    stereo_pair->left_image->header,
                    disparity_pixels);
            }

            metrics.rss_mb = read_current_rss_mb();
            metrics.peak_rss_mb = read_peak_rss_mb();
        }

        total_processed_pairs_ += 1;
        update_profiling_metrics(metrics);
    }

    std::optional<StereoPair> try_pop_synced_pair()
    {
        std::lock_guard<std::mutex> lock(image_stream_mutex_);
        prune_stale_images(left_image_queue_);
        prune_stale_images(right_image_queue_);

        while (!left_image_queue_.empty() && !right_image_queue_.empty())
        {
            const auto &left_front = left_image_queue_.front();
            auto best_right_it = right_image_queue_.begin();
            double best_delta_ms =
                time_delta_ms(left_front.msg->header.stamp, best_right_it->msg->header.stamp);

            for (auto candidate_it = right_image_queue_.begin();
                 candidate_it != right_image_queue_.end();
                 ++candidate_it)
            {
                const double candidate_delta_ms =
                    time_delta_ms(left_front.msg->header.stamp, candidate_it->msg->header.stamp);
                if (candidate_delta_ms < best_delta_ms)
                {
                    best_delta_ms = candidate_delta_ms;
                    best_right_it = candidate_it;
                }
            }

            if (best_delta_ms <= stereo_sync_tolerance_ms_)
            {
                StereoPair stereo_pair;
                stereo_pair.left_image = left_front.msg;
                stereo_pair.right_image = best_right_it->msg;
                stereo_pair.left_received_steady = left_front.received_steady;
                stereo_pair.right_received_steady = best_right_it->received_steady;
                stereo_pair.skew_ms = best_delta_ms;
                left_image_queue_.pop_front();
                right_image_queue_.erase(best_right_it);
                return stereo_pair;
            }

            const auto left_time = rclcpp::Time(left_front.msg->header.stamp);
            const auto right_time = rclcpp::Time(right_image_queue_.front().msg->header.stamp);
            if (left_time <= right_time)
            {
                left_image_queue_.pop_front();
            }
            else
            {
                right_image_queue_.pop_front();
            }
            total_sync_drop_frames_ += 1;
        }

        return std::nullopt;
    }

    void prune_stale_images(std::deque<BufferedImage> &queue)
    {
        const auto now = std::chrono::steady_clock::now();
        while (!queue.empty())
        {
            const double age_ms =
                std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                    now - queue.front().received_steady)
                    .count();
            if (age_ms <= max_buffer_age_ms_)
            {
                break;
            }
            queue.pop_front();
            if (&queue == &left_image_queue_)
            {
                total_overwritten_left_frames_ += 1;
            }
            else if (&queue == &right_image_queue_)
            {
                total_overwritten_right_frames_ += 1;
            }
        }
    }

    void publish_object_detections(
        const std::vector<YoloDetectionResult> &detections,
        const std_msgs::msg::Header &image_header)
    {
        vision_msgs::msg::Detection2DArray detection_array;
        detection_array.header = image_header;
        detection_array.detections.reserve(detections.size());

        for (const auto &detection : detections)
        {
            vision_msgs::msg::Detection2D detection_msg;
            detection_msg.header = image_header;
            detection_msg.bbox.center.position.x =
                detection.bounding_box.x + (0.5 * detection.bounding_box.width);
            detection_msg.bbox.center.position.y =
                detection.bounding_box.y + (0.5 * detection.bounding_box.height);
            detection_msg.bbox.center.theta = 0.0;
            detection_msg.bbox.size_x = detection.bounding_box.width;
            detection_msg.bbox.size_y = detection.bounding_box.height;

            vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
            hypothesis.hypothesis.class_id = std::to_string(detection.class_id);
            hypothesis.hypothesis.score = detection.confidence;
            detection_msg.results.push_back(hypothesis);
            detection_array.detections.push_back(detection_msg);
        }

        object_bbox_publisher_->publish(detection_array);
    }

    void publish_camera_info_messages(
        const sensor_msgs::msg::Image &left_image,
        const sensor_msgs::msg::Image &right_image)
    {
        if (!publish_camera_info_ ||
            !left_camera_info_publisher_ ||
            !right_camera_info_publisher_ ||
            !stereo_calibration_.valid)
        {
            return;
        }

        auto left_info = stereo_calibration_.left_camera_info;
        left_info.header = left_image.header;
        left_info.width = left_image.width;
        left_info.height = left_image.height;
        left_camera_info_publisher_->publish(left_info);

        auto right_info = stereo_calibration_.right_camera_info;
        right_info.header = right_image.header;
        right_info.width = right_image.width;
        right_info.height = right_image.height;
        right_camera_info_publisher_->publish(right_info);
    }

    cv::Mat compute_disparity_map(
        const cv::Mat &left_image_bgr,
        const cv::Mat &right_image_bgr)
    {
        cv::Mat left_gray;
        cv::Mat right_gray;
        cv::cvtColor(left_image_bgr, left_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(right_image_bgr, right_gray, cv::COLOR_BGR2GRAY);

        cv::Mat disparity_raw;
        stereo_matcher_->compute(left_gray, right_gray, disparity_raw);

        cv::Mat disparity_pixels;
        disparity_raw.convertTo(disparity_pixels, CV_32F, 1.0 / 16.0);
        return disparity_pixels;
    }

    std::size_t publish_stereo_detections(
        const std::vector<YoloDetectionResult> &detections,
        const std_msgs::msg::Header &image_header,
        const cv::Mat &disparity_pixels)
    {
        vision_msgs::msg::Detection3DArray detection_array;
        detection_array.header = image_header;
        std::vector<int> source_indices;

        if (!stereo_calibration_.valid || disparity_pixels.empty())
        {
            stereo_detection_publisher_->publish(detection_array);
            publish_stereo_detection_markers(detection_array, source_indices);
            return 0U;
        }

        detection_array.detections.reserve(detections.size());
        source_indices.reserve(detections.size());
        for (std::size_t index = 0; index < detections.size(); ++index)
        {
            vision_msgs::msg::Detection3D detection_msg;
            if (!build_stereo_detection(
                    detections[index],
                    image_header,
                    disparity_pixels,
                    index,
                    detection_msg))
            {
                continue;
            }
            detection_array.detections.push_back(detection_msg);
            source_indices.push_back(static_cast<int>(index));
        }

        stereo_detection_publisher_->publish(detection_array);
        publish_stereo_detection_markers(detection_array, source_indices);
        return detection_array.detections.size();
    }

    void publish_stereo_detection_markers(
        const vision_msgs::msg::Detection3DArray &detection_array,
        const std::vector<int> &source_indices)
    {
        if (!publish_detection_markers_ || !stereo_detection_markers_publisher_)
        {
            return;
        }

        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header = detection_array.header;
        clear_marker.ns = "camera_stereo_detections";
        clear_marker.id = 0;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        for (std::size_t index = 0; index < detection_array.detections.size(); ++index)
        {
            const auto &detection = detection_array.detections[index];
            const int marker_base_id =
                static_cast<int>(index < source_indices.size() ? source_indices[index] : static_cast<int>(index));

            visualization_msgs::msg::Marker box_marker;
            box_marker.header = detection.header;
            box_marker.header.stamp = rclcpp::Time(0);
            box_marker.ns = "camera_stereo_boxes";
            box_marker.id = marker_base_id;
            box_marker.type = visualization_msgs::msg::Marker::CUBE;
            box_marker.action = visualization_msgs::msg::Marker::ADD;
            box_marker.pose = detection.bbox.center;
            box_marker.scale.x = std::max(0.05, detection.bbox.size.x);
            box_marker.scale.y = std::max(0.05, detection.bbox.size.y);
            box_marker.scale.z = std::max(0.05, detection.bbox.size.z);
            box_marker.color.r = 0.15F;
            box_marker.color.g = 0.85F;
            box_marker.color.b = 1.0F;
            box_marker.color.a = 0.35F;
            box_marker.lifetime = rclcpp::Duration::from_seconds(0.25);
            marker_array.markers.push_back(box_marker);

            visualization_msgs::msg::Marker text_marker;
            text_marker.header = detection.header;
            text_marker.header.stamp = rclcpp::Time(0);
            text_marker.ns = "camera_stereo_labels";
            text_marker.id = marker_base_id + 10000;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            text_marker.pose = detection.bbox.center;
            text_marker.pose.position.z += 0.7 * std::max(0.05, detection.bbox.size.z);
            text_marker.scale.z = 0.4;
            text_marker.color.r = 1.0F;
            text_marker.color.g = 1.0F;
            text_marker.color.b = 1.0F;
            text_marker.color.a = 0.95F;
            const std::string label =
                detection.results.empty()
                    ? "stereo"
                    : class_id_to_label(std::stoi(detection.results.front().hypothesis.class_id));
            text_marker.text =
                label + " " + cv::format("%.1fm", detection.bbox.center.position.z);
            text_marker.lifetime = rclcpp::Duration::from_seconds(0.25);
            marker_array.markers.push_back(text_marker);
        }

        stereo_detection_markers_publisher_->publish(marker_array);
    }

    bool build_stereo_detection(
        const YoloDetectionResult &detection,
        const std_msgs::msg::Header &image_header,
        const cv::Mat &disparity_pixels,
        const std::size_t detection_index,
        vision_msgs::msg::Detection3D &detection_msg) const
    {
        const cv::Rect image_bounds(0, 0, disparity_pixels.cols, disparity_pixels.rows);
        const cv::Rect raw_roi = detection.bounding_box & image_bounds;
        if (raw_roi.width < 4 || raw_roi.height < 4)
        {
            return false;
        }

        const int border_x = std::min(roi_sampling_border_px_, raw_roi.width / 4);
        const int border_y = std::min(roi_sampling_border_px_, raw_roi.height / 4);
        cv::Rect sampling_roi(
            raw_roi.x + border_x + (raw_roi.width / 4),
            raw_roi.y + border_y + (raw_roi.height / 2),
            std::max(1, raw_roi.width / 2),
            std::max(1, raw_roi.height / 3));
        sampling_roi &= image_bounds;
        if (sampling_roi.width < 1 || sampling_roi.height < 1)
        {
            sampling_roi = raw_roi;
        }

        std::vector<float> valid_disparities;
        valid_disparities.reserve(static_cast<std::size_t>(sampling_roi.area()));
        for (int row = sampling_roi.y; row < sampling_roi.y + sampling_roi.height; ++row)
        {
            for (int column = sampling_roi.x; column < sampling_roi.x + sampling_roi.width; ++column)
            {
                const float disparity = disparity_pixels.at<float>(row, column);
                if (std::isfinite(disparity) && disparity > static_cast<float>(min_valid_disparity_px_))
                {
                    valid_disparities.push_back(disparity);
                }
            }
        }

        if (valid_disparities.empty())
        {
            return false;
        }

        const auto median_it = valid_disparities.begin() + (valid_disparities.size() / 2);
        std::nth_element(valid_disparities.begin(), median_it, valid_disparities.end());
        const double disparity_px = static_cast<double>(*median_it);
        if (disparity_px <= min_valid_disparity_px_)
        {
            return false;
        }

        const double depth_m =
            (stereo_calibration_.left_fx * stereo_calibration_.baseline_m) / disparity_px;
        if (depth_m < stereo_min_depth_m_ || depth_m > stereo_max_depth_m_)
        {
            return false;
        }

        const ObjectDimensions prior_dimensions = class_dimensions_for_id(detection.class_id);
        const double projected_width_m =
            (static_cast<double>(raw_roi.width) * depth_m) / stereo_calibration_.left_fx;
        const double projected_height_m =
            (static_cast<double>(raw_roi.height) * depth_m) / stereo_calibration_.left_fy;
        const double width_m = clamp_with_reference(projected_width_m, prior_dimensions.width_m);
        const double height_m = clamp_with_reference(projected_height_m, prior_dimensions.height_m);
        const double length_m = prior_dimensions.length_m;

        const double center_u_px =
            static_cast<double>(raw_roi.x) + (0.5 * static_cast<double>(raw_roi.width));
        const double bottom_v_px =
            static_cast<double>(raw_roi.y + raw_roi.height);

        const double center_x_m =
            ((center_u_px - stereo_calibration_.left_cx) * depth_m) / stereo_calibration_.left_fx;
        const double bottom_y_m =
            ((bottom_v_px - stereo_calibration_.left_cy) * depth_m) / stereo_calibration_.left_fy;
        const double center_y_m = bottom_y_m - (0.5 * height_m);

        detection_msg.header = image_header;
        detection_msg.id = "stereo_detection_" + std::to_string(detection_index);
        detection_msg.bbox.center.position.x = center_x_m;
        detection_msg.bbox.center.position.y = center_y_m;
        detection_msg.bbox.center.position.z = depth_m;
        detection_msg.bbox.center.orientation.w = 1.0;
        detection_msg.bbox.size.x = length_m;
        detection_msg.bbox.size.y = width_m;
        detection_msg.bbox.size.z = height_m;

        vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
        hypothesis.hypothesis.class_id = std::to_string(detection.class_id);
        hypothesis.hypothesis.score = detection.confidence;
        detection_msg.results.push_back(hypothesis);
        return true;
    }

    void publish_overlay_image(
        const cv::Mat &left_image_bgr,
        const std::vector<YoloDetectionResult> &detections,
        const std_msgs::msg::Header &image_header,
        const cv::Mat &disparity_pixels)
    {
        cv::Mat overlay = left_image_bgr.clone();
        for (std::size_t index = 0; index < detections.size(); ++index)
        {
            const auto &detection = detections[index];
            cv::rectangle(overlay, detection.bounding_box, cv::Scalar(0, 255, 0), 2);

            std::string label =
                class_id_to_label(detection.class_id) + " " +
                cv::format("%.2f", detection.confidence);

            vision_msgs::msg::Detection3D detection_msg;
            if (!disparity_pixels.empty() &&
                build_stereo_detection(detection, image_header, disparity_pixels, index, detection_msg))
            {
                label += " " + cv::format("%.1fm", detection_msg.bbox.center.position.z);
            }

            int baseline = 0;
            const cv::Size label_size =
                cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
            const int label_top = std::max(detection.bounding_box.y, label_size.height);
            cv::rectangle(
                overlay,
                cv::Point(detection.bounding_box.x, label_top - label_size.height),
                cv::Point(detection.bounding_box.x + label_size.width, label_top + baseline),
                cv::Scalar(0, 255, 0),
                cv::FILLED);
            cv::putText(
                overlay,
                label,
                cv::Point(detection.bounding_box.x, label_top),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 0, 0),
                1);
        }

        try
        {
            std_msgs::msg::Header header = image_header;
            header.stamp = get_clock()->now();
            cv_bridge::CvImage overlay_bridge(
                header,
                sensor_msgs::image_encodings::BGR8,
                overlay);
            overlay_image_publisher_->publish(*overlay_bridge.toImageMsg());
        }
        catch (const cv_bridge::Exception &exception)
        {
            RCLCPP_ERROR(get_logger(), "Overlay image publication failed: %s", exception.what());
        }
    }

    bool load_stereo_calibration()
    {
        const auto calib_path = build_kitti_calibration_path();
        if (calib_path.empty())
        {
            RCLCPP_WARN(
                get_logger(),
                "Stereo calibration not loaded because dataset_path/dataset_sequence is not configured.");
            return false;
        }

        std::ifstream calib_stream(calib_path);
        if (!calib_stream.is_open())
        {
            RCLCPP_WARN(
                get_logger(),
                "Failed to open KITTI calibration file '%s'.",
                calib_path.c_str());
            return false;
        }

        const auto left_label = projection_label_for_camera_name(left_camera_name_);
        const auto right_label = projection_label_for_camera_name(right_camera_name_);
        std::optional<std::array<double, 12>> left_projection;
        std::optional<std::array<double, 12>> right_projection;

        std::string line;
        while (std::getline(calib_stream, line))
        {
            if (starts_with(line, left_label))
            {
                left_projection = parse_projection_row(line, left_label);
            }
            else if (starts_with(line, right_label))
            {
                right_projection = parse_projection_row(line, right_label);
            }
        }

        if (!left_projection || !right_projection)
        {
            RCLCPP_WARN(
                get_logger(),
                "Could not find both stereo projection rows '%s' and '%s' in '%s'.",
                left_label.c_str(),
                right_label.c_str(),
                calib_path.c_str());
            return false;
        }

        stereo_calibration_.left_projection = *left_projection;
        stereo_calibration_.right_projection = *right_projection;
        stereo_calibration_.left_fx = (*left_projection)[0];
        stereo_calibration_.left_fy = (*left_projection)[5];
        stereo_calibration_.left_cx = (*left_projection)[2];
        stereo_calibration_.left_cy = (*left_projection)[6];
        if (std::abs(stereo_calibration_.left_fx) < 1e-6 ||
            std::abs((*right_projection)[0]) < 1e-6)
        {
            RCLCPP_WARN(get_logger(), "Invalid stereo calibration focal length.");
            return false;
        }

        const double left_camera_x_m = -((*left_projection)[3] / (*left_projection)[0]);
        const double right_camera_x_m = -((*right_projection)[3] / (*right_projection)[0]);
        stereo_calibration_.baseline_m = std::abs(right_camera_x_m - left_camera_x_m);
        if (stereo_calibration_.baseline_m <= 1e-6)
        {
            RCLCPP_WARN(get_logger(), "Derived stereo baseline is too small.");
            return false;
        }

        stereo_calibration_.left_camera_info =
            make_camera_info_from_projection(*left_projection);
        stereo_calibration_.right_camera_info =
            make_camera_info_from_projection(*right_projection);
        stereo_calibration_.left_camera_info.header.frame_id = left_camera_name_;
        stereo_calibration_.right_camera_info.header.frame_id = right_camera_name_;
        stereo_calibration_.valid = true;

        RCLCPP_INFO(
            get_logger(),
            "Loaded stereo calibration from '%s' with baseline %.3f m.",
            calib_path.c_str(),
            stereo_calibration_.baseline_m);
        return true;
    }

    std::optional<std::array<double, 12>> parse_projection_row(
        const std::string &line,
        const std::string &label) const
    {
        std::istringstream line_stream(line.substr(label.size()));
        std::array<double, 12> projection{};
        for (double &value : projection)
        {
            if (!(line_stream >> value))
            {
                return std::nullopt;
            }
        }
        return projection;
    }

    sensor_msgs::msg::CameraInfo make_camera_info_from_projection(
        const std::array<double, 12> &projection) const
    {
        sensor_msgs::msg::CameraInfo camera_info;
        camera_info.distortion_model = "plumb_bob";
        camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
        camera_info.k = {
            projection[0], projection[1], projection[2],
            projection[4], projection[5], projection[6],
            projection[8], projection[9], projection[10]};
        camera_info.r = {
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0};
        camera_info.p = {
            projection[0], projection[1], projection[2], projection[3],
            projection[4], projection[5], projection[6], projection[7],
            projection[8], projection[9], projection[10], projection[11]};
        return camera_info;
    }

    std::string build_kitti_calibration_path() const
    {
        if (dataset_path_.empty())
        {
            return "";
        }

        return (std::filesystem::path(dataset_path_) /
                "data_odometry_calib" /
                "dataset" /
                "sequences" /
                normalize_dataset_sequence(dataset_sequence_) /
                "calib.txt")
            .string();
    }

    void update_profiling_metrics(const FrameProcessingMetrics &metrics)
    {
        interval_sum_buffer_age_ms_ += metrics.buffer_age_ms;
        interval_sum_stereo_skew_ms_ += metrics.stereo_skew_ms;
        interval_sum_conversion_ms_ += metrics.conversion_time_ms;
        interval_sum_inference_ms_ += metrics.inference_time_ms;
        interval_sum_disparity_ms_ += metrics.disparity_time_ms;
        interval_sum_publish_ms_ += metrics.publish_time_ms;
        interval_sum_overlay_ms_ += metrics.overlay_time_ms;
        interval_sum_frame_total_ms_ += metrics.frame_total_time_ms;
        interval_sum_rss_mb_ += metrics.rss_mb;
        interval_sum_peak_rss_mb_ += metrics.peak_rss_mb;
        interval_sum_num_2d_detections_ += metrics.num_2d_detections;
        interval_sum_num_3d_detections_ += metrics.num_3d_detections;
        interval_frame_count_ += 1;

        if (interval_frame_count_ < profiling_interval_frames_)
        {
            return;
        }

        const double avg_buffer_age_ms = interval_sum_buffer_age_ms_ / interval_frame_count_;
        const double avg_stereo_skew_ms = interval_sum_stereo_skew_ms_ / interval_frame_count_;
        const double avg_conversion_ms = interval_sum_conversion_ms_ / interval_frame_count_;
        const double avg_inference_ms = interval_sum_inference_ms_ / interval_frame_count_;
        const double avg_disparity_ms = interval_sum_disparity_ms_ / interval_frame_count_;
        const double avg_publish_ms = interval_sum_publish_ms_ / interval_frame_count_;
        const double avg_overlay_ms = interval_sum_overlay_ms_ / interval_frame_count_;
        const double avg_frame_total_ms = interval_sum_frame_total_ms_ / interval_frame_count_;
        const double avg_rss_mb = interval_sum_rss_mb_ / interval_frame_count_;
        const double avg_peak_rss_mb = interval_sum_peak_rss_mb_ / interval_frame_count_;
        const double avg_num_2d_detections = interval_sum_num_2d_detections_ / interval_frame_count_;
        const double avg_num_3d_detections = interval_sum_num_3d_detections_ / interval_frame_count_;
        double avg_process_cpu_percent = 0.0;
        double effective_output_rate_hz = 0.0;
        compute_interval_resource_metrics(avg_process_cpu_percent, effective_output_rate_hz);

        write_csv_interval_metrics(
            interval_frame_count_,
            avg_buffer_age_ms,
            avg_stereo_skew_ms,
            avg_conversion_ms,
            avg_inference_ms,
            avg_disparity_ms,
            avg_publish_ms,
            avg_overlay_ms,
            avg_frame_total_ms,
            avg_process_cpu_percent,
            effective_output_rate_hz,
            avg_rss_mb,
            avg_peak_rss_mb,
            avg_num_2d_detections,
            avg_num_3d_detections);

        interval_frame_count_ = 0;
        interval_sum_buffer_age_ms_ = 0.0;
        interval_sum_stereo_skew_ms_ = 0.0;
        interval_sum_conversion_ms_ = 0.0;
        interval_sum_inference_ms_ = 0.0;
        interval_sum_disparity_ms_ = 0.0;
        interval_sum_publish_ms_ = 0.0;
        interval_sum_overlay_ms_ = 0.0;
        interval_sum_frame_total_ms_ = 0.0;
        interval_sum_rss_mb_ = 0.0;
        interval_sum_peak_rss_mb_ = 0.0;
        interval_sum_num_2d_detections_ = 0.0;
        interval_sum_num_3d_detections_ = 0.0;
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
                    get_logger(),
                    "Failed to open CSV log file at '%s'. Disabling CSV logging.",
                    csv_log_file_path_.c_str());
                csv_logging_ = false;
                return;
            }

            csv_log_stream_
                << "timestamp_utc,dataset_sequence,interval_frames,avg_buffer_age_ms,avg_stereo_skew_ms,"
                << "avg_conversion_time_ms,avg_inference_time_ms,avg_disparity_time_ms,avg_publish_time_ms,"
                << "avg_overlay_time_ms,avg_frame_total_time_ms,avg_process_cpu_percent,effective_output_rate_hz,"
                << "avg_rss_mb,avg_peak_rss_mb,avg_num_2d_detections,avg_num_3d_detections,"
                << "total_received_left_frames,total_received_right_frames,total_processed_pairs,"
                << "total_overwritten_left_frames,total_overwritten_right_frames,total_sync_drop_frames\n";
            csv_log_stream_.flush();
            RCLCPP_INFO(
                get_logger(),
                "CSV logging enabled. Writing stereo interval metrics to '%s'.",
                csv_log_file_path_.c_str());
        }
        catch (const std::exception &exception)
        {
            RCLCPP_WARN(
                get_logger(),
                "Failed to initialize CSV logging: %s. Disabling CSV logging.",
                exception.what());
            csv_logging_ = false;
        }
    }

    void write_csv_interval_metrics(
        const int interval_frames,
        const double avg_buffer_age_ms,
        const double avg_stereo_skew_ms,
        const double avg_conversion_ms,
        const double avg_inference_ms,
        const double avg_disparity_ms,
        const double avg_publish_ms,
        const double avg_overlay_ms,
        const double avg_frame_total_ms,
        const double avg_process_cpu_percent,
        const double effective_output_rate_hz,
        const double avg_rss_mb,
        const double avg_peak_rss_mb,
        const double avg_num_2d_detections,
        const double avg_num_3d_detections)
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
                        << avg_stereo_skew_ms << ','
                        << avg_conversion_ms << ','
                        << avg_inference_ms << ','
                        << avg_disparity_ms << ','
                        << avg_publish_ms << ','
                        << avg_overlay_ms << ','
                        << avg_frame_total_ms << ','
                        << avg_process_cpu_percent << ','
                        << effective_output_rate_hz << ','
                        << avg_rss_mb << ','
                        << avg_peak_rss_mb << ','
                        << avg_num_2d_detections << ','
                        << avg_num_3d_detections << ','
                        << total_received_left_frames_ << ','
                        << total_received_right_frames_ << ','
                        << total_processed_pairs_ << ','
                        << total_overwritten_left_frames_ << ','
                        << total_overwritten_right_frames_ << ','
                        << total_sync_drop_frames_ << '\n';
        csv_log_stream_.flush();
    }

    std::string build_csv_filename() const
    {
        std::ostringstream filename_builder;
        filename_builder << get_name()
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
            elapsed_wall_ms > 0.0
                ? (1000.0 * static_cast<double>(interval_frame_count_) / elapsed_wall_ms)
                : 0.0;

        interval_resource_window_start_ = now;
        interval_resource_window_cpu_ms_ = current_cpu_ms;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoCameraProcessing>());
    rclcpp::shutdown();
    return 0;
}
