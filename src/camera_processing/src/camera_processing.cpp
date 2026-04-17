#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <array>
#include <cctype>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <sys/resource.h>
#include <unistd.h>
#include <vector>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include "vision_msgs/msg/detection2_d_array.hpp"
#include <camera_processing/yolo_detector.hpp>

struct ScopedTimer
{
    std::chrono::steady_clock::time_point start_time_;
    double &duration_ms_;

    ScopedTimer(double &duration_ms) : duration_ms_(duration_ms)
    {
        start_time_ = std::chrono::steady_clock::now();
    }

    ~ScopedTimer()
    {
        auto end_time = std::chrono::steady_clock::now();
        duration_ms_ = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end_time - start_time_).count();
    }
};

// These resource reads are Linux-specific, which matches the current target environment.
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

class CameraProcessing : public rclcpp::Node
{
    static constexpr std::size_t kSensorImageQosDepth{5};
    static constexpr std::size_t kSemanticOutputQosDepth{5};

private:
    struct FrameProcessingMetrics
    {
        double buffer_age_ms{0.0};
        double conversion_time_ms{0.0};
        double inference_time_ms{0.0};
        double publish_time_ms{0.0};
        double overlay_time_ms{0.0};
        double frame_total_time_ms{0.0};
        double rss_mb{0.0};
        double peak_rss_mb{0.0};
        std::size_t num_detections{0};
    };

    struct BufferedImage
    {
        sensor_msgs::msg::Image::SharedPtr msg;
        std::chrono::steady_clock::time_point received_steady;
    };

    double processing_rate_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_image_subscription_;
    std::mutex image_stream_mutex_;
    std::deque<BufferedImage> image_queue_;
    std::unique_ptr<YoloDetector> yolo_detector_;
    std::string model_path_;
    rclcpp::TimerBase::SharedPtr processing_timer_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr object_bbox_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
    bool publish_overlay_image_{false};
    bool publish_camera_info_{false};
    std::string dataset_path_;
    std::string camera_name_{"p2"};
    sensor_msgs::msg::CameraInfo camera_info_template_;
    int input_queue_size_{4};
    double max_buffer_age_ms_{300.0};
    int interval_frame_count_{0};
    double interval_sum_buffer_age_ms_{0.0};
    double interval_sum_conversion_ms_{0.0};
    double interval_sum_inference_ms_{0.0};
    double interval_sum_publish_ms_{0.0};
    double interval_sum_overlay_ms_{0.0};
    double interval_sum_frame_total_ms_{0.0};
    double interval_sum_rss_mb_{0.0};
    double interval_sum_peak_rss_mb_{0.0};
    double interval_sum_num_detections_{0.0};
    std::chrono::steady_clock::time_point interval_resource_window_start_;
    double interval_resource_window_cpu_ms_{0.0};

    std::uint64_t total_received_frames_{0};
    std::uint64_t total_processed_frames_{0};
    std::uint64_t total_overwritten_frames_{0};

    int profiling_interval_frames_;
    bool csv_logging_{false};
    std::string csv_log_dir_{"csv_logs/camera_processing"};
    std::string csv_log_file_path_;
    std::string dataset_sequence_{"unknown"};
    std::ofstream csv_log_stream_;

public:
    CameraProcessing() : Node("camera_processing")
    {
        this->declare_parameter<double>("processing_rate", 10.0);
        this->declare_parameter<std::string>("model_path", "models/yolo/yolov8n.onnx");
        this->declare_parameter<bool>("publish_overlay_image", false);
        this->declare_parameter<bool>("publish_camera_info", false);
        this->declare_parameter<int>("profiling_interval_frames", 60);
        this->declare_parameter<bool>("enable_csv_logging", false);
        this->declare_parameter<std::string>("csv_log_dir", "csv_logs/camera_processing");
        this->declare_parameter<std::string>("dataset_path", "");
        this->declare_parameter<std::string>("dataset_sequence", "unknown");
        this->declare_parameter<std::string>("camera_name", "p2");
        this->declare_parameter<int>("input_queue_size", 4);
        this->declare_parameter<double>("max_buffer_age_ms", 300.0);

        this->get_parameter("processing_rate", processing_rate_);
        this->get_parameter("model_path", model_path_);
        this->get_parameter("publish_overlay_image", publish_overlay_image_);
        this->get_parameter("publish_camera_info", publish_camera_info_);
        this->get_parameter("profiling_interval_frames", profiling_interval_frames_);
        this->get_parameter("enable_csv_logging", csv_logging_);
        this->get_parameter("csv_log_dir", csv_log_dir_);
        this->get_parameter("dataset_path", dataset_path_);
        this->get_parameter("dataset_sequence", dataset_sequence_);
        this->get_parameter("camera_name", camera_name_);
        input_queue_size_ = std::max(1, static_cast<int>(this->get_parameter("input_queue_size").as_int()));
        max_buffer_age_ms_ = std::max(0.0, this->get_parameter("max_buffer_age_ms").as_double());

        const auto camera_image_qos =
            rclcpp::SensorDataQoS().keep_last(kSensorImageQosDepth);
        const auto semantic_output_qos =
            rclcpp::QoS(rclcpp::KeepLast(kSemanticOutputQosDepth))
                .reliable()
                .durability_volatile();

        processing_timer_ = this->create_wall_timer(
            processing_rate_ > 0 ? std::chrono::milliseconds(static_cast<int>(1000.0 / processing_rate_)) : std::chrono::milliseconds(100),
            std::bind(&CameraProcessing::process_latest_image, this));
        camera_image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera_in",
            camera_image_qos,
            std::bind(&CameraProcessing::camera_image_subscriber_callback, this, std::placeholders::_1));

        yolo_detector_ = std::make_unique<YoloDetector>(model_path_);
        object_bbox_publisher_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
            "object_detections",
            semantic_output_qos);
        if (publish_overlay_image_)
        {
            overlay_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
                "overlay_image",
                semantic_output_qos);
        }
        if (publish_camera_info_ && load_camera_info_template())
        {
            camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
                "camera_info",
                semantic_output_qos);
        }
        else
        {
            publish_camera_info_ = false;
        }
        initialize_csv_logging();
        reset_resource_window();
        RCLCPP_INFO(this->get_logger(), "CameraProcessing node has been initialized.");
    }

    void camera_image_subscriber_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {

        std::lock_guard<std::mutex> lock(image_stream_mutex_);
        if (static_cast<int>(image_queue_.size()) >= input_queue_size_)
        {
            image_queue_.pop_front();
            total_overwritten_frames_++;
        }
        image_queue_.push_back(BufferedImage{msg, std::chrono::steady_clock::now()});
        total_received_frames_++;
    }

    void process_latest_image()
    {
        FrameProcessingMetrics metrics;
        {
            ScopedTimer total_frame_timer(metrics.frame_total_time_ms);
            cv::Mat frame_image;
            sensor_msgs::msg::Image::SharedPtr image_to_process_;
            std::chrono::steady_clock::time_point image_received_steady_;

            {
                std::lock_guard<std::mutex> lock(image_stream_mutex_);
                const auto now_steady = std::chrono::steady_clock::now();
                while (!image_queue_.empty())
                {
                    const double queue_age_ms =
                        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                            now_steady - image_queue_.front().received_steady)
                            .count();
                    if (queue_age_ms <= max_buffer_age_ms_)
                    {
                        break;
                    }

                    image_queue_.pop_front();
                    total_overwritten_frames_++;
                }

                if (!image_queue_.empty())
                {
                    image_to_process_ = image_queue_.front().msg;
                    image_received_steady_ = image_queue_.front().received_steady;
                    image_queue_.pop_front();
                }
                else
                {
                    return;
                }
            }
            auto now_steady = std::chrono::steady_clock::now();
            metrics.buffer_age_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(now_steady - image_received_steady_).count();
            // convert ROS2 image message to OpenCV format to be proceessed by yolo
            // call in conversion scoped Timer to measure conversion time
            {
                ScopedTimer conversion_timer(metrics.conversion_time_ms);
                frame_image = ros2_image_to_opencv(image_to_process_);
            }
            if (frame_image.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to convert ROS2 image to OpenCV format.");
                return;
            }
            std::vector<YoloDetectionResult> detections;
            {
                // Measure inference time
                ScopedTimer inference_timer(metrics.inference_time_ms);
                detections = yolo_detector_->infer(frame_image);
            }
            // RCLCPP_INFO(this->get_logger(), "Detected %zu objects in the image.", detections.size());
            //  puiblish the detection results with bounding boxes
            {
                ScopedTimer publish_timer(metrics.publish_time_ms);
                publish_object_detections(detections, image_to_process_->header);
                publish_camera_info(image_to_process_->header, image_to_process_->width, image_to_process_->height);
            }
            if (publish_overlay_image_)
            {
                ScopedTimer overlay_timer(metrics.overlay_time_ms);
                publish_overlay_image(frame_image, detections, image_to_process_->header);
            }

            total_processed_frames_++;
            metrics.num_detections = detections.size();
            metrics.rss_mb = read_current_rss_mb();
            metrics.peak_rss_mb = read_peak_rss_mb();
        }
        update_profiling_metrics(metrics);
    }

    cv::Mat ros2_image_to_opencv(const sensor_msgs::msg::Image::SharedPtr &ros2_image_to_process)
    {
        // Convert ROS2 image message to OpenCV format
        cv::Mat opencv_image;
        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(ros2_image_to_process, sensor_msgs::image_encodings::BGR8);
            opencv_image = cv_ptr->image;
            return opencv_image;
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return cv::Mat(); // Return an empty image on error
        }
    }

    void publish_object_detections(
        const std::vector<YoloDetectionResult> &detections,
        const std_msgs::msg::Header &img_header)
    {
        vision_msgs::msg::Detection2DArray detection_msg_array;
        detection_msg_array.header = img_header;
        detection_msg_array.detections.reserve(detections.size());

        for (const auto &detection : detections)
        {
            vision_msgs::msg::Detection2D detection_msg;
            detection_msg.header = img_header;

            detection_msg.bbox.center.position.x =
                detection.bounding_box.x + detection.bounding_box.width / 2.0;
            detection_msg.bbox.center.position.y =
                detection.bounding_box.y + detection.bounding_box.height / 2.0;
            detection_msg.bbox.center.theta = 0.0;
            detection_msg.bbox.size_x = detection.bounding_box.width;
            detection_msg.bbox.size_y = detection.bounding_box.height;

            vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
            hypothesis.hypothesis.class_id = std::to_string(detection.class_id);
            hypothesis.hypothesis.score = detection.confidence;

            detection_msg.results.push_back(hypothesis);
            detection_msg_array.detections.push_back(detection_msg);
        }
        // RCLCPP_INFO(this->get_logger(), "Publishing %zu detections.", detection_msg_array.detections.size());
        object_bbox_publisher_->publish(detection_msg_array);
    }

    void publish_overlay_image(const cv::Mat &frame_image, const std::vector<YoloDetectionResult> &detections, const std_msgs::msg::Header &img_header)
    {
        cv::Mat overlay_image = frame_image.clone();
        for (const auto &detection : detections)
        {
            cv::rectangle(overlay_image, detection.bounding_box, cv::Scalar(0, 255, 0), 2);
            std::string label = "ID: " + std::to_string(detection.class_id) + " Conf: " + std::to_string(detection.confidence);
            int baseline = 0;
            cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
            int top = std::max(detection.bounding_box.y, label_size.height);
            cv::rectangle(overlay_image, cv::Point(detection.bounding_box.x, top - label_size.height),
                          cv::Point(detection.bounding_box.x + label_size.width, top + baseline),
                          cv::Scalar(0, 255, 0), cv::FILLED);
            cv::putText(overlay_image, label, cv::Point(detection.bounding_box.x, top),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        }

        // Convert OpenCV image back to ROS2 image message
        sensor_msgs::msg::Image overlay_image_msg;
        try
        {
            std_msgs::msg::Header header = img_header;
            header.stamp = this->get_clock()->now(); // Update timestamp to current time
            cv_bridge::CvImage cv_overlay_image(header, sensor_msgs::image_encodings::BGR8, overlay_image);
            overlay_image_msg = *cv_overlay_image.toImageMsg();
            overlay_image_publisher_->publish(overlay_image_msg);
            // RCLCPP_INFO(this->get_logger(), "Published overlay image with %zu detections.", detections.size());
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception while converting overlay image: %s", e.what());
        }
    }

    void publish_camera_info(
        const std_msgs::msg::Header &img_header,
        const std::uint32_t image_width,
        const std::uint32_t image_height)
    {
        if (!publish_camera_info_ || !camera_info_publisher_)
        {
            return;
        }

        auto camera_info_msg = camera_info_template_;
        camera_info_msg.header = img_header;
        camera_info_msg.width = image_width;
        camera_info_msg.height = image_height;
        camera_info_publisher_->publish(camera_info_msg);
    }

    bool load_camera_info_template()
    {
        const auto calib_path = build_kitti_calibration_path();
        if (calib_path.empty())
        {
            RCLCPP_WARN(this->get_logger(), "CameraInfo publishing requested but dataset_path/dataset_sequence is not configured.");
            return false;
        }

        std::ifstream calib_stream(calib_path);
        if (!calib_stream.is_open())
        {
            RCLCPP_WARN(this->get_logger(), "Failed to open KITTI calibration file '%s'.", calib_path.c_str());
            return false;
        }

        const std::string projection_label = build_projection_label();
        std::string line;
        while (std::getline(calib_stream, line))
        {
            if (!starts_with(line, projection_label))
            {
                continue;
            }

            std::istringstream line_stream(line.substr(projection_label.size()));
            std::array<double, 12> projection{};
            for (double &value : projection)
            {
                if (!(line_stream >> value))
                {
                    RCLCPP_WARN(this->get_logger(), "Invalid projection row '%s' in '%s'.", projection_label.c_str(), calib_path.c_str());
                    return false;
                }
            }

            camera_info_template_.distortion_model = "plumb_bob";
            camera_info_template_.d = {0.0, 0.0, 0.0, 0.0, 0.0};
            camera_info_template_.k = {
                projection[0], projection[1], projection[2],
                projection[4], projection[5], projection[6],
                projection[8], projection[9], projection[10]};
            camera_info_template_.r = {
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0};
            camera_info_template_.p = {
                projection[0], projection[1], projection[2], projection[3],
                projection[4], projection[5], projection[6], projection[7],
                projection[8], projection[9], projection[10], projection[11]};

            RCLCPP_INFO(this->get_logger(), "Loaded KITTI projection row '%s' from '%s'.", projection_label.c_str(), calib_path.c_str());
            return true;
        }

        RCLCPP_WARN(this->get_logger(), "Projection row '%s' was not found in '%s'.", projection_label.c_str(), calib_path.c_str());
        return false;
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
                "calib.txt").string();
    }

    std::string build_projection_label() const
    {
        std::string label = camera_name_;
        std::transform(label.begin(), label.end(), label.begin(), [](unsigned char ch) {
            return static_cast<char>(std::toupper(ch));
        });
        return label + ":";
    }

    static bool starts_with(const std::string &value, const std::string &prefix)
    {
        return value.rfind(prefix, 0) == 0;
    }

    static std::string normalize_dataset_sequence(std::string sequence)
    {
        sequence.erase(std::remove_if(sequence.begin(), sequence.end(), [](unsigned char ch) {
            return std::isspace(ch) != 0;
        }), sequence.end());

        if (sequence.empty())
        {
            return sequence;
        }

        const bool numeric = std::all_of(sequence.begin(), sequence.end(), [](unsigned char ch) {
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

    void update_profiling_metrics(const FrameProcessingMetrics &metrics)
    {

        // sum the frame metrics to calculate average metrics over the profiling interval
        interval_sum_buffer_age_ms_ += metrics.buffer_age_ms;
        interval_sum_conversion_ms_ += metrics.conversion_time_ms;
        interval_sum_inference_ms_ += metrics.inference_time_ms;
        interval_sum_publish_ms_ += metrics.publish_time_ms;
        interval_sum_overlay_ms_ += metrics.overlay_time_ms;
        interval_sum_frame_total_ms_ += metrics.frame_total_time_ms;
        interval_sum_rss_mb_ += metrics.rss_mb;
        interval_sum_peak_rss_mb_ += metrics.peak_rss_mb;
        interval_sum_num_detections_ += metrics.num_detections;
        interval_frame_count_++;
        if (interval_frame_count_ >= profiling_interval_frames_)
        {
            // Calculate and log average metrics
            double avg_buffer_time = interval_sum_buffer_age_ms_ / interval_frame_count_;
            double avg_conversion_time = interval_sum_conversion_ms_ / interval_frame_count_;
            double avg_inference_time = interval_sum_inference_ms_ / interval_frame_count_;
            double avg_publish_time = interval_sum_publish_ms_ / interval_frame_count_;
            double avg_overlay_time = interval_sum_overlay_ms_ / interval_frame_count_;
            double avg_frame_total_time = interval_sum_frame_total_ms_ / interval_frame_count_;
            double avg_rss_mb = interval_sum_rss_mb_ / interval_frame_count_;
            double avg_peak_rss_mb = interval_sum_peak_rss_mb_ / interval_frame_count_;
            double avg_num_detections = interval_sum_num_detections_ / interval_frame_count_;
            double avg_process_cpu_percent = 0.0;
            double effective_output_rate_hz = 0.0;
            compute_interval_resource_metrics(avg_process_cpu_percent, effective_output_rate_hz);

            write_csv_interval_metrics(
                interval_frame_count_,
                avg_buffer_time,
                avg_conversion_time,
                avg_inference_time,
                avg_publish_time,
                avg_overlay_time,
                avg_frame_total_time,
                avg_process_cpu_percent,
                effective_output_rate_hz,
                avg_rss_mb,
                avg_peak_rss_mb,
                avg_num_detections);

            // RCLCPP_INFO(this->get_logger(), "Average Frame processing metrics over last %d frames: Buffer Age: %.2f ms, Conversion Time: %.2f ms, Inference Time: %.2f ms, Publish Time: %.2f ms, Overlay Time: %.2f ms, Frame Total Time: %.2f ms, Average Number of Detections: %0.2f, Total Received Frames: %ld, Total Processed Frames: %ld, Total Overwritten Frames: %ld",
            //             profiling_interval_frames_,
            //             avg_buffer_time,
            //             avg_conversion_time,
            //             avg_inference_time,
            //             avg_publish_time,
            //             avg_overlay_time,
            //             avg_frame_total_time,
            //             avg_num_detections,
            //             total_received_frames_,
            //             total_processed_frames_,
            //             total_overwritten_frames_);

            // Reset the counters for the next interval
            interval_frame_count_ = 0;
            interval_sum_buffer_age_ms_ = 0.0;
            interval_sum_conversion_ms_ = 0.0;
            interval_sum_inference_ms_ = 0.0;
            interval_sum_publish_ms_ = 0.0;
            interval_sum_overlay_ms_ = 0.0;
            interval_sum_frame_total_ms_ = 0.0;
            interval_sum_rss_mb_ = 0.0;
            interval_sum_peak_rss_mb_ = 0.0;
            interval_sum_num_detections_ = 0.0;
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

            csv_log_stream_ << "timestamp_utc,dataset_sequence,interval_frames,avg_buffer_age_ms,avg_conversion_time_ms,avg_inference_time_ms,avg_publish_time_ms,avg_overlay_time_ms,avg_frame_total_time_ms,avg_process_cpu_percent,effective_output_rate_hz,avg_rss_mb,avg_peak_rss_mb,avg_num_detections,total_received_frames,total_processed_frames,total_overwritten_frames\n";
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
        double avg_buffer_time,
        double avg_conversion_time,
        double avg_inference_time,
        double avg_publish_time,
        double avg_overlay_time,
        double avg_frame_total_time,
        double avg_process_cpu_percent,
        double effective_output_rate_hz,
        double avg_rss_mb,
        double avg_peak_rss_mb,
        double avg_num_detections)
    {
        if (!csv_logging_ || !csv_log_stream_.is_open())
        {
            return;
        }

        csv_log_stream_ << current_utc_timestamp("%Y-%m-%dT%H:%M:%SZ") << ','
                        << dataset_sequence_ << ','
                        << interval_frames << ','
                        << std::fixed << std::setprecision(2)
                        << avg_buffer_time << ','
                        << avg_conversion_time << ','
                        << avg_inference_time << ','
                        << avg_publish_time << ','
                        << avg_overlay_time << ','
                        << avg_frame_total_time << ','
                        << avg_process_cpu_percent << ','
                        << effective_output_rate_hz << ','
                        << avg_rss_mb << ','
                        << avg_peak_rss_mb << ','
                        << avg_num_detections << ','
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
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraProcessing>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
