#include <rclcpp/rclcpp.hpp>
#include <mutex>
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

class CameraProcessing : public rclcpp::Node
{

private:
    struct FrameProcessingMetrics
    {
        double buffer_age_ms{0.0};
        double conversion_time_ms{0.0};
        double inference_time_ms{0.0};
        double publish_time_ms{0.0};
        double overlay_time_ms{0.0};
        double frame_total_time_ms{0.0};
        std::size_t num_detections{0};
    };

    double processing_rate_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_image_subscription_;
    std::mutex image_stream_mutex_;
    sensor_msgs::msg::Image::SharedPtr latest_image_;
    std::chrono::steady_clock::time_point latest_image_received_steady_;
    std::unique_ptr<YoloDetector> yolo_detector_;
    std::string model_path_;
    bool new_image_available_{false};
    rclcpp::TimerBase::SharedPtr processing_timer_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr object_bbox_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_image_publisher_;
    bool publish_overlay_image_{false};
    int interval_frame_count_{0};
    double interval_sum_buffer_age_ms_{0.0};
    double interval_sum_conversion_ms_{0.0};
    double interval_sum_inference_ms_{0.0};
    double interval_sum_publish_ms_{0.0};
    double interval_sum_overlay_ms_{0.0};
    double interval_sum_frame_total_ms_{0.0};
    double interval_sum_num_detections_{0.0};

    std::uint64_t total_received_frames_{0};
    std::uint64_t total_processed_frames_{0};
    std::uint64_t total_overwritten_frames_{0};

    int profiling_interval_frames_;
    bool csv_logging_{false};
    std::string csv_log_file_path_{"camera_processing_profiling_log.csv"};

public:
    CameraProcessing() : Node("camera_processing")
    {
        this->declare_parameter<double>("processing_rate", 10.0);
        this->declare_parameter<std::string>("model_path", "models/yolo/yolov8n.onnx");
        this->declare_parameter<bool>("publish_overlay_image", false);
        this->declare_parameter<int>("profiling_interval_frames", 60);

        this->get_parameter("processing_rate", processing_rate_);
        this->get_parameter("model_path", model_path_);
        this->get_parameter("publish_overlay_image", publish_overlay_image_);
        this->get_parameter("profiling_interval_frames", profiling_interval_frames_);

        processing_timer_ = this->create_wall_timer(
            processing_rate_ > 0 ? std::chrono::milliseconds(static_cast<int>(1000.0 / processing_rate_)) : std::chrono::milliseconds(100),
            std::bind(&CameraProcessing::process_latest_image, this));
        camera_image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera_in",
            10,
            std::bind(&CameraProcessing::camera_image_subscriber_callback, this, std::placeholders::_1));

        yolo_detector_ = std::make_unique<YoloDetector>(model_path_);
        object_bbox_publisher_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
            "object_detections",
            10);
        if (publish_overlay_image_)
        {
            overlay_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
                "overlay_image",
                10);
        }
        RCLCPP_INFO(this->get_logger(), "CameraProcessing node has been initialized.");
    }

    void camera_image_subscriber_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {

        std::lock_guard<std::mutex> lock(image_stream_mutex_);
        if (new_image_available_)
        {
            total_overwritten_frames_++;
        }
        latest_image_ = msg;
        latest_image_received_steady_ = std::chrono::steady_clock::now();
        new_image_available_ = true; // RCLCPP_INFO(this->get_logger(), "Received new image. Input callback time: %.6f seconds.", input_time.count());
        total_received_frames_++;
        // RCLCPP_INFO(this->get_logger(), "Received new image with %d x %d pixels.", msg->width, msg->height);
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
                if (new_image_available_ && latest_image_)
                {
                    // Process the latest image here
                    // RCLCPP_INFO(this->get_logger(), "Processing new image with %d x %d pixels.", latest_image_->width, latest_image_->height);
                    image_to_process_ = latest_image_;
                    image_received_steady_ = latest_image_received_steady_;
                    new_image_available_ = false; // Reset the flag after processing
                }
                else
                {
                    // No new image available, skip processing
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
            }
            if (publish_overlay_image_)
            {
                ScopedTimer overlay_timer(metrics.overlay_time_ms);
                publish_overlay_image(frame_image, detections, image_to_process_->header);
            }

            total_processed_frames_++;
            metrics.num_detections = detections.size();
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

    void update_profiling_metrics(const FrameProcessingMetrics &metrics)
    {

        // sum the frame metrics to calculate average metrics over the profiling interval
        interval_sum_buffer_age_ms_ += metrics.buffer_age_ms;
        interval_sum_conversion_ms_ += metrics.conversion_time_ms;
        interval_sum_inference_ms_ += metrics.inference_time_ms;
        interval_sum_publish_ms_ += metrics.publish_time_ms;
        interval_sum_overlay_ms_ += metrics.overlay_time_ms;
        interval_sum_frame_total_ms_ += metrics.frame_total_time_ms;
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
            double avg_num_detections = interval_sum_num_detections_ / interval_frame_count_;

            RCLCPP_INFO(this->get_logger(), "Average Frame processing metrics over last %d frames: Buffer Age: %.2f ms, Conversion Time: %.2f ms, Inference Time: %.2f ms, Publish Time: %.2f ms, Overlay Time: %.2f ms, Frame Total Time: %.2f ms, Average Number of Detections: %0.2f, Total Received Frames: %ld, Total Processed Frames: %ld, Total Overwritten Frames: %ld",
                        profiling_interval_frames_,
                        avg_buffer_time,
                        avg_conversion_time,
                        avg_inference_time,
                        avg_publish_time,
                        avg_overlay_time,
                        avg_frame_total_time,
                        avg_num_detections,
                        total_received_frames_,
                        total_processed_frames_,
                        total_overwritten_frames_);

            // Reset the counters for the next interval
            interval_frame_count_ = 0;
            interval_sum_buffer_age_ms_ = 0.0;
            interval_sum_conversion_ms_ = 0.0;
            interval_sum_inference_ms_ = 0.0;
            interval_sum_publish_ms_ = 0.0;
            interval_sum_overlay_ms_ = 0.0;
            interval_sum_frame_total_ms_ = 0.0;
            interval_sum_num_detections_ = 0.0;
        }
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