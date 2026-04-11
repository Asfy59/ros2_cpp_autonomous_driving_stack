#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include "vision_msgs/msg/detection2_d_array.hpp"
#include <camera_processing/yolo_detector.hpp>


class CameraProcessing : public rclcpp::Node
{
public:
    CameraProcessing() : Node("camera_processing")
    {
        this->declare_parameter<double>("processing_rate", 10.0);
        this->declare_parameter<std::string>("model_path", "models/yolo/yolov8n.onnx");
        this->declare_parameter<bool>("publish_overlay_image", false);

        this->get_parameter("processing_rate", processing_rate_);
        this->get_parameter("model_path", model_path_);
        this->get_parameter("publish_overlay_image", publish_overlay_image_);

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
        latest_image_ = msg;
        new_image_available_ = true;
        //RCLCPP_INFO(this->get_logger(), "Received new image with %d x %d pixels.", msg->width, msg->height);
    }

    void process_latest_image()
    {
        cv::Mat frame_image;
        sensor_msgs::msg::Image::SharedPtr image_to_process_;

        {
            std::lock_guard<std::mutex> lock(image_stream_mutex_);
            if (new_image_available_ && latest_image_)
            {
                // Process the latest image here
                RCLCPP_INFO(this->get_logger(), "Processing new image with %d x %d pixels.", latest_image_->width, latest_image_->height);
                image_to_process_ = latest_image_;
                new_image_available_ = false; // Reset the flag after processing
            }
            else
            {
                // No new image available, skip processing
                return;
            }
        }

        // convert ROS2 image message to OpenCV format to be proceessed by yolo
        frame_image = ros2_image_to_opencv(image_to_process_);
        if (frame_image.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert ROS2 image to OpenCV format.");
            return;
        }
        auto detections = yolo_detector_->infer(frame_image);
        RCLCPP_INFO(this->get_logger(), "Detected %zu objects in the image.", detections.size());
        // puiblish the detection results with bounding boxes
        publish_object_detections(detections, image_to_process_->header);
        if (publish_overlay_image_)
        {
            publish_overlay_image(frame_image, detections, image_to_process_->header);
        }
        
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
    RCLCPP_INFO(this->get_logger(), "Publishing %zu detections.", detection_msg_array.detections.size());
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
        RCLCPP_INFO(this->get_logger(), "Published overlay image with %zu detections.", detections.size());
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception while converting overlay image: %s", e.what());
    }
}

private:
    double processing_rate_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_image_subscription_;
    std::mutex image_stream_mutex_;
    sensor_msgs::msg::Image::SharedPtr latest_image_;
    std::unique_ptr<YoloDetector> yolo_detector_;
    std::string model_path_;
    bool new_image_available_{false};
    rclcpp::TimerBase::SharedPtr processing_timer_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr object_bbox_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_image_publisher_;
    bool publish_overlay_image_{false};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraProcessing>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}