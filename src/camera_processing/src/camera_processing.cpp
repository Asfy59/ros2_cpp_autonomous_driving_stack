#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include "vision_msgs/msg/bounding_box2_d.hpp"
//#include <camera_processing/yolo_detector.hpp>

class CameraProcessing : public rclcpp::Node
{
public:
    CameraProcessing() : Node("camera_processing")
    {
        this->declare_parameter<double>("processing_rate", 10.0);
        //this->declare_parameter < std::string >> ("model_path", "./model/yolo/model.onnx");
        // this->declare_parameter<std::vector<double>>("crop_box_max", {30.0, 15.0, 2.0});
        // this->declare_parameter<std::vector<double>>("voxel_leaf_size", {0.1, 0.1, 0.1});
        // this->declare_parameter<bool>("enable_ground_segmentation", true);

        this->get_parameter("processing_rate", processing_rate_);
        //this->get_parameter("model_path", model_path_);
        // this->get_parameter("crop_box_min", crop_box_min_);
        // this->get_parameter("crop_box_max", crop_box_max_);
        // this->get_parameter("voxel_leaf_size", voxel_leaf_size_);
        // this->get_parameter("enable_ground_segmentation", enable_ground_segmentation_);

        processing_timer_ = this->create_wall_timer(
            processing_rate_ > 0 ? std::chrono::milliseconds(static_cast<int>(1000.0 / processing_rate_)) : std::chrono::milliseconds(100),
            std::bind(&CameraProcessing::process_latest_image, this));
        camera_image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera_image_in",
            10,
            std::bind(&CameraProcessing::camera_image_subscriber_callback, this, std::placeholders::_1));

        //YoloDetector yolo_detector(model_path_);

        processed_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "camera_image_out",
            10);
        RCLCPP_INFO(this->get_logger(), "CameraProcessing node has been initialized.");
    }

    void camera_image_subscriber_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(image_stream_mutex_);
        latest_image_ = msg;
        new_image_available_ = true;
        std::cout << "Received new image with " << msg->width * msg->height << " pixels." << std::endl;
    }

    void process_latest_image()
    {

        sensor_msgs::msg::Image::SharedPtr image_to_process_;

        {
            std::lock_guard<std::mutex> lock(image_stream_mutex_);
            if (new_image_available_ && latest_image_)
            {
                // Process the latest image here
                std::cout << "Processing image with " << latest_image_->width * latest_image_->height << " pixels." << std::endl;
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

        //detection_results_ = yolo_detector.infer(frame_image);
        // puiblish the detection results with bounding boxes
        //publish_object_detections(detection_results_);
        //  publish the processed image with bounding boxes(optional)
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

    void publish_object_detections(const vision_msgs::msg::BoundingBox2D &detections)
    {
        // Publish the detected bounding boxes

            object_bbox_publisher_->publish(detections);

    }

private:
    double processing_rate_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_image_subscription_;
    std::mutex image_stream_mutex_;
    sensor_msgs::msg::Image::SharedPtr latest_image_;
    std::string model_path_;
    bool new_image_available_{false};
    cv::Mat frame_image;
    //std::vector<yolo_detection_result> detection_results_;
    rclcpp::TimerBase::SharedPtr processing_timer_;
    rclcpp::Publisher<vision_msgs::msg::BoundingBox2D>::SharedPtr object_bbox_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_image_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraProcessing>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}