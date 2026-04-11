#ifndef YOLO_DETECTOR_HPP
#define YOLO_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <onnxruntime_cxx_api.h>
#include <array>
#include <stdexcept>

struct YoloDetectionResult
    {
        cv::Rect bounding_box;
        float confidence;
        int class_id;
    };

class YoloDetector
{
public:
    YoloDetector(const std::string &model_path,
                         const float conf_threshold = 0.25f,
                         const float nms_threshold = 0.45f,
                         const int input_width = 640,
                         const int input_height = 640);
    cv::Mat preprocess(const cv::Mat& image, float& scale, int& pad_w, int& pad_h);
    std::vector<YoloDetectionResult> infer(const cv::Mat &image);
    std::vector<YoloDetectionResult> postprocess(const cv::Mat& image,
                                       const cv::Mat& output,
                                       float scale,
                                       int pad_w,
                                       int pad_h);
private:
    Ort::Env env_;
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;
    
    std::string input_name_;
    std::string output_name_;

    float conf_threshold_;
    float nms_threshold_;
    int input_width_;
    int input_height_;


};

#endif // YOLO_DETECTOR_HPP 