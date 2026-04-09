#ifndef YOLO_DETECTOR_HPP
#define YOLO_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

class YoloDetector
{
public:
    YoloDetector(const std::string &model_path);
    cv::Mat preprocess(const cv::Mat &image);
    std::vector<yolo_detection_result> infer(const cv::Mat &image, std::vector<cv::Rect> &boxes, std::vector<float> &confidences);

private:
    Ort::Env env_;
    Ort::SessionOptions session_options_;
    Ort::Session session_;
    std::vector<yolo_detection_result> detection_results_;
};

#endif // YOLO_DETECTOR_HPP 