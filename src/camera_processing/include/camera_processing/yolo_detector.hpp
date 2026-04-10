#ifndef YOLO_DETECTOR_HPP
#define YOLO_DETECTOR_HPP

#include <opencv2/opencv.hpp>

struct yolo_detection_result
    {
        cv::Rect bounding_box;
        float confidence;
        int class_id;
    };

class YoloDetector
{
public:
    YoloDetector(const std::string &model_path);
    cv::Mat preprocess(const cv::Mat &image);
    std::vector<yolo_detection_result> infer(const cv::Mat &image);

private:

    std::vector<yolo_detection_result> detection_results_;
};

#endif // YOLO_DETECTOR_HPP 