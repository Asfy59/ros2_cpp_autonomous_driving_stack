#include <camera_processing/yolo_detector.hpp>

YoloDetector::YoloDetector(const std::string &model_path)
{

    //load model using ONNX Runtime
    printf(model_path.c_str());
} 

cv::Mat YoloDetector::preprocess(const cv::Mat &image)
{
    // Implement preprocessing steps such as resizing, normalization, etc.
    // This is a placeholder implementation and should be replaced with actual preprocessing logic.
    cv::Mat preprocessed_image;
    cv::resize(image, preprocessed_image, cv::Size(640, 640)); // Resize to model input size
    preprocessed_image.convertTo(preprocessed_image, CV_32F, 1.0 / 255); // Normalize pixel values
    return preprocessed_image;
}


std::vector<yolo_detection_result>YoloDetector::infer(const cv::Mat &image)
{
    // Implement inference logic using ONNX Runtime
    // This is a placeholder implementation and should be replaced with actual inference logic.
    std::vector<yolo_detection_result> results;
    // Perform inference and populate results with detected bounding boxes and confidence scores
    return results;
}