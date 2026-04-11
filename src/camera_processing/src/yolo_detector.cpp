#include <camera_processing/yolo_detector.hpp>

YoloDetector::YoloDetector(const std::string &model_path,
                           float conf_threshold,
                           float nms_threshold,
                           int input_width,
                           int input_height)
    : conf_threshold_(conf_threshold),
      nms_threshold_(nms_threshold),
      input_width_(input_width),
      input_height_(input_height)
{

    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    session_options_.SetIntraOpNumThreads(1);

    session_ = std::make_unique<Ort::Session>(
        env_, model_path.c_str(), session_options_);

    Ort::AllocatorWithDefaultOptions allocator;

    auto input_name_alloc = session_->GetInputNameAllocated(0, allocator);
    auto output_name_alloc = session_->GetOutputNameAllocated(0, allocator);

    input_name_ = input_name_alloc.get();
    output_name_ = output_name_alloc.get();
}

cv::Mat YoloDetector::preprocess(const cv::Mat &image, float &scale, int &pad_w, int &pad_h)
{
    const int img_w = image.cols;
    const int img_h = image.rows;

    scale = std::min(static_cast<float>(input_width_) / static_cast<float>(img_w),
                     static_cast<float>(input_height_) / static_cast<float>(img_h));

    const int new_w = static_cast<int>(img_w * scale);
    const int new_h = static_cast<int>(img_h * scale);

    cv::Mat resized;
    cv::resize(image, resized, cv::Size(new_w, new_h));

    cv::Mat padded = cv::Mat::zeros(input_height_, input_width_, CV_8UC3);
    pad_w = (input_width_ - new_w) / 2;
    pad_h = (input_height_ - new_h) / 2;

    resized.copyTo(padded(cv::Rect(pad_w, pad_h, new_w, new_h)));

    cv::Mat blob;
    cv::dnn::blobFromImage(
        padded,
        blob,
        1.0 / 255.0,
        cv::Size(input_width_, input_height_),
        cv::Scalar(0, 0, 0),
        true,
        false);

    return blob;
}

std::vector<YoloDetectionResult> YoloDetector::postprocess(const cv::Mat &image,
                                                           const cv::Mat &output,
                                                           float scale,
                                                           int pad_w,
                                                           int pad_h)
{
    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    const int dimensions = output.size[1];
    const int rows = output.size[2];

    cv::Mat output0(dimensions, rows, CV_32F, (void *)output.ptr<float>());
    cv::Mat output1 = output0.t();

    for (int i = 0; i < rows; ++i)
    {
        const float *data = output1.ptr<float>(i);

        const float x = data[0];
        const float y = data[1];
        float w = data[2];
        float h = data[3];

        cv::Mat scores(1, dimensions - 4, CV_32F, (void *)(data + 4));
        cv::Point class_id_point;
        double max_score = 0.0;

        cv::minMaxLoc(scores, nullptr, &max_score, nullptr, &class_id_point);

        if (max_score < conf_threshold_)
        {
            continue;
        }

        float left = x - 0.5f * w;
        float top = y - 0.5f * h;

        left = (left - static_cast<float>(pad_w)) / scale;
        top = (top - static_cast<float>(pad_h)) / scale;
        w /= scale;
        h /= scale;

        const int x1 = std::max(0, static_cast<int>(left));
        const int y1 = std::max(0, static_cast<int>(top));
        const int x2 = std::min(image.cols, static_cast<int>(left + w));
        const int y2 = std::min(image.rows, static_cast<int>(top + h));

        if (x2 <= x1 || y2 <= y1)
        {
            continue;
        }

        class_ids.push_back(class_id_point.x);
        confidences.push_back(static_cast<float>(max_score));
        boxes.emplace_back(x1, y1, x2 - x1, y2 - y1);
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold_, nms_threshold_, indices);

    std::vector<YoloDetectionResult> detections;
    detections.reserve(indices.size());

    for (const int idx : indices)
    {
        YoloDetectionResult det;
        det.bounding_box = boxes[idx];
        det.confidence = confidences[idx];
        det.class_id = class_ids[idx];
        detections.push_back(det);
    }

    return detections;
}


std::vector<YoloDetectionResult> YoloDetector::infer(const cv::Mat &image)
{
    if (image.empty())
    {
        throw std::runtime_error("Input image is empty.");
    }

    float scale = 1.0f;
    int pad_w = 0;
    int pad_h = 0;

    cv::Mat blob = preprocess(image, scale, pad_w, pad_h);

    std::array<int64_t, 4> input_shape = {
        1,
        3,
        input_height_,
        input_width_};

    const size_t input_tensor_size =
        static_cast<size_t>(input_shape[0] * input_shape[1] * input_shape[2] * input_shape[3]);

    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(
        OrtArenaAllocator, OrtMemTypeDefault);

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info,
        reinterpret_cast<float *>(blob.data),
        input_tensor_size,
        input_shape.data(),
        input_shape.size());

    const char *input_names[] = {input_name_.c_str()};
    const char *output_names[] = {output_name_.c_str()};

    auto output_tensors = session_->Run(
        Ort::RunOptions{nullptr},
        input_names,
        &input_tensor,
        1,
        output_names,
        1);

    if (output_tensors.empty())
    {
        throw std::runtime_error("ONNX Runtime returned no outputs.");
    }

    auto &output_tensor = output_tensors.front();
    auto output_info = output_tensor.GetTensorTypeAndShapeInfo();
    auto output_shape = output_info.GetShape();

    float *output_data = output_tensor.GetTensorMutableData<float>();

    std::vector<int> mat_shape;
    mat_shape.reserve(output_shape.size());
    for (const auto dim : output_shape)
    {
        mat_shape.push_back(static_cast<int>(dim));
    }

    cv::Mat output_mat(static_cast<int>(mat_shape.size()), mat_shape.data(), CV_32F, output_data);

    return postprocess(image, output_mat, scale, pad_w, pad_h);
}
