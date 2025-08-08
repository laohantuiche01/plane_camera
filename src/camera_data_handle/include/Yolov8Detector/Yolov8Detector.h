#ifndef YOLOV8DETECTOR_H
#define YOLOV8DETECTOR_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <openvino/openvino.hpp>

namespace Yolov8 {

    inline std::vector<std::string> defaultClassNames = {
        "H", "tent", "car",
        "bridge", "pillbox", "tank"
    };

    struct Detection {
        int classId;
        float confidence;
        cv::Rect box;
    };

    class YOLOv8Detector {
    public:
        explicit YOLOv8Detector(const std::string &modelPath,
                                const std::vector<std::string> &classNames = defaultClassNames,
                                int imgSize = 640,
                                float confThreshold = 0.5,
                                float nmsThreshold = 0.4);

        void setConfidenceThreshold(float threshold) { confThreshold_ = threshold; }

        void setNmsThreshold(float threshold) { nmsThreshold_ = threshold; }

        std::vector<Detection> detect(cv::Mat &image);

        std::vector<std::vector<double> > drawDetections(cv::Mat &image, const std::vector<Detection> &detections);

    private:

#ifdef DETECTION_OPENVINO_OPEN
        ov::Core core_{};
        ov::CompiledModel compiled_model_{};
        ov::InferRequest infer_request_{};
        ov::Shape input_shape_{};
        std::shared_ptr<ov::Model> model_{};
        std::string inputName_{};
        std::vector<std::string> outputNames_;



#endif

#ifndef DETECTION_OPENVINO_OPEN
        cv::dnn::Net net_;
#endif

        std::vector<std::string> classNames_;
        int imgSize_;
        float confThreshold_;
        float nmsThreshold_;
        int imgWidth_;
        int imgHeight_;

        [[nodiscard]] cv::Mat preprocess(const cv::Mat &image) const;
        std::vector<Detection> postprocess(const std::vector<cv::Mat> &outputs, const cv::Size &originalSize);

    };



}

#endif
