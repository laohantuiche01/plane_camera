#ifndef YOLOV8DETECTOR_H
#define YOLOV8DETECTOR_H

#include <opencv2/opencv.hpp>
#include <vector>

namespace Yolov8 {
    using namespace cv;
    using namespace dnn;
    using namespace std;

    inline std::vector<std::string> defaultClassNames = {
        "H", "tent", "car",
        "bridge", "pillbox", "tank"
    };

    struct Detection {
        int classId;
        float confidence;
        Rect box;
    };

    class YOLOv8Detector {
    public:
        explicit YOLOv8Detector(const string &modelPath,
                       const vector<string> &classNames=defaultClassNames,
                       int imgSize = 640,
                       float confThreshold = 0.5,
                       float nmsThreshold = 0.4);

        void setConfidenceThreshold(float threshold) { confThreshold = threshold; }

        void setNmsThreshold(float threshold) { nmsThreshold = threshold; }

        vector<Detection> detect(Mat &image);

        std::vector<std::vector<double>> drawDetections(Mat &image, const vector<Detection> &detections) ;

    private:
        Net net;
        vector<string> classNames;
        int imgSize;
        float confThreshold;
        float nmsThreshold;
        int imgWidth;
        int imgHeight;

        [[nodiscard]] Mat preprocess(const Mat &image) const ;

        vector<Detection> postprocess(const vector<Mat> &outputs, const Size &originalSize);
    };
}


#endif //YOLOV8DETECTOR_H
