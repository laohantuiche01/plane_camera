#include "../include/Yolov8Detector/Yolov8Detector.h"

#include <rclcpp/logging.hpp>

using namespace cv;
using namespace dnn;
using namespace std;

Yolov8::YOLOv8Detector::YOLOv8Detector(const string &modelPath, const vector<string> &classNames, int imgSize,
                                       float confThreshold, float nmsThreshold)
    : classNames(classNames), imgSize(imgSize),
      confThreshold(confThreshold), nmsThreshold(nmsThreshold),
      imgHeight(480), imgWidth(640) {
    net = readNet(modelPath);
    net.setPreferableBackend(DNN_BACKEND_OPENCV);
    net.setPreferableTarget(DNN_TARGET_CPU);
}

vector<Yolov8::Detection> Yolov8::YOLOv8Detector::detect(Mat &image) {
    Mat blob = preprocess(image);

    net.setInput(blob);

    vector<Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    return postprocess(outputs, image.size());
}

std::vector<std::vector<double> > Yolov8::YOLOv8Detector::drawDetections(
    Mat &image, const vector<Detection> &detections) {
    std::vector<std::vector<double> > position_container;
    for (const auto &detection: detections) {
        rectangle(image, detection.box, Scalar(0, 255, 0), 2);

        string label = format("%s: %.2f", classNames[detection.classId].c_str(), detection.confidence);

        int baseLine;
        std::vector<double> position;
        Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        Rect labelRect = Rect(detection.box.x, detection.box.y - labelSize.height - baseLine,
                              labelSize.width, labelSize.height + baseLine);
        rectangle(image, labelRect, Scalar(0, 255, 0), FILLED);


        position.push_back(detection.box.x + detection.box.width / 2 - imgWidth / 2);
        position.push_back(imgHeight / 2 - detection.box.y - detection.box.height / 2);

        //std::cout << detection.box.x + detection.box.width / 2 - imgWidth / 2 << endl;
        //std::cout << imgHeight / 2 - detection.box.y - detection.box.height / 2 << endl;

        circle(image, Point(detection.box.x + detection.box.width / 2, detection.box.y + detection.box.height / 2), 3,
               Scalar(0, 0, 255), FILLED);

        putText(image, label, Point(detection.box.x, detection.box.y - baseLine),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);

        position_container.push_back(position);
    }
    return position_container;
}

Mat Yolov8::YOLOv8Detector::preprocess(const Mat &image) const {
    double scale = 1.0 / 255.0;
    Scalar mean = Scalar(0, 0, 0);
    bool swapRB = true;

    Mat blob;
    blobFromImage(image, blob, scale, Size(imgSize, imgSize),
                  mean, swapRB, false, CV_32F);
    return blob;
}

vector<Yolov8::Detection> Yolov8::YOLOv8Detector::postprocess(const vector<Mat> &outputs, const Size &originalSize) {
    vector<Detection> detections;

    if (outputs.empty()) {
        cerr << "No outputs from network!" << endl;
        return detections;
    }

    Mat output = outputs[0];

    output = output.reshape(1, output.size[1]);
    transpose(output, output);

    int origW = originalSize.width;
    int origH = originalSize.height;

    vector<int> classIds;
    vector<float> confidences;
    vector<Rect> boxes;

    for (int i = 0; i < output.rows; i++) {
        Mat scores = output.row(i).colRange(4, 4 + classNames.size());

        Point classIdPoint;
        double maxScore;
        minMaxLoc(scores, 0, &maxScore, 0, &classIdPoint);

        if (maxScore > confThreshold) {
            float cx = output.at<float>(i, 0);
            float cy = output.at<float>(i, 1);
            float w = output.at<float>(i, 2);
            float h = output.at<float>(i, 3);

            int left = static_cast<int>((cx - w / 2) * origW / imgSize);
            int top = static_cast<int>((cy - h / 2) * origH / imgSize);
            int width = static_cast<int>(w * origW / imgSize);
            int height = static_cast<int>(h * origH / imgSize);

            left = max(0, min(left, origW - 1));
            top = max(0, min(top, origH - 1));
            width = max(1, min(width, origW - left));
            height = max(1, min(height, origH - top));

            classIds.push_back(classIdPoint.x);
            confidences.push_back(static_cast<float>(maxScore));
            boxes.push_back(Rect(left, top, width, height));
        }
    }

    vector<int> indices;
    NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices); // 非极大值抑制

    for (int idx: indices) {
        Detection detection;
        detection.classId = classIds[idx];
        detection.confidence = confidences[idx];
        detection.box = boxes[idx];
        detections.push_back(detection);
    }

    return detections;
}
