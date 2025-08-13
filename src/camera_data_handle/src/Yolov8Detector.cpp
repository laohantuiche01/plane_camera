#include "../include/Yolov8Detector/Yolov8Detector.h"

#include <iostream>
#include <rclcpp/logging.hpp>
#include <openvino/openvino.hpp>
using namespace cv;
using namespace dnn;
using namespace std;


Yolov8::YOLOv8Detector::YOLOv8Detector(const string &modelPath, const vector<string> &classNames, int imgSize,
                                       float confThreshold, float nmsThreshold)
    : classNames_(classNames), imgSize_(imgSize),
      confThreshold_(confThreshold), nmsThreshold_(nmsThreshold),
      imgHeight_(480), imgWidth_(640) {
#ifndef DETECTION_OPENVINO_OPEN
    net_ = readNet(modelPath);
    net_.setPreferableBackend(DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(DNN_TARGET_CPU);
#endif

#ifdef DETECTION_OPENVINO_OPEN
    try {
        auto model = core_.read_model(modelPath);
        auto inputs = model->inputs();
        inputName_ = inputs[0].get_any_name();

        auto outputs = model->outputs();
        for (const auto &output: outputs) {
            outputNames_.push_back(output.get_any_name());
        }

        ov::Shape inputShape = inputs[0].get_shape();
        inputShape[0] = 1;
        inputShape[2] = imgSize_;
        inputShape[3] = imgSize_;
        model->reshape({inputShape});

        for (const auto &device: core_.get_available_devices()) {
            std::cout << "Available device: " << device << std::endl;
        }

        ov::AnyMap config;
        config["GPU_DISABLE_WINOGRAD_CONVOLUTION"] = "True";

        compiled_model_ = core_.compile_model(model, "GPU", config);
        //compiled_model_ = core_.compile_model(model, "GPU");

        infer_request_ = compiled_model_.create_infer_request();
    } catch (const std::exception &e) {
        std::cerr << "cannot initialize : " << e.what() << std::endl;
        throw;
    }
#endif
}

vector<Yolov8::Detection> Yolov8::YOLOv8Detector::detect(Mat &image) {
    Mat blob = preprocess(image);
#ifndef DETECTION_OPENVINO_OPEN

    net_.setInput(blob);

    vector<Mat> outputs;
    net_.forward(outputs, net_.getUnconnectedOutLayersNames());

    return postprocess(outputs, image.size());
#endif

#ifdef DETECTION_OPENVINO_OPEN

    ov::Shape temp_shape;
    temp_shape.push_back(1);
    temp_shape.push_back(3);
    temp_shape.push_back(imgSize_);
    temp_shape.push_back(imgSize_);

    ov::Tensor inputTensor(ov::element::f32,
                           temp_shape,
                           blob.ptr<float>());

    infer_request_.set_tensor(inputName_, inputTensor);

    infer_request_.infer();

    std::vector<cv::Mat> outputs;
    for (const auto &name: outputNames_) {
        ov::Tensor outputTensor = infer_request_.get_tensor(name);
        const float *data = outputTensor.data<float>();
        ov::Shape shape = outputTensor.get_shape();

        int rows = shape[1];
        int cols = shape[2];
        cv::Mat output(cols, rows, CV_32F);
        memcpy(output.data, data, rows * cols * sizeof(float));
        outputs.push_back(output);
    }

    return postprocess(outputs, image.size());

#endif
}

Mat Yolov8::YOLOv8Detector::preprocess(const Mat &image) const {
    double scale = 1.0 / 255.0;
    Scalar mean = Scalar(0, 0, 0);
    bool swapRB = true;

    Mat blob;
    blobFromImage(image, blob, scale, Size(imgSize_, imgSize_),
                  mean, swapRB, false, CV_32F);
    return blob;
}


vector<Yolov8::Detection> Yolov8::YOLOv8Detector::postprocess(const vector<Mat> &outputs, const Size &originalSize) {
    vector<Detection> detections;

    Mat output = outputs[0];

    output = output.reshape(1, output.size[1]);
    transpose(output, output);

    int origW = originalSize.width;
    int origH = originalSize.height;

    vector<int> classIds;
    vector<float> confidences;
    vector<Rect> boxes;

    for (int i = 0; i < output.rows; i++) {
        Mat scores = output.row(i).colRange(4, 4 + classNames_.size());

        Point classIdPoint;
        double maxScore;
        minMaxLoc(scores, 0, &maxScore, 0, &classIdPoint);

        if (maxScore > confThreshold_) {
            float cx = output.at<float>(i, 0);
            float cy = output.at<float>(i, 1);
            float w = output.at<float>(i, 2);
            float h = output.at<float>(i, 3);

            int left = static_cast<int>((cx - w / 2) * origW / imgSize_);
            int top = static_cast<int>((cy - h / 2) * origH / imgSize_);
            int width = static_cast<int>(w * origW / imgSize_);
            int height = static_cast<int>(h * origH / imgSize_);

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
    NMSBoxes(boxes, confidences, confThreshold_, nmsThreshold_, indices); // 非极大值抑制

    for (int idx: indices) {
        Detection detection;
        detection.classId = classIds[idx];
        detection.confidence = confidences[idx];
        detection.box = boxes[idx];
        detections.push_back(detection);
    }

    return detections;
}

std::vector<std::vector<double> > Yolov8::YOLOv8Detector::drawDetections(
    Mat &image, const vector<Detection> &detections) {
    std::vector<std::vector<double> > position_container;
    for (const auto &detection: detections) {
        std::vector<double> position;

#ifndef NO_IMAGE
        rectangle(image, detection.box, Scalar(0, 255, 0), 2);
        string label = format("%s: %.2f", classNames_[detection.classId].c_str(), detection.confidence);

        int baseLine;

        Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        Rect labelRect = Rect(detection.box.x, detection.box.y - labelSize.height - baseLine,
                              labelSize.width, labelSize.height + baseLine);

        rectangle(image, labelRect, Scalar(0, 255, 0), FILLED);
#endif

        // position.push_back(detection.box.x + detection.box.width / 2 - imgWidth_ / 2);
        // position.push_back(imgHeight_ / 2 - detection.box.y - detection.box.height / 2);

        position.push_back(imgHeight_ / 2 - detection.box.y - detection.box.height / 2);
        position.push_back(imgWidth_ / 2 - detection.box.x - detection.box.width / 2);

        //std::cout << detection.box.x + detection.box.width / 2 - imgWidth / 2 << endl;
        //std::cout << imgHeight / 2 - detection.box.y - detection.box.height / 2 << endl;
#ifndef NO_IMAGE
        circle(image, Point(detection.box.x + detection.box.width / 2, detection.box.y + detection.box.height / 2), 3,
               Scalar(0, 0, 255), FILLED);

        putText(image, label, Point(detection.box.x, detection.box.y - baseLine),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
#endif

        position_container.push_back(position);
    }
    return position_container;
}
