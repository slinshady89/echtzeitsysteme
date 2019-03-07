#ifndef PROJECT_NONTRANSFORMINGLANEDETECTOR_H
#define PROJECT_NONTRANSFORMINGLANEDETECTOR_H

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>

using namespace cv;

class NonTransformingLaneDetector : public LaneDetector {
public:
    NonTransformingLaneDetector(ImageProcessor& proc, LanePointsCalculator& lpc, CameraCalibration cal, Mat& inputImage);

    virtual std::vector<Point2d> getRightLane() = 0;
    virtual std::vector<Point2d> getLeftLane() = 0;
    virtual std::vector<Point2d> getMiddleLane() = 0;
    virtual Mat& getProcessedImage() = 0;
    virtual void publishProcessedImage(image_transport::Publisher publisher) = 0;
};
#endif //PROJECT_NONTRANSFORMINGLANEDETECTOR_H
