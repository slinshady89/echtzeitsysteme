#ifndef PROJECT_TRANSFORMINGLANEDETECTOR_H
#define PROJECT_TRANSFORMINGLANEDETECTOR_H


#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <lane_detection/LaneDetector.h>

using namespace cv;

/**
 * A lane detector which transforms the full received image in the 2D plane.
 * The coordinates for the lanes are found by traversing the rows in the transformed image in fixed distances.
 */
class TransformingLaneDetector : public LaneDetector {
public:
    TransformingLaneDetector(ImageProcessor& proc, LanePointsCalculator& lpc, CameraCalibration cal,
            Mat& inputImage, int maxPointsPerLane);;

    void detectLanes(Mat& inputImage, Scalar& lowColorGreen,Scalar& highColorGreen,
            Scalar& lowColorPink, Scalar& highColorPink) override;
    std::vector<Point2d> getRightLane() override;;
    std::vector<Point2d> getLeftLane() override;;
    std::vector<Point2d> getMiddleLane() override;;
    Mat& getProcessedImage() override {

    };
    void publishProcessedImage(image_transport::Publisher publisher) override;;
private:
    int maxDistBetweenAdjacentPoints_px;

};


#endif //PROJECT_TRANSFORMINGLANEDETECTOR_H
