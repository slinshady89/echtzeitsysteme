#ifndef PROJECT_LANEDETECTOR_H
#define PROJECT_LANEDETECTOR_H

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <lane_detection/image_processor.hpp>
#include <lane_detection/lane_points_calculator.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

class LaneDetector {
public:
    LaneDetector(ImageProcessor& proc, LanePointsCalculator& lpc, CameraCalibration cal, int maxPointsPerLane)
        : proc(proc), lpc(lpc), cal(cal), maxPointsPerLane(maxPointsPerLane) {};

    virtual ~LaneDetector() = default;

    virtual void detectLanes(Mat& inputImage, Scalar& lowColorGreen,Scalar& highColorGreen,
                     Scalar& lowColorPink, Scalar& highColorPink) = 0;
    virtual std::vector<Point2d> getRightLane() = 0;
    virtual std::vector<Point2d> getLeftLane() = 0;
    virtual std::vector<Point2d> getMiddleLane() = 0;
    virtual Mat& getProcessedImage() = 0;
    virtual void publishProcessedImage(image_transport::Publisher publisher) = 0; // TODO: maybe extend with publisher for image information

protected:
    ImageProcessor& proc;
    LanePointsCalculator& lpc;
    CameraCalibration cal;
    int maxPointsPerLane;

    // processed image
    Mat processedImage;

    // rows where to look for points
    int* rows;
    int rowsCount;

    // detected lane points in image coordinates
    std::vector<Point2i> leftLanePoints_px;
    std::vector<Point2i> rightLanePoints_px;
    std::vector<Point2i> middleLanePoints_px;

    // flags that must be reset if a new image is used
    bool rightLaneDetected;
    bool leftLaneDetected;
    bool middleLaneDetected;

    std::vector<Point2i> filterForMaxWidthDistOfAdjacentPoints(std::vector<Point2i>& points, int maxDist_px);
};


#endif //PROJECT_LANEDETECTOR_H
