#ifndef PROJECT_TRANSFORMINGLANEDETECTOR_H
#define PROJECT_TRANSFORMINGLANEDETECTOR_H


#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <lane_detection/LaneDetector.h>
#include <algorithm>

using namespace cv;

/**
 * A lane detector which transforms the full received image in the 2D plane and filtering colors.
 * The coordinates for the lanes are found by traversing the rows in the transformed image in fixed distances.
 */
class TransformingLaneDetector : public LaneDetector {
public:
    TransformingLaneDetector(ImageProcessor& proc, LanePointsCalculator& lpc, CameraCalibration cal,
            int maxPointsPerLane);;

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
    void correctOuterLines(std::vector<Point2i>& leftTmp, std::vector<Point2i>& rightTmp, std::vector<Point2i>& middleLine);
    void sortPointsInDescendingYOrder(std::vector<Point2i>& points);

    /**
     * Traverses through the (already sorted) points and adds them to the right or left line,
     * dependent on if they lie left or right of the pink line.
     * For comparing the x-positions, the middle point is chosen such that it is the next one that is higher
     * (in terms of y-coordinates) than the currently considered point.
     * If two point are located in the same row, but too close together, only one of them is added to the
     * corresponding line.
     * @param points points to add to right or left lane (should be sorted in descending y-order)
     */
    void addPointsToLeftAndRightLane(std::vector<Point2i>& points);
    Mat morphologicalPreprocessing(Mat input);
};


#endif //PROJECT_TRANSFORMINGLANEDETECTOR_H
