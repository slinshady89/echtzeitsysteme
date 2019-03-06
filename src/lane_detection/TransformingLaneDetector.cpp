#include <lane_detection/TransformingLaneDetector.h>

TransformingLaneDetector::TransformingLaneDetector(ImageProcessor &proc, LanePointsCalculator &lpc,
                                                   CameraCalibration cal, Mat &inputImage, int maxPointsPerLane)
        : LaneDetector(proc, lpc, cal, inputImage, maxPointsPerLane) {
    // calculate the rows where to search for points
    int outputImageHeight = cal.getDstHeight();
    int distBetweenRows = outputImageHeight / maxPointsPerLane;
    int firstRowOffset = distBetweenRows/2;
    rows = new int[maxPointsPerLane];
    for (int i=0; i<maxPointsPerLane; i++) {
        rows[i] = firstRowOffset + i*distBetweenRows;
    }

    // FIXME: dirty solution
    maxDistBetweenAdjacentPoints_px = cal.getDstWidth()/3;

}

void TransformingLaneDetector::detectLanes(Mat &inputImage, Scalar &lowColorGreen, Scalar &highColorGreen,
                                           Scalar &lowColorPink, Scalar &highColorPink) {
    proc.setImage(inputImage, BGR);
    Mat transformed = proc.transformTo2D();
    // filter green lines
    Mat processed = proc.filterColor(lowColorGreen, highColorGreen);

    // detect right and left line
    rightLanePoints_px = lpc.lanePoints(rows, rowsCount, LEFT, proc);
    leftLanePoints_px = lpc.lanePoints(rows, rowsCount, RIGHT, proc);
    middleLanePoints_px = std::vector<Point2i>(); // TODO implement
}

std::vector<Point2d> TransformingLaneDetector::getRightLane() {
    rightLanePoints_px = filterForMaxWidthDistOfAdjacentPoints(rightLanePoints_px, maxDistBetweenAdjacentPoints_px);
    return cal.getWorldCoordinatesFrom2DImageCoordinates(rightLanePoints_px);
}

std::vector<Point2d> TransformingLaneDetector::getLeftLane() {
    leftLanePoints_px = filterForMaxWidthDistOfAdjacentPoints(leftLanePoints_px, maxDistBetweenAdjacentPoints_px);
    return cal.getWorldCoordinatesFrom2DImageCoordinates(leftLanePoints_px);
}

std::vector<Point2d> TransformingLaneDetector::getMiddleLane() {
    middleLanePoints_px = filterForMaxWidthDistOfAdjacentPoints(middleLanePoints_px, maxDistBetweenAdjacentPoints_px);
    return cal.getWorldCoordinatesFrom2DImageCoordinates(middleLanePoints_px);
}

void TransformingLaneDetector::publishProcessedImage(image_transport::Publisher publisher) {
    // fully processed image is greyscale and 2D
    // TODO: okay to publish from the image's reference?
    publisher.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, proc.getImage()).toImageMsg());
}
