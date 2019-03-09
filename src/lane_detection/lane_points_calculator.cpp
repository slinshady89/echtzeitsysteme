
#include <lane_detection/image_processor.hpp>
#include <lane_detection/lane_points_calculator.hpp>

#include "lane_detection/lane_points_calculator.hpp"


Point2i LanePointsCalculator::orthoPoint(Point2i start, Point2i end, Direction dir, double dist) {
    Point2i middle = 0.5 * (start + end);
    Point2i startToEnd = end - start;

    int rotatedX, rotatedY;
    switch (dir) {
        case LEFT:
            rotatedX = -startToEnd.y;
            rotatedY = startToEnd.x;
            break;
        case RIGHT:
            rotatedX = startToEnd.y;
            rotatedY = -startToEnd.x;
            break;
    }
    double length = norm(startToEnd);
    Point2d normedToMiddle = (1.0/length) * Point2d(rotatedX, rotatedY);
    return middle + (Point2i)(dist * normedToMiddle);
}


Point2i LanePointsCalculator::innerLanePoint(Direction searchDir, int row, ImageProcessor& proc) {
    Point2i p;
    switch (searchDir) {
        case LEFT:
            p = proc.firstMatchFromRight(row);
            return proc.lastMatchLeft(p);
        case RIGHT:
            p = proc.firstMatchFromLeft(row);
            return proc.lastMatchRight(p);
    }
}

std::vector<Point2i> LanePointsCalculator::trajPointsSimple(int *rows, int rows_size, Direction searchDir, SearchMode searchMode, int distance, ImageProcessor& proc) {
    std::vector<Point2i> results;
    Point2i firstLanePoint, secondLanePoint; 
    for (int i=0; i<rows_size; i++) {
        firstLanePoint = innerLanePoint(searchDir, rows[i], proc);
        if (searchMode!=FIXED_DIST) {
            Direction opposite = (searchDir==LEFT) ? RIGHT : LEFT;
            secondLanePoint = innerLanePoint(opposite, rows[i], proc);
            // check if the two points are really from different lanes
            if ((searchDir==LEFT && firstLanePoint.x>secondLanePoint.x) || (searchDir==RIGHT && firstLanePoint.x<secondLanePoint.x)) {
                results.push_back(0.5 * (firstLanePoint+secondLanePoint));
                continue;
            }
            if (searchMode==MIDDLE) {
                continue;
            }
        }
        int offset = (searchDir==LEFT) ? -distance : distance;
        // only add point to the results if it is still in image range
        int x = firstLanePoint.x + offset;
        if (x>=0 && x<proc.getImage().cols) {
            results.push_back(Point2i(x,firstLanePoint.y));
        }
    }
    return results;
}

std::vector<Point2i> LanePointsCalculator::lanePoints(int *rows, int rows_size, Direction searchDir, ImageProcessor &proc) {
    std::vector<Point2i> results;
    for (int i=0; i<rows_size; i++) {
        Point2i foundPoint = innerLanePoint(searchDir, rows[i], proc);
        if (foundPoint.x >=0 && foundPoint.y >=0) {
            results.push_back(foundPoint);
        }
    }
    return results;
}

Point2d LanePointsCalculator::singleTrajPoint(double rightLaneDist_cm, double y_cm, int colorThreshold, Mat& image, CameraCalibration& cal) {
    int pxHeight = (y_cm - cal.getOffset_cm()) * cal.getHeight_px_per_cm();
    int pxDistLane = int(rightLaneDist_cm * cal.getWidth_px_per_cm());

    int y = image.rows - pxHeight;
    int width = image.cols;
    if (y>=0 && y < image.rows) { // validity check
        Mat imageRow = image.row(y);
        for (int i=width-1; i>=0; i--) {
            Scalar pixel = imageRow.at<uchar>(0,i);

            if (pixel.val[0]>colorThreshold) {
                return Point2d(i-pxDistLane,y);
            }
        }
    }
    std::cerr << "No trajectory point found." << std::endl;
    return Point2d(-1,-1);
}