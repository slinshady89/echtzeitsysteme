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