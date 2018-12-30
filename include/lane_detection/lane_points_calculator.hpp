#ifndef LANE_POINTS_CALCULATOR_HPP_
#define LANE_POINTS_CALCULATOR_HPP_

#include <opencv2/opencv.hpp>

using namespace cv;

enum Direction { LEFT, RIGHT };

// TODO: make this class singleton?
class LanePointsCalculator {
    public:
        LanePointsCalculator() {};
        ~LanePointsCalculator() {};

        Point2i orthoPoint(Point2i start, Point2i end, Direction dir, double dist);

    private:

};




#endif // LANE_POINTS_CALCULATOR_HPP_