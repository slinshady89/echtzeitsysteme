#ifndef LANE_POINTS_CALCULATOR_HPP_
#define LANE_POINTS_CALCULATOR_HPP_

#include <opencv2/opencv.hpp>
#include "lane_detection/image_processor.hpp"

using namespace cv;

enum Direction { LEFT, RIGHT };
enum SearchMode { FIXED_DIST, MIDDLE, MIDDLE_FALLBACK }

// TODO: make this class singleton?
class LanePointsCalculator {
    public:
        LanePointsCalculator() {};
        ~LanePointsCalculator() {};

        Point2i orthoPoint(Point2i start, Point2i end, Direction dir, double dist);
        Point2i innerLanePoint(Direction searchDir, int row, ImageProcessor& proc);
        /*  Searches in the given rows for lane pixels and calculates trajectory points from these.
            searchDir specifies from which side the search is done (LEFT for finding the right, RIGHT for finding the left lane first) 
            searchMode: 
                FIXED_DIST calculates all points with a fixed distance from the first found lane
                MIDDLE only finds points for rows where both lanes can be detected
                MIDDLE_FALLBACK prefers calculating middle points and uses the distance as a fallback
        */
        std::vector<Point2i> trajPointsSimple(int *rows, int rows_size, Direction searchDir, SearchMode searchMode, int distance, ImageProcessor& proc);

    private:

};




#endif // LANE_POINTS_CALCULATOR_HPP_