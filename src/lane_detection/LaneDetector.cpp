#include <lane_detection/LaneDetector.h>

std::vector<Point2i> LaneDetector::filterForMaxWidthDistOfAdjacentPoints(std::vector<Point2i> &points, int maxDist_px) {
    if (points.empty()) return std::vector<Point2i>();
    std::vector<Point2i> filtered;
    Point2i lastPoint = points.at(0);
    for (auto it:points) {
        if (abs(it.x - it.y) <= maxDist_px) {
            filtered.emplace_back(it);
        }
        lastPoint = it;
    }
    return filtered;
}
