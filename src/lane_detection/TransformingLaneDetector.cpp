#include <lane_detection/TransformingLaneDetector.h>

TransformingLaneDetector::TransformingLaneDetector(ImageProcessor &proc, LanePointsCalculator &lpc,
                                                   CameraCalibration cal, int maxPointsPerLane)
        : LaneDetector(proc, lpc, cal, maxPointsPerLane) {
    // calculate the rows where to search for points
    int outputImageHeight = cal.getDstHeight();
    int distBetweenRows = outputImageHeight / maxPointsPerLane;
    int firstRowOffset = distBetweenRows/2;
    rows = new int[maxPointsPerLane];
    rowsCount = maxPointsPerLane;
    for (int i=0; i<maxPointsPerLane; i++) {
        rows[maxPointsPerLane-1-i] = firstRowOffset + i*distBetweenRows;
    }

    maxDistBetweenAdjacentPoints_px = cal.getDstWidth()/3;

}

void TransformingLaneDetector::detectLanes(Mat &inputImage, Scalar &lowColorGreen, Scalar &highColorGreen,
                                           Scalar &lowColorPink, Scalar &highColorPink) {
    proc.setImage(inputImage, BGR);
    proc.transformTo2D();
    Mat transformed = proc.convertToHSV();

    std::vector<std::vector<Point> > contoursGreen, contoursPink;
    std::vector<Vec4i> hierarchyGreen, hierarchyPink;

    // filter green lines
    Mat greenLines = proc.filterColor(lowColorGreen, highColorGreen);
    findContours( greenLines, contoursGreen, hierarchyGreen, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    int greenLargest(0);
    int green2ndLargest(0);
    int greenLargestSize(0);
    int green2ndLargestSize(0);


    for(int i = 0; i < contoursGreen.size(); i++){
        auto size = contoursGreen.at(i).size();
        if (size > greenLargestSize) {
            green2ndLargestSize = greenLargestSize;
            greenLargestSize = static_cast<int>(size);
            green2ndLargest = greenLargest;
            greenLargest = i;
        } else if (size > green2ndLargestSize) {
            green2ndLargestSize = static_cast<int>(size);
            green2ndLargest = i;
        }
    }
    if(greenLargest >= contoursGreen.size() || green2ndLargest >= contoursGreen.size()) {
        ROS_INFO("greenLargestSize.size: %d", contoursGreen.size());
    }else {

        double largestGreenMeanX_PX(0);
        for (auto it: contoursGreen.at(greenLargest)) largestGreenMeanX_PX += it.x;
        largestGreenMeanX_PX = largestGreenMeanX_PX / greenLargestSize;
        double secondLargestGreenMeanX_PX(0);
        for (auto it: contoursGreen.at(green2ndLargest)) secondLargestGreenMeanX_PX += it.x;
        secondLargestGreenMeanX_PX = secondLargestGreenMeanX_PX / green2ndLargestSize;

        bool largestIsRight = false;
        if (largestGreenMeanX_PX > secondLargestGreenMeanX_PX) largestIsRight = true;

        rightLanePoints_px.erase(rightLanePoints_px.begin(), rightLanePoints_px.end());
        leftLanePoints_px.erase(leftLanePoints_px.begin(), leftLanePoints_px.end());

        if (largestIsRight) {
            if (contoursGreen.at(greenLargest).size() > 10) {
                auto it = contoursGreen.at(greenLargest).size() / 8;
                for (int j = 0; j < contoursGreen.at(greenLargest).size() / 2;) {
                    rightLanePoints_px.emplace_back(contoursGreen.at(greenLargest).at(j));
                    j = j + it;
                }
            } else {
                ROS_INFO("Wait until the first frame has been received...");
                rightLanePoints_px = (contoursGreen.at(greenLargest));
            }
            if (contoursGreen.at(green2ndLargest).size() > 10) {
                auto it = contoursGreen.at(green2ndLargest).size() / 8;
                for (int j = 0; j < contoursGreen.at(green2ndLargest).size() / 2;) {
                    leftLanePoints_px.emplace_back(contoursGreen.at(green2ndLargest).at(j));
                    j = j + it;
                }
            } else {
                leftLanePoints_px = (contoursGreen.at(green2ndLargest));
            }
        } else {
            if (contoursGreen.at(greenLargest).size() > 10) {
                auto it = contoursGreen.at(greenLargest).size() / 8;
                for (int j = 0; j < contoursGreen.at(greenLargest).size() / 2;) {
                    leftLanePoints_px.emplace_back(contoursGreen.at(greenLargest).at(j));
                    j = j + it;
                }
            } else {
                leftLanePoints_px = (contoursGreen.at(greenLargest));
            }
            if (contoursGreen.at(green2ndLargest).size() > 10) {
                auto it = contoursGreen.at(green2ndLargest).size() / 8;
                for (int j = 0; j < contoursGreen.at(green2ndLargest).size() / 2;) {
                    rightLanePoints_px.emplace_back(contoursGreen.at(green2ndLargest).at(j));
                    j = j + it;
                }
            } else {
                rightLanePoints_px = (contoursGreen.at(green2ndLargest));
            }
        }
        contoursGreen.erase(contoursGreen.begin(), contoursGreen.end());

        sortPointsInDescendingYOrder(rightLanePoints_px);
        sortPointsInDescendingYOrder(leftLanePoints_px);
        proc.setImage(greenLines, GREY);


        contoursGreen.erase(contoursGreen.begin(), contoursGreen.end());
    }
        /*
    // detect right and left line points and combine them in one sorted vector
    std::vector<Point2i> rightTmp = lpc.lanePoints(rows,rowsCount, LEFT, proc);
    std::vector<Point2i> leftRightTemp = lpc.lanePoints(rows, rowsCount, RIGHT, proc);
    leftRightTemp.insert(leftRightTemp.end(), rightTmp.begin(), rightTmp.end());
    sortPointsInDescendingYOrder(leftRightTemp);

    // filter middle lane
    proc.setImage(transformed, HSV);
    Mat pinkLine = proc.filterColor(lowColorPink, highColorPink);
    middleLanePoints_px = lpc.lanePoints(rows, rowsCount, LEFT, proc);

    // add points to the vectors of the right / left line according to their relative position to the middle line
    addPointsToLeftAndRightLane(leftRightTemp);
    proc.setImage(pinkLine + greenLines, GREY);

         */

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
    publisher.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, proc.getImage()).toImageMsg());
}

void TransformingLaneDetector::correctOuterLines(std::vector<Point2i> &leftTmp, std::vector<Point2i> &rightTmp,
                                                 std::vector<Point2i> &middleLine) {
    rightLanePoints_px.clear();
    leftLanePoints_px.clear();
    int indexLeft = 0;
    int indexRight = 0;
    int indexMiddle = 0;

    if (middleLine.size()==0) return;
    int currentCenterY = middleLine.at(0).y;
    while (indexLeft<leftTmp.size() || indexRight<rightTmp.size()) {

        if (indexLeft < leftTmp.size()) {
            int leftY = leftTmp.at(indexLeft).y;
            if (leftY > middleLine.at(indexMiddle).y) {

            }

            indexLeft++;
        }
        if (indexRight < rightTmp.size()) {

            indexRight++;
        }
    }


    if (indexMiddle < middleLine.size()) {
    }

}


bool firstGreaterSecond (Point2i first,Point2i second) { return (first.y>second.y); }
void TransformingLaneDetector::sortPointsInDescendingYOrder(std::vector<Point2i>& points) {
    std::sort(points.begin(), points.end(), firstGreaterSecond);
}

void TransformingLaneDetector::addPointsToLeftAndRightLane(std::vector<Point2i> &points) {
    if (middleLanePoints_px.empty()) return;
    leftLanePoints_px.clear();
    rightLanePoints_px.clear();
    int middleIndex = 0;
    int middleIndexMax = middleLanePoints_px.size()-1;
    int mY = middleLanePoints_px.at(0).y;
    for (auto point : points) {
        //ensure that center point has similar or higher y value for comparison
        while (middleIndex < middleIndexMax && point.y < mY) {
            middleIndex++;
            mY = middleLanePoints_px.at(middleIndex).y;
        }
        //append point to right or left line
        auto mX =  middleLanePoints_px.at(middleIndex).x;
        // check if right of centerLine then add to right line
        if(point.x > mX){
            if(!rightLanePoints_px.empty() && point.y == rightLanePoints_px.back().y) {
                if (point.x < rightLanePoints_px.back().x)
                    rightLanePoints_px.at(rightLanePoints_px.size() - 1) = point;
            }else {
                if (point.y < mY) {
                    // don't add point if point is too close to last point of other line
                    if (!leftLanePoints_px.empty() && abs(leftLanePoints_px.back().x - point.x)<maxDistBetweenAdjacentPoints_px)
                        continue;
                }
                rightLanePoints_px.emplace_back(point);
            }
        }
            // else add to left line
        else{
            if(!leftLanePoints_px.empty() && point.y == leftLanePoints_px.back().y) {
                if (point.x > leftLanePoints_px.back().x)
                    leftLanePoints_px.at(leftLanePoints_px.size() - 1) = point;
            }else {
                if (point.y < mY) {
                    // don't add point if point is too close to last point of other line
                    if (!rightLanePoints_px.empty() && abs(rightLanePoints_px.back().x - point.x)<maxDistBetweenAdjacentPoints_px)
                        continue;
                }
                leftLanePoints_px.emplace_back(point);
            }
        }

    }
}

Mat TransformingLaneDetector::morphologicalPreprocessing(Mat input) {
    Mat output, morph_img;
    /* morph_operator
     * Opening: MORPH_OPEN : 2
     * Closing: MORPH_CLOSE: 3
     * Gradient: MORPH_GRADIENT: 4
     * Top Hat: MORPH_TOPHAT: 5
     * Black Hat: MORPH_BLACKHAT: 6
     */
    int morph_operator = 2;
    /* morph_elem
     * 0: Rect
     * 1: Cross
     * 2: Ellipse
     * */
    int morph_elem = 0;
    /* morph_size
     * 1 : matrix 3x3
     * 2 : matrix 5x5
     * n : matrix 2n+1x2n+1
     * */
    int morph_size = 1;
    // processes opening with rectangle object
    cv::Mat element = cv::getStructuringElement( morph_elem, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
    cv::morphologyEx( input, morph_img, morph_operator, element );
    morph_operator = 3;
    morph_elem = 0;
    morph_size = 4;
    // processes closing with rectangle object
    element = cv::getStructuringElement( morph_elem, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
    cv::morphologyEx( morph_img, output, morph_operator, element );

    return output;
}
