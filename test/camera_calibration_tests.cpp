#include <lane_detection/CameraCalibration.hpp>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <constants/constants.h>
#include <opencv2/opencv.hpp>

/**
 * Tests for proving the correctness of the three coordinate transformation functions.
 */

using namespace constants;

const int TARGET_WIDTH = 120;
const int TARGET_HEIGHT = 200;
const int TARGET_PX_PER_CM = 5;

const double RECT_WIDTH =  59.0;
// width in px (2D) should be 420
const double RECT_HEIGHT = 84.0;
// height in px (2D) should be 295
const double OFFSET_ORIGIN = 37.0;
// offset in px (2D) should be 0

const Point2i expected_bottomLeft2D(153,999);
const Point2i expected_bottomRight2D(447,999);
const Point2i expected_topRight2D(447,579);
const Point2i expected_topLeft2D(153,579);

const Point2i expected_bottomLeft3D(345,513);
const Point2i expected_bottomRight3D(918,521);
const Point2i expected_topRight3D(747,311);
const Point2i expected_topLeft3D(522,308);

const Point2d expected_bottomLeftWorld(37.0,29.5);
const Point2d expected_bottomRightWorld(37.0,-29.5);
const Point2d expected_topRightWorld(121.0,-29.5);
const Point2d expected_topLeftWorld(121.0,29.5);


const double acceptedWorldError = 1.0;
const double acceptedPixelError = 2.0;  // TODO: somewhere single pixel errors occur - this should not happen

// test fixture for camera calibration object
class CameraCalibrationTest : public ::testing::Test {
protected:
    CameraCalibrationTest() : cal(
            CameraCalibration(
                    RECT_WIDTH, RECT_HEIGHT, OFFSET_ORIGIN,
                    TARGET_WIDTH, TARGET_HEIGHT,
                    expected_bottomLeft3D, expected_bottomRight3D, expected_topRight3D, expected_topLeft3D,
                    TARGET_PX_PER_CM)
                ){}
    void SetUp() override {
    }

    CameraCalibration cal;
};



TEST_F(CameraCalibrationTest, correctWorldCoordinatesFrom2DImageCoordinates) {
    Point2d actual_bottomLeftWorld = cal.getWorldCoordinatesFrom2DImageCoordinates(expected_bottomLeft2D);
    Point2d actual_bottomRightWorld = cal.getWorldCoordinatesFrom2DImageCoordinates(expected_bottomRight2D);
    Point2d actual_topRightWorld = cal.getWorldCoordinatesFrom2DImageCoordinates(expected_topRight2D);
    Point2d actual_topLeftWorld = cal.getWorldCoordinatesFrom2DImageCoordinates(expected_topLeft2D);
    EXPECT_NEAR(actual_bottomLeftWorld.x, expected_bottomLeftWorld.x, acceptedWorldError);
    EXPECT_NEAR(actual_bottomLeftWorld.y, expected_bottomLeftWorld.y, acceptedWorldError);
    EXPECT_NEAR(actual_bottomRightWorld.x, expected_bottomRightWorld.x, acceptedWorldError);
    EXPECT_NEAR(actual_bottomRightWorld.y, expected_bottomRightWorld.y, acceptedWorldError);
    EXPECT_NEAR(actual_topRightWorld.x, expected_topRightWorld.x, acceptedWorldError);
    EXPECT_NEAR(actual_topRightWorld.y, expected_topRightWorld.y, acceptedWorldError);
    EXPECT_NEAR(actual_topLeftWorld.x, expected_topLeftWorld.x, acceptedWorldError);
    EXPECT_NEAR(actual_topLeftWorld.y, expected_topLeftWorld.y, acceptedWorldError);
}

TEST_F(CameraCalibrationTest, correct2DImageCoordinatesFromWorldCoordinates) {
    Point2i actual_bottomLeft2D = cal.get2DImageCoordinatesFromWorldCoordinates(expected_bottomLeftWorld);
    Point2i actual_bottomRight2D = cal.get2DImageCoordinatesFromWorldCoordinates(expected_bottomRightWorld);
    Point2i actual_topRight2D = cal.get2DImageCoordinatesFromWorldCoordinates(expected_topRightWorld);
    Point2i actual_topLeft2D = cal.get2DImageCoordinatesFromWorldCoordinates(expected_topLeftWorld);
    EXPECT_NEAR(actual_bottomLeft2D.x, expected_bottomLeft2D.x, acceptedPixelError);
    EXPECT_NEAR(actual_bottomLeft2D.y, expected_bottomLeft2D.y, acceptedPixelError);
    EXPECT_NEAR(actual_bottomRight2D.x, expected_bottomRight2D.x, acceptedPixelError);
    EXPECT_NEAR(actual_bottomRight2D.y, expected_bottomRight2D.y, acceptedPixelError);
    EXPECT_NEAR(actual_topRight2D.x, expected_topRight2D.x, acceptedPixelError);
    EXPECT_NEAR(actual_topRight2D.y, expected_topRight2D.y, acceptedPixelError);
    EXPECT_NEAR(actual_topLeft2D.x, expected_topLeft2D.x, acceptedPixelError);
    EXPECT_NEAR(actual_topLeft2D.y, expected_topLeft2D.y, acceptedPixelError);
}

TEST_F(CameraCalibrationTest, correct3DImageCoordinatesFrom2DImageCoordinates) {
    std::vector<Point2i> inputPoints2D{expected_bottomLeft2D, expected_bottomRight2D, expected_topRight2D, expected_topLeft2D};
    std::vector<Point2i> actualPoints3D = cal.get3DFrom2DImageCoordinates(inputPoints2D);
    EXPECT_NEAR(actualPoints3D.at(0).x, expected_bottomLeft3D.x, acceptedPixelError);
    EXPECT_NEAR(actualPoints3D.at(0).y, expected_bottomLeft3D.y, acceptedPixelError);
    EXPECT_NEAR(actualPoints3D.at(1).x, expected_bottomRight3D.x, acceptedPixelError);
    EXPECT_NEAR(actualPoints3D.at(1).y, expected_bottomRight3D.y, acceptedPixelError);
    EXPECT_NEAR(actualPoints3D.at(2).x, expected_topRight3D.x, acceptedPixelError);
    EXPECT_NEAR(actualPoints3D.at(2).y, expected_topRight3D.y, acceptedPixelError);
    EXPECT_NEAR(actualPoints3D.at(3).x, expected_topLeft3D.x, acceptedPixelError);
    EXPECT_NEAR(actualPoints3D.at(3).y, expected_topLeft3D.y, acceptedPixelError);
}

TEST_F(CameraCalibrationTest, correct2DImageCoordinatesFrom3DImageCoordinates) {
    std::vector<Point2i> inputPoints3D{expected_bottomLeft3D, expected_bottomRight3D, expected_topRight3D, expected_topLeft3D};
    std::vector<Point2i> actualPoints2D = cal.get2DFrom3DImageCoordinates(inputPoints3D);
    EXPECT_NEAR(actualPoints2D.at(0).x, expected_bottomLeft2D.x, acceptedPixelError);
    EXPECT_NEAR(actualPoints2D.at(0).y, expected_bottomLeft2D.y, acceptedPixelError);
    EXPECT_NEAR(actualPoints2D.at(1).x, expected_bottomRight2D.x, acceptedPixelError);
    EXPECT_NEAR(actualPoints2D.at(1).y, expected_bottomRight2D.y, acceptedPixelError);
    EXPECT_NEAR(actualPoints2D.at(2).x, expected_topRight2D.x, acceptedPixelError);
    EXPECT_NEAR(actualPoints2D.at(2).y, expected_topRight2D.y, acceptedPixelError);
    EXPECT_NEAR(actualPoints2D.at(3).x, expected_topLeft2D.x, acceptedPixelError);
    EXPECT_NEAR(actualPoints2D.at(3).y, expected_topLeft2D.y, acceptedPixelError);
}

TEST_F(CameraCalibrationTest, correctWorldCoordinatesFrom3DImageCoordinates) {
    std::vector<Point2i> inputPoints3D{expected_bottomLeft3D, expected_bottomRight3D, expected_topRight3D, expected_topLeft3D};
    std::vector<Point2d> actualWorldCoordinates = cal.getWorldCoordinatesFrom3DImageCoordinates(inputPoints3D);
    EXPECT_NEAR(actualWorldCoordinates.at(0).x, expected_bottomLeftWorld.x, acceptedPixelError);
    EXPECT_NEAR(actualWorldCoordinates.at(0).y, expected_bottomLeftWorld.y, acceptedPixelError);
    EXPECT_NEAR(actualWorldCoordinates.at(1).x, expected_bottomRightWorld.x, acceptedPixelError);
    EXPECT_NEAR(actualWorldCoordinates.at(1).y, expected_bottomRightWorld.y, acceptedPixelError);
    EXPECT_NEAR(actualWorldCoordinates.at(2).x, expected_topRightWorld.x, acceptedPixelError);
    EXPECT_NEAR(actualWorldCoordinates.at(2).y, expected_topRightWorld.y, acceptedPixelError);
    EXPECT_NEAR(actualWorldCoordinates.at(3).x, expected_topLeftWorld.x, acceptedPixelError);
    EXPECT_NEAR(actualWorldCoordinates.at(3).y, expected_topLeftWorld.y, acceptedPixelError);
}

TEST_F(CameraCalibrationTest, correct3DImageCoordinatesFromWorldCoordinates) {
    std::vector<Point2d> inputPointsWorld{expected_bottomLeftWorld, expected_bottomRightWorld, expected_topRightWorld, expected_topLeftWorld};
    std::vector<Point2i> actualPoints3D = cal.get3DImageCoordinatesFromWorldCoordinates(inputPointsWorld);
    EXPECT_NEAR(actualPoints3D.at(0).x, expected_bottomLeft3D.x, acceptedPixelError);
    EXPECT_NEAR(actualPoints3D.at(0).y, expected_bottomLeft3D.y, acceptedPixelError);
    EXPECT_NEAR(actualPoints3D.at(1).x, expected_bottomRight3D.x, acceptedPixelError);
    EXPECT_NEAR(actualPoints3D.at(1).y, expected_bottomRight3D.y, acceptedPixelError);
    EXPECT_NEAR(actualPoints3D.at(2).x, expected_topRight3D.x, acceptedPixelError);
    EXPECT_NEAR(actualPoints3D.at(2).y, expected_topRight3D.y, acceptedPixelError);
    EXPECT_NEAR(actualPoints3D.at(3).x, expected_topLeft3D.x, acceptedPixelError);
    EXPECT_NEAR(actualPoints3D.at(3).y, expected_topLeft3D.y, acceptedPixelError);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
