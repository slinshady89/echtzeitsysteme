#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <lane_detection/image_processor.hpp>
#include <lane_detection/lane_points_calculator.hpp>
#include <stdio.h>
#include <echtzeitsysteme/ImageProcessingConfig.h>
#include <dynamic_reconfigure/server.h>
#include <time.h>
#include <constants/constants.h>

using namespace cv;
using namespace constants::calibrations;

#define SHOW_IMAGES
#define TEST_PICTURE_PATH "./src/echtzeitsysteme/images/my_photo-2.jpg"


#define USE_TEST_PICTURE

// my_photo-2.jpg
#define PARAMS_3 59.0,84.0,20,180,180,Point(549,799),Point(1384,786),Point(1129,490),Point(800,493),5

const int TARGET_WIDTH = 120;
const int TARGET_HEIGHT = 200;
const int TARGET_PX_PER_CM = 5;
const int LOOP_RATE_IN_HERTZ = 10;

const Scalar RED = Scalar(0,0,255);
const Scalar GREEN = Scalar(0,255,0);

int IMAGE_ROWS[] = {100, 200, 300, 400, 500, 600, 700, 800, 900};
int IMAGE_ROWS_SIZE = 9;

/* configuration parameters */
int green_low_H, green_low_S, green_low_V, green_high_H, green_high_S, green_high_V;
double y_dist_cm, lane_dist_cm;
int loop_rate;
int laneColorThreshold;

Mat processImage(ImageProcessor& proc, LanePointsCalculator& lpc);

void configCallback(echtzeitsysteme::ImageProcessingConfig &config, uint32_t level) {
    green_low_H = config.green_low_H;
    green_low_S = config.green_low_S;
    green_low_V = config.green_low_V;
    green_high_H = config.green_high_H;
    green_high_S = config.green_high_S;
    green_high_V = config.green_high_V;
    y_dist_cm = config.y_dist_cm;
    lane_dist_cm = config.lane_dist_cm;
    loop_rate = config.loop_rate;
    laneColorThreshold = config.colorThreshold;

    ROS_INFO("Updated configuration.");
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "show_multiple_lane_points");

    // initialize this node
    ros::NodeHandle nh;

    Mat sourceImage;

    // connect the ROS rqt_reconfigure node with the callback to set parameters
    dynamic_reconfigure::Server<echtzeitsysteme::ImageProcessingConfig> server;
    dynamic_reconfigure::Server<echtzeitsysteme::ImageProcessingConfig>::CallbackType f;
    f = boost::bind(&configCallback, _1, _2);
    server.setCallback(f);

    // read test picture from file sytem
    sourceImage = imread(TEST_PICTURE_PATH, IMREAD_COLOR);
    if (sourceImage.empty())
    {
        ROS_ERROR("Test image could not be opened!");
    }

    // TODO: move pwd logic to util function
    ROS_INFO("show multiple lane points");
    char dir_name[100];
    getcwd(dir_name, 100);
    ROS_INFO("Current directory is: %s", dir_name);


    CameraCalibration calibration(
            myphoto2::RECT_WIDTH, myphoto2::RECT_HEIGHT, myphoto2::OFFSET_ORIGIN,
            TARGET_WIDTH, TARGET_HEIGHT,
            myphoto2::BOTTOM_LEFT, myphoto2::BOTTOM_RIGHT, myphoto2::TOP_RIGHT, myphoto2::TOP_LEFT,
            TARGET_PX_PER_CM
    );
    ImageProcessor imageProcessor(sourceImage, BGR, calibration);
    LanePointsCalculator& lanePointsCalculator = LanePointsCalculator::getInstance();
    ROS_INFO("Calibrated camera image.");
    // show source image in window "SourceImage"
    imshow("SourceImage", sourceImage);

    ros::Rate loop_rate(LOOP_RATE_IN_HERTZ); //TODO: Hz anpassen

    while (ros::ok())
    {
        ROS_INFO("begin of loop iteration");

        // reset image to source image for a new processing
        imageProcessor.setImage(sourceImage, BGR);
        processImage(imageProcessor, lanePointsCalculator);

        waitKey(100);

        // call subscription, services, other callbacks
        ros::spinOnce();

        // this is needed to ensure a const. loop rate
        loop_rate.sleep();
    }

    return 0;
}

Mat processImage(ImageProcessor& proc, LanePointsCalculator& lpc)
{
    Mat output;
    output = proc.transformTo2D();
    imshow("perspective transformed", output);

    proc.convertToHSV();
    output = proc.filterColor(Scalar(green_low_H, green_low_S, green_low_V),Scalar(green_high_H, green_high_S, green_high_V));
    imshow("colors filtered", output);

    std::vector<Point2i> rightLanePoints = lpc.lanePoints(IMAGE_ROWS, IMAGE_ROWS_SIZE, LEFT, proc);
    std::vector<Point2i> leftLanePoints = lpc.lanePoints(IMAGE_ROWS, IMAGE_ROWS_SIZE, RIGHT, proc);

    Mat coloredOutput;
    cvtColor(output, coloredOutput, COLOR_GRAY2BGR);
    proc.setImage(coloredOutput, BGR);
    for (int i=0; i<rightLanePoints.size(); i++) {
        coloredOutput = proc.drawPoint(rightLanePoints.at(i), Scalar(0,0,255));
    }
    for (int i=0; i<leftLanePoints.size(); i++) {
        coloredOutput = proc.drawPoint(leftLanePoints.at(i), Scalar(0,255,0));
    }
    imshow("colored", coloredOutput);

    return output;
}

