#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <echtzeitsysteme/points.h>
#include <opencv2/opencv.hpp>
#include <lane_detection/CameraReader.hpp>
#include <lane_detection/image_processor.hpp>
#include <stdio.h>
#include <echtzeitsysteme/ImageProcessingConfig.h>
#include <dynamic_reconfigure/server.h>
#include <lane_detection/lane_points_calculator.hpp>
#include <constants/constants.h>

using namespace cv;
using namespace constants::calibrations;

#define SHOW_IMAGES

//#define TEST_PICTURE_PATH "camera_reading_test/images/calibration_test_2.jpg"
//#define TEST_PICTURE_PATH "camera_reading_test/images/track_straight.jpg"
//#define TEST_PICTURE_PATH "/home/pses/catkin_ws/src/echtzeitsysteme/include/lane_detection/images/calibration_test_2.jpg"
//#define TEST_PICTURE_PATH "echtzeitsysteme/include/lane_detection/images/2018-12-05-220157.jpg"

// NOTE: run from inside "catkin_ws" folder to find test photo
#define TEST_PICTURE_PATH "./src/echtzeitsysteme/images/my_photo-2.jpg"

#define USE_TEST_PICTURE
//#define DRAW_GRID

#define PARAMS_1 59.0, 84.0, 30.0, 640, 480, Point(0, 366), Point(632, 363), Point(404, 238), Point(237, 237), Point(151, 639), Point(488, 639), Point(488, 0), Point(151, 0)
#define PARAMS_2 59.0, 84.0, 20, 640, 991, Point(43, 387), Point(583, 383), Point(404, 189), Point(234, 190), Point(95, 990), Point(545, 990), Point(545, 350), Point(95, 350)

// my_photo-2.jpg
#define PARAMS_3 59.0, 84.0, 20, 180, 180, Point(549, 799), Point(1384, 786), Point(1129, 490), Point(800, 493), 5
// photo from 15.12.
#define PARAMS_4 59.0, 84.0, 22, 180, 180, Point(384, 895), Point(1460, 900), Point(1128, 472), Point(760, 460), 5

int IMAGE_ROWS[] = {100, 200, 300, 400, 500, 600, 700, 800, 900};
int IMAGE_ROWS_SIZE = 9;

const int TARGET_WIDTH = 120;
const int TARGET_HEIGHT = 200;
const int TARGET_PX_PER_CM = 5;
const int LOOP_RATE_IN_HERTZ = 10;

/* configuration parameters */
int low_H, low_S, low_V, high_H, high_S, high_V;
double y_dist_cm, lane_dist_cm;
int loop_rate;
int laneColorThreshold;

Mat processImage(Mat input, ImageProcessor &proc, LanePointsCalculator& lpc);
geometry_msgs::Point pointToMessagePoint(Point2i point);

void configCallback(echtzeitsysteme::ImageProcessingConfig &config, uint32_t level)
{
  low_H = config.low_H;
  low_S = config.low_S;
  low_V = config.low_V;
  high_H = config.high_H;
  high_S = config.high_S;
  high_V = config.high_V;
  y_dist_cm = config.y_dist_cm;
  lane_dist_cm = config.lane_dist_cm;
  loop_rate = config.loop_rate;
  laneColorThreshold = config.colorThreshold;

  ROS_INFO("Updated configuration.");
}


/**
 * Here comes the real magic
 */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lane_detection");
  ros::NodeHandle nh;
  Mat frame;

  /* dynamic reconfigure commands */

  dynamic_reconfigure::Server<echtzeitsysteme::ImageProcessingConfig> server;
  dynamic_reconfigure::Server<echtzeitsysteme::ImageProcessingConfig>::CallbackType f;

  f = boost::bind(&configCallback, _1, _2);
  server.setCallback(f);

  LanePointsCalculator& lpc = LanePointsCalculator::getInstance();


#ifdef USE_TEST_PICTURE
  frame = imread(TEST_PICTURE_PATH, IMREAD_COLOR);
  if (frame.empty())
  {
    ROS_ERROR("Test image could not be opened!");
  }
#endif

  ROS_INFO("LANE DETECTION NODE");
  char dir_name[100];
  getcwd(dir_name, 100);
  ROS_INFO("Current directory is: %s", dir_name);

#ifndef USE_TEST_PICTURE
  CameraReader reader;
  ROS_INFO("FPS: %f", reader.getVideoCapture().get(CV_CAP_PROP_FPS));
  frame = reader.readImage();


#endif

  // TODO: for more meaningful testing, move object creation in the loop

  ImageProcessor imageProcessor(frame, BGR);
  //imageProcessor.calibrateCameraImage(PARAMS_2);
    imageProcessor.calibrateCameraImage(
            myphoto2::RECT_WIDTH, myphoto2::RECT_HEIGHT, myphoto2::OFFSET_ORIGIN,
            TARGET_WIDTH, TARGET_HEIGHT,
            myphoto2::BOTTOM_LEFT, myphoto2::BOTTOM_RIGHT, myphoto2::TOP_RIGHT, myphoto2::TOP_LEFT,
            TARGET_PX_PER_CM
    );
  ROS_INFO("Calibrated camera image.");
  imshow("CameraFrame", frame);
  waitKey(100);



  /**
   * Init ROS Publisher here. Can set to be a fixed array
   */
  echtzeitsysteme::points right_line, left_line, center_line;

  ros::Publisher right_line_pub = nh.advertise<echtzeitsysteme::points>("right_line", 10);   //TODO: change buffer size
  ros::Publisher left_line_pub = nh.advertise<echtzeitsysteme::points>("left_line", 10);     //TODO: change buffer size
  ros::Publisher center_line_pub = nh.advertise<echtzeitsysteme::points>("center_line", 10); //TODO: change buffer size

  ros::Rate loop_rate(LOOP_RATE_IN_HERTZ); //TODO: Hz anpassen

  while (ros::ok())
  {
    ROS_INFO("Show frame.");
#ifndef USE_TEST_PICTURE
    frame = reader.readImage();
#endif
#ifdef DRAW_GRID
    drawGrid(frame);
#endif

    Mat processedImage = processImage(frame, imageProcessor, lpc);


    std::vector<Point2i> rightLanePoints_px = lpc.lanePoints(IMAGE_ROWS, IMAGE_ROWS_SIZE, LEFT, imageProcessor);
    std::vector<Point2i> leftLanePoints_px = lpc.lanePoints(IMAGE_ROWS, IMAGE_ROWS_SIZE, RIGHT, imageProcessor);

    // convert points to world coordinates
    right_line.points.clear();
    left_line.points.clear();

    for (auto it:rightLanePoints_px) {
        right_line.points.emplace_back(pointToMessagePoint(imageProcessor.getWorldCoordinates(it)));
    }
    for (auto it:leftLanePoints_px) {
        left_line.points.emplace_back(pointToMessagePoint(imageProcessor.getWorldCoordinates(it)));
    }

    right_line_pub.publish(right_line);
    left_line_pub.publish(left_line);
    center_line_pub.publish(center_line);

    // clear input/output buffers
    ros::spinOnce();


    // this is needed to ensure a const. loop rate
    loop_rate.sleep();
  }


  return 0;
}

Mat processImage(Mat input, ImageProcessor &proc, LanePointsCalculator& lpc)
{

  Mat output;

  proc.setImage(input, BGR);
  output = proc.transformTo2D();

#ifdef SHOW_IMAGES
  imshow("2D input", output);
  waitKey(100);
#endif

  proc.convertToHSV();
  output = proc.filterColor(Scalar(low_H, low_S, low_V),
                            Scalar(high_H, high_S, high_V));

#ifdef SHOW_IMAGES
  imshow("green filtered", output);
#endif

  return output;
}

geometry_msgs::Point pointToMessagePoint(Point2i point) {
    geometry_msgs::Point output;
    output.x = point.x;
    output.y = point.y;
    return output;
}

