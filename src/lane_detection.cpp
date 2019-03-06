#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <echtzeitsysteme/points.h>
#include <opencv2/opencv.hpp>
#include <lane_detection/image_processor.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <echtzeitsysteme/ImageProcessingConfig.h>
#include <dynamic_reconfigure/server.h>
#include <lane_detection/lane_points_calculator.hpp>
#include <constants/constants.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <lane_detection/LaneDetector.h>
#include <lane_detection/TransformingLaneDetector.h>
#include <lane_detection/NonTransformingLaneDetector.h>

using namespace cv;
using namespace constants::calibrations;

//#define TEST_PICTURE_PATH "camera_reading_test/images/calibration_test_2.jpg"
//#define TEST_PICTURE_PATH "camera_reading_test/images/track_straight.jpg"
//#define TEST_PICTURE_PATH "/home/pses/catkin_ws/src/echtzeitsysteme/include/lane_detection/images/calibration_test_2.jpg"
//#define TEST_PICTURE_PATH "echtzeitsysteme/include/lane_detection/images/2018-12-05-220157.jpg"

// NOTE: run from inside "catkin_ws" folder to find test photo
#define TEST_PICTURE_PATH "./src/echtzeitsysteme/images/my_photo-2.jpg"

#define PUBLISHER_QUEUE 1
#define SUBSCRIBER_QUEUE 1

//#define PARAMS_1 59.0, 84.0, 30.0, 640, 480, Point(0, 366), Point(632, 363), Point(404, 238), Point(237, 237), Point(151, 639), Point(488, 639), Point(488, 0), Point(151, 0)
//#define PARAMS_2 59.0, 84.0, 20, 640, 991, Point(43, 387), Point(583, 383), Point(404, 189), Point(234, 190), Point(95, 990), Point(545, 990), Point(545, 350), Point(95, 350)

// my_photo-2.jpg
#define PARAMS_3 59.0, 84.0, 20, 180, 180, Point(549, 799), Point(1384, 786), Point(1129, 490), Point(800, 493), 5
// photo from 15.12.
#define PARAMS_4 59.0, 84.0, 22, 180, 180, Point(384, 895), Point(1460, 900), Point(1128, 472), Point(760, 460), 5

int IMAGE_ROWS[] = {900, 800, 700, 600, 500, 400, 300, 200, 100};
int IMAGE_ROWS_SIZE = 9;

const int TARGET_WIDTH = 120;
const int TARGET_HEIGHT = 200;
const int TARGET_PX_PER_CM = 5;
const int LOOP_RATE_IN_HERTZ = 10;


// variables that are set by the rqt_reconfigure callback
int low_H, low_S, low_V, high_H, high_S, high_V;
double y_dist_cm, lane_dist_cm;
int loop_rate;
int laneColorThreshold;

Mat processImage(Mat input, ImageProcessor &proc, LanePointsCalculator& lpc);
geometry_msgs::Point convertPointToMessagePoint(Point2i point);

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

void receiveFrameCallback(const sensor_msgs::ImageConstPtr& receivedFrame, Mat& frame) {
    // TODO: do all the processing in here?
    frame = cv_bridge::toCvCopy(receivedFrame, sensor_msgs::image_encodings::BGR8)->image;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lane_detection");
  ros::NodeHandle nh;
  Mat frame;

  int x = 0;

  /* dynamic reconfigure commands */

  dynamic_reconfigure::Server<echtzeitsysteme::ImageProcessingConfig> server;
  dynamic_reconfigure::Server<echtzeitsysteme::ImageProcessingConfig>::CallbackType f;

  f = boost::bind(&configCallback, _1, _2);
  server.setCallback(f);

  LanePointsCalculator& lpc = LanePointsCalculator::getInstance();

  image_transport::ImageTransport imageTransport(nh);
  image_transport::Subscriber frameSubscriber = imageTransport.subscribe("camera/frame", SUBSCRIBER_QUEUE, boost::bind(&receiveFrameCallback, _1, boost::ref(frame)));

  image_transport::Publisher processedImagePublisher = imageTransport.advertise("image_processing/fully_processed", PUBLISHER_QUEUE);

  ROS_INFO("LANE DETECTION NODE");
  char dir_name[100];
  getcwd(dir_name, 100);
  ROS_INFO("Current directory is: %s", dir_name);

  CameraCalibration calibration(
          calibration_02_25::RECT_WIDTH, calibration_02_25::RECT_HEIGHT, calibration_02_25::OFFSET_ORIGIN,
          TARGET_WIDTH, TARGET_HEIGHT,
          calibration_02_25::BOTTOM_LEFT, calibration_02_25::BOTTOM_RIGHT, calibration_02_25::TOP_RIGHT, calibration_02_25::TOP_LEFT,
          TARGET_PX_PER_CM
  );
  ImageProcessor imageProcessor(frame, BGR, calibration);
  //imageProcessor.calibrateCameraImage(PARAMS_2);
  /**
   * Init ROS Publisher here. Can set to be a fixed array
   */
  echtzeitsysteme::points right_line, left_line, center_line;

  ros::Publisher right_line_pub = nh.advertise<echtzeitsysteme::points>("right_line", 10);   //TODO: change buffer size
  ros::Publisher left_line_pub = nh.advertise<echtzeitsysteme::points>("left_line", 10);     //TODO: change buffer size
  ros::Publisher center_line_pub = nh.advertise<echtzeitsysteme::points>("center_line", 10); //TODO: change buffer size

  ros::Rate loop_rate(LOOP_RATE_IN_HERTZ); //TODO: Hz anpassen

  ROS_INFO("Wait until the first frame has been received...");
  while(frame.dims==0) {
      ros::spinOnce();
  }
  ROS_INFO("First frame received. Process following frames and publish the outcomes...");

  while (ros::ok())
  {

    /* TODO: should be done by LaneDetector */


    Mat processedImage = processImage(frame, imageProcessor, lpc);
    // TODO: error-prone to use a fixed MONO8 encoding... what if the fully processed image is different in the future?
    processedImagePublisher.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, processedImage).toImageMsg());


  std::vector<Point2i> rightLanePoints_px = lpc.lanePoints(IMAGE_ROWS, IMAGE_ROWS_SIZE, LEFT, imageProcessor);
  std::vector<Point2i> leftLanePoints_px = lpc.lanePoints(IMAGE_ROWS, IMAGE_ROWS_SIZE, RIGHT, imageProcessor);

  // prepare sending of new lane points
  right_line.points.clear();
  left_line.points.clear();
  // convert found lane points to world coordinates and push them to the messages
  const double THRESHOLD = 30;
  Point2d lastPoint;
  if (!rightLanePoints_px.empty()) lastPoint = calibration.getWorldCoordinatesFrom2DImageCoordinates(
            rightLanePoints_px.at(0));

  for (auto it:rightLanePoints_px) {
      Point2d worldCoordinates = calibration.getWorldCoordinatesFrom2DImageCoordinates(it);
      if(abs(worldCoordinates.y - lastPoint.y) < THRESHOLD) {
        right_line.points.emplace_back(convertPointToMessagePoint(
                calibration.getWorldCoordinatesFrom2DImageCoordinates(it)));
      }
      lastPoint = worldCoordinates;
  }
  for (auto it:leftLanePoints_px) {
      left_line.points.emplace_back(convertPointToMessagePoint(
              calibration.getWorldCoordinatesFrom2DImageCoordinates(it)));
  }


  /* TODO: just get lane points from LaneDetector and publish then them */

  right_line_pub.publish(right_line);
  left_line_pub.publish(left_line);
  center_line_pub.publish(center_line);


      // write most recent frame on topic to frame
      ros::spinOnce();

    // ensure a constant loop rate
    loop_rate.sleep();
  }


  return 0;
}

Mat processImage(Mat input, ImageProcessor &proc, LanePointsCalculator& lpc)
{

  Mat output, morph_img;

  proc.setImage(input, BGR);
  output = proc.transformTo2D();

  // prepare color filtering
  proc.convertToHSV();
  output = proc.filterColor(Scalar(low_H, low_S, low_V),
                            Scalar(high_H, high_S, high_V));


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
  cv::morphologyEx( output, morph_img, morph_operator, element );
  morph_operator = 3;
  morph_elem = 0;
  morph_size = 4;
  // processes closing with rectangle object
  element = cv::getStructuringElement( morph_elem, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
  cv::morphologyEx( morph_img, output, morph_operator, element );

  return output;
}

geometry_msgs::Point convertPointToMessagePoint(Point2i point) {
    geometry_msgs::Point output;
    // convert from cm to m and write in message
    output.x = (point.x / 100.0);
    output.y = (point.y / 100.0);
    return output;
}

