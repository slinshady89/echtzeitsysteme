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

using namespace cv;
using namespace constants::calibrations;

#define PUBLISHER_QUEUE 1
#define SUBSCRIBER_QUEUE 1

const int LOOP_RATE_IN_HERTZ = 20;

// variables that are set by the rqt_reconfigure callback
int green_low_H, green_low_S, green_low_V, green_high_H, green_high_S, green_high_V;
int pink_low_H, pink_low_S, pink_low_V, pink_high_H, pink_high_S, pink_high_V;

Mat processImage(Mat input, ImageProcessor &proc, LanePointsCalculator& lpc);
geometry_msgs::Point convertPointToMessagePoint(Point2d point);

void configCallback(echtzeitsysteme::ImageProcessingConfig &config, uint32_t level)
{
  green_low_H = config.green_low_H;
  green_low_S = config.green_low_S;
  green_low_V = config.green_low_V;
  green_high_H = config.green_high_H;
  green_high_S = config.green_high_S;
  green_high_V = config.green_high_V;
  pink_low_H = config.pink_low_H;
  pink_low_S = config.pink_low_S;
  pink_low_V = config.pink_low_V;
  pink_high_H = config.pink_high_H;
  pink_high_S = config.pink_high_S;
  pink_high_V = config.pink_high_V;

  ROS_INFO("Updated configuration.");
}

void receiveFrameCallback(const sensor_msgs::ImageConstPtr& receivedFrame, Mat& frame) {
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

  // TODO: change instantiation
  LanePointsCalculator& lpc = LanePointsCalculator::getInstance();

  image_transport::ImageTransport imageTransport(nh);
  image_transport::Subscriber frameSubscriber = imageTransport.subscribe("camera/frame", SUBSCRIBER_QUEUE, boost::bind(&receiveFrameCallback, _1, boost::ref(frame)));

  image_transport::Publisher processedImagePublisher = imageTransport.advertise("image_processing/fully_processed", PUBLISHER_QUEUE);

  ROS_INFO("LANE DETECTION NODE");
  char dir_name[100];
  getcwd(dir_name, 100);
  ROS_INFO("Current directory is: %s", dir_name);

  CameraCalibration calibration = calibration_02_25::calibration_150_200_px;
  ImageProcessor imageProcessor(frame, BGR, calibration);
  TransformingLaneDetector laneDetector (imageProcessor, lpc, calibration, 10);

  echtzeitsysteme::points right_line, left_line, center_line;

  ros::Publisher right_line_pub = nh.advertise<echtzeitsysteme::points>("right_line", 1);
  ros::Publisher left_line_pub = nh.advertise<echtzeitsysteme::points>("left_line", 1);
  ros::Publisher center_line_pub = nh.advertise<echtzeitsysteme::points>("center_line", 1);

  ros::Rate loop_rate(LOOP_RATE_IN_HERTZ);

  ROS_INFO("Wait until the first frame has been received...");
  while(frame.dims==0) {
      ros::spinOnce();
  }
  ROS_INFO("First frame received. Process following frames and publish the outcomes...");

  while (ros::ok())
  {
    // use most recent color thresholds
    Scalar lowGreen = Scalar(green_low_H, green_low_S, green_low_V);
    Scalar highGreen = Scalar(green_high_H, green_high_S, green_high_V);
    Scalar lowPink = Scalar(pink_low_H, pink_low_S, pink_low_V);
    Scalar highPink = Scalar(pink_high_H, pink_high_S, pink_high_V);

    // detect lanes and publish the image after processing (for monitoring only)
    laneDetector.detectLanes(frame, lowGreen, highGreen, lowPink, highPink);

    // prepare sending of new lane points
    right_line.points.clear();
    left_line.points.clear();
    center_line.points.clear();
/*
    int teiler = 50;
    // push points to messages
    for (int j = 0; j*teiler < laneDetector.getLeftLane().size(); ++j) {
      left_line.points.emplace_back(convertPointToMessagePoint(laneDetector.getLeftLane().at(j*teiler )));
    }
    for (int j = 0; j*teiler < laneDetector.getRightLane().size(); ++j) {
      right_line.points.emplace_back(convertPointToMessagePoint(laneDetector.getRightLane().at(j*teiler)));
    }*/

    for (auto it:laneDetector.getRightLane()) {
      right_line.points.emplace_back(convertPointToMessagePoint(it));
    }
    for (auto it:laneDetector.getLeftLane()) {
      left_line.points.emplace_back(convertPointToMessagePoint(it));
    }
    for (auto it:laneDetector.getMiddleLane()) {
      center_line.points.emplace_back(convertPointToMessagePoint(it));
    }


    laneDetector.publishProcessedImage(processedImagePublisher);
    right_line_pub.publish(right_line);
    left_line_pub.publish(left_line);
    center_line_pub.publish(center_line);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

Mat processImage(Mat input, ImageProcessor &proc, LanePointsCalculator& lpc)
{

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
  cv::morphologyEx( output, morph_img, morph_operator, element );
  morph_operator = 3;
  morph_elem = 0;
  morph_size = 4;
  // processes closing with rectangle object
  element = cv::getStructuringElement( morph_elem, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
  cv::morphologyEx( morph_img, output, morph_operator, element );

  return output;
}

geometry_msgs::Point convertPointToMessagePoint(Point2d point) {
    geometry_msgs::Point output;
    // convert from cm to m and write in message
    output.x = (point.x / 100.0);
    output.y = (point.y / 100.0);
    return output;
}

