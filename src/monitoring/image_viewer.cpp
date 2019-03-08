#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <echtzeitsysteme/points.h>
#include <lane_detection/CameraCalibration.hpp>
#include <constants/constants.h>

#define SUBSCRIBER_QUEUE 1

using namespace constants::calibrations;

const int TARGET_WIDTH = 120;
const int TARGET_HEIGHT = 200;
const int TARGET_PX_PER_CM = 5;


void receiveFrameCallback(const sensor_msgs::ImageConstPtr &imageMessage) {
  ROS_INFO("Received an image");
  cv::Mat receivedImage = cv_bridge::toCvShare(imageMessage, sensor_msgs::image_encodings::BGR8)->image;
  cv::imshow("received image", receivedImage);
  cv::waitKey(10);
  // TODO: do something with the image (e.g. draw some lines)
}

void receiveProcessedImageCallback(const sensor_msgs::ImageConstPtr& imageMessage,
        std::vector<Point2i>& right, std::vector<Point2i>& left, std::vector<Point2i>& center, std::vector<Point2i>& traj) {
  ROS_INFO("Received a processed image");
  std::vector<std::vector<Point2i>> lines{right, left, center, traj};
  std::vector<Scalar> colors{Scalar(255, 0, 0), Scalar(0,255,0), Scalar(0,0,255), Scalar(255,102,153)};
  // TODO: fixed encoding is error-prone
  cv::Mat processedImage = cv_bridge::toCvCopy(imageMessage, sensor_msgs::image_encodings::MONO8)->image;

  cvtColor(processedImage, processedImage, COLOR_GRAY2BGR);
  for (int i=0; i<(int)lines.size(); i++) {
      std::vector<Point2i>& currentLine = lines.at(i);
    for(auto point : currentLine) {
      circle(processedImage, point, 3, colors.at(i), -1);
    }
    for (int j=0; j<((int)currentLine.size())-1; j++) {
      line(processedImage, currentLine.at(j), currentLine.at(j+1), colors.at(i), 3);
    }
  }

  /*
  for (auto line : lines) {
    for (int i=0; i<((int)line.size())-1; i++) {
        line(processedImage, line.at(i), )
    }
  }
  for(int i=0; i<((int)rightLane.size())-1; i++) {
    line(processedImage, rightLane.at(i), rightLane.at(i+1), Scalar(0,0,255), 3);
  }
  ROS_INFO("hallo");
  for(auto it : traj) {
    circle(processedImage, it, 3, Scalar(0,255,0), -1);
  }
  for(int j=0; j<((int)traj.size())-1; j++) {
    line(processedImage, traj.at(j), traj.at(j+1), Scalar(0,255,0), 3);
  }
  */
  cv::imshow("processed image", processedImage);
  cv::waitKey(10);
}
void rightLaneCallback(const echtzeitsysteme::points::ConstPtr &msg, std::vector<double> &right_lane_x, std::vector<double> &right_lane_y,
                       std::vector<Point2i> &rightLane_px, CameraCalibration& calibration) {
  ROS_INFO("rightLaneCallback");
  right_lane_x.clear();
  right_lane_y.clear();
  rightLane_px.clear();
  for(auto it : msg->points) {
    right_lane_x.emplace_back(it.x);
    right_lane_y.emplace_back(it.y);
    auto msgPoints = Point2d(it.x, it.y);
    Point2i imgCoordinates = calibration.get2DImageCoordinatesFromWorldCoordinates(Point2d(it.x * 100.0, it.y * 100.0));
    if (imgCoordinates.x>=0 && imgCoordinates.x<calibration.getDstWidth()
        && imgCoordinates.y>=0 && imgCoordinates.y<calibration.getDstHeight()) {
      rightLane_px.emplace_back(imgCoordinates);
    }
  }
}
void leftLaneCallback(const echtzeitsysteme::points::ConstPtr &msg, std::vector<double> &left_lane_x,
                      std::vector<double> &left_lane_y,
                      std::vector<Point2i> &leftLane_px, CameraCalibration &calibration) {
  ROS_INFO("leftLaneCallback");
  left_lane_x.clear();
  left_lane_y.clear();
  leftLane_px.clear();
  for(auto it : msg->points) {
    left_lane_x.emplace_back(it.x);
    left_lane_y.emplace_back(it.y);
    auto msgPoints = Point2d(it.x, it.y);
    Point2i imgCoordinates = calibration.get2DImageCoordinatesFromWorldCoordinates(Point2d(it.x * 100.0, it.y * 100.0));
    if (imgCoordinates.x>=0 && imgCoordinates.x<calibration.getDstWidth()
        && imgCoordinates.y>=0 && imgCoordinates.y<calibration.getDstHeight()) {
      leftLane_px.emplace_back(imgCoordinates);
    }
  }
}
void centerLaneCallback(const echtzeitsysteme::points::ConstPtr &msg, std::vector<double> &center_lane_x,
                      std::vector<double> &center_lane_y,
                      std::vector<Point2i> &centerLane_px, CameraCalibration &calibration) {
  ROS_INFO("centerLaneCallback");
  center_lane_x.clear();
  center_lane_y.clear();
  centerLane_px.clear();
  for(auto it : msg->points) {
    center_lane_x.emplace_back(it.x);
    center_lane_y.emplace_back(it.y);
    auto msgPoints = Point2d(it.x, it.y);
    Point2i imgCoordinates = calibration.get2DImageCoordinatesFromWorldCoordinates(Point2d(it.x * 100.0, it.y * 100.0));
    if (imgCoordinates.x>=0 && imgCoordinates.x<calibration.getDstWidth()
        && imgCoordinates.y>=0 && imgCoordinates.y<calibration.getDstHeight()) {
      centerLane_px.emplace_back(imgCoordinates);
    }
  }
}
void trajCallback(const echtzeitsysteme::points::ConstPtr &msg, std::vector<double>& traj_x, std::vector<double>& traj_y,
                      std::vector<Point2i>& traj_px, CameraCalibration& calibration)
{
  ROS_INFO("trajCallback");
  traj_x.clear();
  traj_y.clear();
  traj_px.clear();
  for(auto it : msg->points) {
    traj_x.emplace_back(it.x);
    traj_y.emplace_back(it.y);
    Point2i imgCoordinates = calibration.get2DImageCoordinatesFromWorldCoordinates(Point2d(it.x * 100.0, it.y * 100.0));
    if (imgCoordinates.x>=0 && imgCoordinates.x<calibration.getDstWidth()
        && imgCoordinates.y>=0 && imgCoordinates.y<calibration.getDstHeight()) {
      traj_px.emplace_back(imgCoordinates);
    }
  }

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_viewer");
  ros::NodeHandle nh;

  std::vector<double> traj_x, traj_y;
  std::vector<double> right_lane_x, right_lane_y;
  std::vector<double> left_lane_x, left_lane_y;
  std::vector<double> center_lane_x, center_lane_y;

  std::vector<Point2i> trajImagePoints;
  std::vector<Point2i> rightLaneImagePoints;
  std::vector<Point2i> leftLaneImagePoints;
  std::vector<Point2i> centerLaneImagePoints;

  CameraCalibration calibration = calibration_02_25::calibration_150_200_px;

  ros::Subscriber leftLaneSubscriber = nh.subscribe<echtzeitsysteme::points>("left_line", 1, boost::bind(
          &leftLaneCallback, _1,
          boost::ref(left_lane_x), boost::ref(left_lane_y), boost::ref(leftLaneImagePoints), boost::ref(calibration)));

  ros::Subscriber rightLaneSubscriber = nh.subscribe<echtzeitsysteme::points>("right_line", 1, boost::bind(&rightLaneCallback, _1,
          boost::ref(right_lane_x), boost::ref(right_lane_y), boost::ref(rightLaneImagePoints), boost::ref(calibration)));

  ros::Subscriber centerLaneSubscriber = nh.subscribe<echtzeitsysteme::points>("center_line", 1, boost::bind(&centerLaneCallback, _1,
           boost::ref(center_lane_x), boost::ref(center_lane_y), boost::ref(centerLaneImagePoints), boost::ref(calibration)));


  ros::Subscriber trajectorySubscriber = nh.subscribe<echtzeitsysteme::points>("trajectory", 1, boost::bind(&trajCallback, _1,
          boost::ref(traj_x), boost::ref(traj_y), boost::ref(trajImagePoints), boost::ref(calibration)));

  // calculate vehicle coordinates of trajectory points into pixel coordinates and draw them

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber frameSubscriber = it.subscribe("camera/frame", SUBSCRIBER_QUEUE, receiveFrameCallback);    // TODO: change queue size


  std::vector<std::vector<Point2i>> lines{rightLaneImagePoints, leftLaneImagePoints, centerLaneImagePoints, trajImagePoints};

  image_transport::Subscriber processedImageSubscriber = it.subscribe("image_processing/fully_processed", SUBSCRIBER_QUEUE,
          boost::bind(&receiveProcessedImageCallback, _1,
                  boost::ref(rightLaneImagePoints), boost::ref(leftLaneImagePoints), boost::ref(centerLaneImagePoints), boost::ref(trajImagePoints)));
  sensor_msgs::ImagePtr imageMessage;


  ros::spin();
}
