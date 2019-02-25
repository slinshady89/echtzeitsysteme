#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <echtzeitsysteme/points.h>

#define SUBSCRIBER_QUEUE 1

std::vector<double> traj_x, traj_y;

void receiveFrameCallback(const sensor_msgs::ImageConstPtr &imageMessage) {
  ROS_INFO("Received an image");
  cv::Mat receivedImage = cv_bridge::toCvShare(imageMessage, sensor_msgs::image_encodings::BGR8)->image;
  cv::imshow("received image", receivedImage);
  cv::waitKey(10);
  // TODO: do something with the image (e.g. draw some lines)
}

void receiveProcessedImageCallback(const sensor_msgs::ImageConstPtr& imageMessage) {
  ROS_INFO("Received a processed image");
  // TODO: fixed encoding is error-prone
  cv::Mat processedImage = cv_bridge::toCvCopy(imageMessage, sensor_msgs::image_encodings::MONO8)->image;
  cv::imshow("processed image", processedImage);
  cv::waitKey(10);
}
void trajCallback(const echtzeitsysteme::points::ConstPtr &msg)
{
  for(auto it : msg->points) {
    traj_x.emplace_back(it.x);
    traj_y.emplace_back(it.y);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_viewer");
  ros::NodeHandle nh;

  ros::Subscriber trajectory = nh.subscribe("trajectory", 1, trajCallback);

  // calculate vehicle coordinates of trajectory points into pixel coordinates and draw them

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber frameSubscriber = it.subscribe("camera/frame", SUBSCRIBER_QUEUE, receiveFrameCallback);    // TODO: change queue size
  image_transport::Subscriber processedImageSubscriber = it.subscribe("image_processing/fully_processed", SUBSCRIBER_QUEUE, receiveProcessedImageCallback);
  sensor_msgs::ImagePtr imageMessage;

  traj_x.clear();
  traj_y.clear();

  ros::spin();
}
