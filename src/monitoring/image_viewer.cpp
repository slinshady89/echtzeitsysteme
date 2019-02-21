#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#define SUBSCRIBER_QUEUE 1

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

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_viewer");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber frameSubscriber = it.subscribe("camera/frame", SUBSCRIBER_QUEUE, receiveFrameCallback);    // TODO: change queue size
    image_transport::Subscriber processedImageSubscriber = it.subscribe("image_processing/fully_processed", SUBSCRIBER_QUEUE, receiveProcessedImageCallback);
    sensor_msgs::ImagePtr imageMessage;
    ros::spin();
}
