#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#define READ_RATE 10

void constImageCallback(const sensor_msgs::ImageConstPtr& imageMessage) {
}
void mutableImageCallback(const sensor_msgs::ImageConstPtr& imageMessage) {
    ROS_INFO("Received an image");
    cv::Mat receivedImage = cv_bridge::toCvCopy(imageMessage, sensor_msgs::image_encodings::BGR8)->image;
    cv::imshow("received image", receivedImage);
    cv::waitKey(10);
    // TODO: do something with the image (e.g. draw some lines)
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_viewer");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber frameSubscriber = it.subscribe("camera/frame", 1, mutableImageCallback);    // TODO: change queue size
    sensor_msgs::ImagePtr imageMessage;
    ros::spin();
}
