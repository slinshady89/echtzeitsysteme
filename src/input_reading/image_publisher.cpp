#include <ros/ros.h>
#include <image_transport/image_transport.h>    // TODO: add to package.xml
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>    // TODO: add to package.xm
#include <lane_detection/CameraReader.hpp>


#define TEST_PICTURE_PATH "./src/echtzeitsysteme/images/my_photo-2.jpg"

const int PUBLISH_RATE = 20;
const int PUBLISHER_QUEUE = 1;

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher framePublisher = it.advertise("camera/frame", PUBLISHER_QUEUE);    // TODO: change queue size
    sensor_msgs::ImagePtr imageMessage;

    ros::Rate loop_rate(PUBLISH_RATE);
    ROS_INFO("Publish %d images per second...", PUBLISH_RATE);
    while(ros::ok()) {

        Mat frame = cv::imread(TEST_PICTURE_PATH, IMREAD_COLOR);
        imageMessage = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
        framePublisher.publish(imageMessage);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
