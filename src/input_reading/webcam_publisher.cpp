#include <ros/ros.h>
#include <image_transport/image_transport.h>    // TODO: add to package.xml
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>    // TODO: add to package.xm
#include <lane_detection/CameraReader.hpp>

#define WIDTH 1280
#define HEIGHT 720


const int PUBLISH_RATE = 10;
const int PUBLISHER_QUEUE = 1;

int main(int argc, char** argv) {
    ros::init(argc, argv, "webcam_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher framePublisher = it.advertise("camera/frame", PUBLISHER_QUEUE);    // TODO: change queue size
    sensor_msgs::ImagePtr imageMessage;

    CameraReader cameraReader(WIDTH, HEIGHT);
    ros::Rate loop_rate(PUBLISH_RATE);
    ROS_INFO("Publish %d webcam frames per second...", PUBLISH_RATE);
    while(ros::ok()) {

        Mat frame = cameraReader.readImage();

        imageMessage = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
        framePublisher.publish(imageMessage);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
