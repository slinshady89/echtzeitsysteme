#include <ros/ros.h>
#include <image_transport/image_transport.h>    // TODO: add to package.xml
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>    // TODO: add to package.xm
#include <lane_detection/CameraReader.hpp>

#define PUBLISH_RATE 20

int main(int argc, char** argv) {
    ros::init(argc, argv, "webcam_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher framePublisher = it.advertise("camera/frame", 1);    // TODO: change queue size
    sensor_msgs::ImagePtr imageMessage;

    CameraReader cameraReader;
    ros::Rate loop_rate(PUBLISH_RATE);
    while(ros::ok()) {

        Mat frame = cameraReader.readImage();

        //cv::imshow("sent image", frame);
        //cv::waitKey(10);

        imageMessage = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
        framePublisher.publish(imageMessage);

        ros::spinOnce();
        loop_rate.sleep();
    }
}