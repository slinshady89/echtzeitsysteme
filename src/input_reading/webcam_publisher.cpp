#include <ros/ros.h>
#include <image_transport/image_transport.h>    // TODO: add to package.xml
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>    // TODO: add to package.xm
#include <lane_detection/CameraReader.hpp>
#include <dynamic_reconfigure/server.h>
#include <echtzeitsysteme/WebcamSettingsConfig.h>

#define WIDTH 1280
#define HEIGHT 720


const int PUBLISH_RATE = 10;
const int PUBLISHER_QUEUE = 1;

void configCallback(echtzeitsysteme::WebcamSettingsConfig &config, uint32_t level, CameraReader& cameraReader)
{
    cameraReader.setCameraSettings(config.brightness, config.contrast, config.saturation, config.hue);
  ROS_INFO("Webcam settings configuration updated.");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "webcam_publisher");
    ros::NodeHandle nh;

    CameraReader cameraReader(WIDTH, HEIGHT);

    dynamic_reconfigure::Server<echtzeitsysteme::WebcamSettingsConfig> server;
    dynamic_reconfigure::Server<echtzeitsysteme::WebcamSettingsConfig>::CallbackType f;

    f = boost::bind(&configCallback, _1, _2, boost::ref(cameraReader));
    server.setCallback(f);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher framePublisher = it.advertise("camera/frame", PUBLISHER_QUEUE);    // TODO: change queue size
    sensor_msgs::ImagePtr imageMessage;

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
