#include <ros/ros.h>
#include <image_transport/image_transport.h>    // TODO: add to package.xml
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>    // TODO: add to package.xm
#include <lane_detection/CameraReader.hpp>
#include <stdio.h>


#define TEST_PICTURE_PATH "./src/echtzeitsysteme/images/captured_frames/02-25_12-35-53.jpg"

const int PUBLISH_RATE = 1;
const int PUBLISHER_QUEUE = 1;

const std::vector<std::string> imagePaths{
    std::string("./src/echtzeitsysteme/images/captured_frames/02-25_12-35-53.jpg"),
    std::string("./src/echtzeitsysteme/images/captured_frames/02-25_12-35-43.jpg"),
    std::string("./src/echtzeitsysteme/images/captured_frames/02-25_12-35-27.jpg"),
    std::string("./src/echtzeitsysteme/images/captured_frames/02-25_12-35-17.jpg"),
    std::string("./src/echtzeitsysteme/images/captured_frames/02-25_12-35-12.jpg"),
    std::string("./src/echtzeitsysteme/images/captured_frames/02-25_12-35-03.jpg")
};

std::string nextPath(int i) {
    i = i % imagePaths.size();
    return imagePaths.at(i);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher framePublisher = it.advertise("camera/frame", PUBLISHER_QUEUE);    // TODO: change queue size
    sensor_msgs::ImagePtr imageMessage;

    int currentImage = 0;

    ros::Rate loop_rate(PUBLISH_RATE);
    ROS_INFO("Publish %d images per second...", PUBLISH_RATE);
    while(ros::ok())
    {
        /*int c = getchar();
        if (c=='f')
            currentImage++;
        else if (c=='b')
            currentImage--;*/
        
        Mat frame = cv::imread(nextPath(currentImage).c_str(), IMREAD_COLOR);
        //Mat frame = cv::imread(TEST_PICTURE_PATH, IMREAD_COLOR);
        imageMessage = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
        framePublisher.publish(imageMessage);

        currentImage++;
        ros::spinOnce();
        loop_rate.sleep();
    }
}
