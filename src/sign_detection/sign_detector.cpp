#include "ros/ros.h"
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <std_msgs/Int16.h>
#include <string>

//signs
#define NONE 0
#define STOP 1
#define PED 2
#define FORTY 3
#define SEVENTY 4

//states
#define RESET 5
#define DEFAULT 6
#define STOP_DETECTED 7
#define STOP_ACTIVE 8
#define STOP_PASSING_START 25
#define STOP_PASSING_ACTIVE 26
#define PED_DETECTED 9
#define PED_ACTIVE 10
#define FORTY_DETECTED 11
#define FORTY_ACTIVE 12
#define SEVENTY_DETECTED 13
#define SEVENTY_ACTIVE 14

//events
#define STOP_EVENT 15
#define SLOW_EVENT 16
#define MEDIUM_EVENT 17
#define FAST_EVENT 18

//constants
#define STOP_ACTIVE_TIME 3
#define PED_ACTIVE_TIME 3
#define AREA_DEFAULT

int signChecker(std::string objectClass) {
    if (objectClass == "stop") {
        return STOP;
    }
    else if (objectClass == "pedestrian") {
        return PED;
    }
    else if (objectClass == "forty") {
        return FORTY;
    }
    else {
        return SEVENTY;
    }
}

void signDetectionCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg, int &sign) {
    int biggestArea = 0;
    sign = NONE;
    for (auto it: msg->bounding_boxes) {
        int height = it.ymax - it.ymin;
        int width = it.xmax - it.xmin;
        int area = height * width;
        if (area >= biggestArea && area > 62500) {
            biggestArea = area;
            sign = signChecker(it.Class);
        }
    }
}


int main (int argc, char** argv) {

    std_msgs::Int16 publishedEvent;

    int inputSign = NONE;
    int lastEvent = MEDIUM_EVENT;
    int event = MEDIUM_EVENT;

    ros::init(argc, argv, "sign_detection");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("bounding_boxes", 10, boost::bind(signDetectionCallback, _1, boost::ref(inputSign)));

    ros::Publisher pub = nh.advertise<std_msgs::Int16>("sign_flag", 1);

    ros::Time begin = ros::Time::now();

    int state = RESET;

    ros::Rate r(10);

    double delta = 0.0;

    while (ros::ok()) {

        switch (state) {
            case RESET:
                event = lastEvent;
                publishedEvent.data = event;
                pub.publish(publishedEvent);

                state = DEFAULT;
                break;
            case DEFAULT:
                lastEvent = event;

                switch (inputSign) {
                    case STOP: state = STOP_DETECTED; break;
                    case PED: state = PED_DETECTED; break;
                    case FORTY: state = FORTY_DETECTED; break;
                    case SEVENTY: state = SEVENTY_DETECTED; break;
                    case NONE: break;
                }
                break;
            case STOP_DETECTED:
                event = STOP_EVENT;
                publishedEvent.data = event;
                pub.publish(publishedEvent);

                // reset timer
                begin = ros::Time::now();
                state = STOP_ACTIVE;
                break;
            case STOP_ACTIVE:
                delta = (ros::Time::now() - begin).toSec();
                if (delta > STOP_ACTIVE_TIME) {
                    state = STOP_PASSING_START;
                }
                break;
            case STOP_PASSING_START:
                event = lastEvent;
                publishedEvent.data = event;
                pub.publish(publishedEvent);
                state = STOP_PASSING_ACTIVE;
                break;
            case STOP_PASSING_ACTIVE:
                if (inputSign!=STOP) {
                    state = DEFAULT;
                }
                break;
            case PED_DETECTED:
                event = SLOW_EVENT;
                publishedEvent.data = event;
                pub.publish(publishedEvent);

                // reset timer
                begin = ros::Time::now();
                state = PED_ACTIVE;
                break;
            case PED_ACTIVE:
                delta = (ros::Time::now() - begin).toSec();
                if (delta > PED_ACTIVE_TIME && inputSign!=PED) {
                    state = RESET;
                }
                break;
            case FORTY_DETECTED:
                event = MEDIUM_EVENT;
                publishedEvent.data = event;
                pub.publish(publishedEvent);
                state = FORTY_ACTIVE;
                break;
            case FORTY_ACTIVE:
                if (inputSign != FORTY) {
                    state = DEFAULT;
                }
                break;
            case SEVENTY_DETECTED:
                event = FAST_EVENT;
                publishedEvent.data = event;
                pub.publish(publishedEvent);
                state = SEVENTY_ACTIVE;
                break;
            case SEVENTY_ACTIVE:
                if (inputSign != SEVENTY) {
                    state = DEFAULT;
                }
                break;
        }

        ros::spinOnce();
        r.sleep();
    }
}