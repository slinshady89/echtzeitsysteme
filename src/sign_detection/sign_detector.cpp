#include "ros/ros.h"
#include "BoundingBoxes.h"
#include "BoundingBox.h"
#include <std_msgs/Int16.h>

//signs
#define NONE 0
#define STOP 1
#define PED 2
#define FORTY 3
#define SEVENTY 4

//states
#define RESET 0
#define STOP_DETECTED 1
#define PED_DETECTED 2
#define FORTY_DETECTED 3
#define SEVENTY_DETECTED 4
#define STOP_DELAY 5
#define PED_DELAY 6


void signDetectionCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg, int &sign) {
}

int main (int argc, char** argv) {

    std_msgs::Int16 vel, oldVal;
    vel.data = 200;
    oldVal.data = 200;

    int sign = NONE;

    ros::init(argc, argv, "sign_detection");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("bounding_boxes", 10, boost::bind(signDetectionCallback, _1, boost::ref(sign)));

    ros::Publisher pub = nh.advertise<std_msgs::Int16>("sign_velocity", 1);

    ros::Time begin = ros::Time::now();

    int state = RESET;

    ros::Rate r(10);

    while (ros::ok()) {

        switch (state) {
            case RESET:
                vel.data = oldVal;
                pub.publish(vel);

                if (sign == STOP) {
                    state = STOP_DETECTED;
                    break;
                }
                else if (sign == PED) {
                    state = PED_DETECTED;
                    break;
                }
                else if (sign == FORTY) {
                    state = FORTY_DETECTED;
                    break;
                }
                else if (sign == SEVENTY) {
                    state = FORTY_DETECTED;
                    break;
                }
                break;

            case STOP_DETECTED:
                if (val.data != 100) {
                    oldVal.data = vel.data;
                }
                vel.data = 0;
                pub.publish(vel);

                ros::Duration(2).sleep();

                state = STOP_DELAY;
                break;

            case PED_DETECTED:
                if (vel.data != 100) {
                    oldVal.data = vel.data;
                } 
                vel.data = 100;
                pub.publish(vel);

                if (sign == STOP) {
                    state = STOP_DETECTED;
                    break;
                }
                else if (sign == PED) {
                    state = PED_DETECTED;
                    break;
                }
                else if (sign == FORTY) {
                    state = FORTY_DETECTED;
                    break;
                }
                else if (sign == SEVENTY) {
                    state = FORTY_DETECTED;
                    break;
                }
                else {
                    begin = ros::Time::now();
                    state = PED_DELAY;
                    break;
                }
                

            case FORTY_DETECTED:
                vel.data = 200;
                pub.publish(vel);

                if (sign == STOP) {
                    state = STOP_DETECTED;
                    break;
                }
                else if (sign == PED) {
                    state = PED_DETECTED;
                    break;
                }
                else if (sign == SEVENTY) {
                    state = SEVENTY_DETECTED;
                    break;
                }
                break;
            
            case SEVENTY_DETECTED:
                vel.data = 300;
                pub.publish(vel);

                if (sign == STOP) {
                    state = STOP_DETECTED;
                    break;
                }
                else if (sign == PED) {
                    state = PED_DETECTED;
                    break;
                }
                else if (sign == FORTY) {
                    state = FORTY_DETECTED;
                    break;
                }
                break;

            case STOP_DELAY:
                vel.data = oldVal.data;
                pub.publish(vel);

                if (sign == PED) {
                    state = PED_DETECTED;
                    break;
                }
                else if (sign == FORTY) {
                    state = FORTY_DETECTED;
                    break;
                }
                else if (sign == SEVENTY) {
                    state = FORTY_DETECTED;
                    break;
                }
                else if (sign == NONE) {
                    state = RESET;
                    break;
                }
                break;

            case PED_DELAY:
                if (sign == STOP) {
                    state = STOP_DETECTED;
                    break;
                }
                else if (sign == PED) {
                    state = PED_DETECTED;
                    break;
                }
                else if (sign == FORTY) {
                    state = FORTY_DETECTED;
                    break;
                }
                else if (sign == SEVENTY) {
                    state = FORTY_DETECTED;
                    break;
                }
                
                double sec = ros::Time::now() - begin;
                else if (sec > 2) {
                    state = RESET;
                }
                break;    
        }

        ros::spinOnce();
        r.sleep();
    }
}