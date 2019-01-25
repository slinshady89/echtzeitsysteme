#include <gtest/gtest.h>
#include <ros/ros.h>
#include "lane_detection/lane_points_calculator.hpp"



int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
