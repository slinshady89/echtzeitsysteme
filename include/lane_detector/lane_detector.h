/**
 * @file "lane_detector/lane_detector.h"
 * @brief Header file for the LANE_DETECTOR class.
 *
*/

#ifndef LANE_DETECTOR_H
#define LANE_DETECTOR_H

#include <math.h>
#include <string>
#include <sstream>
#include <boost/math/special_functions/sign.hpp>

#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>


#endif //LANE_DETECTOR_H