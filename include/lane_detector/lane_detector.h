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


    namespace lane_detector{
    /**
    * @typedef sensor_msgs::Image image_msg
    * @brief shortcut for Image messages
    */
    typedef sensor_msgs::Image image_msg;
    /**
    * @typedef sensor_msgs::Imu imu_msg
    * @brief shortcut for IMU messages
    */
    typedef sensor_msgs::Imu imu_msg;/**
    * @typedef std_msgs::Int16 int16_msg
    * @brief shortcut for int16 messages
    */
    typedef std_msgs::Int16 int16_msg;
    /**
    * @typedef std_msgs::Float64 float64_msg
    * @brief shortcut for double messages
    */
    typedef std_msgs::Float64 float64_msg;
    /**
    * @typedef std_msgs::UInt8 uint8_msg
    * @brief shortcut for uint8 messages
    */
    typedef std_msgs::UInt8 uint8_msg;


    static const std::string DEFAULT_IMU_TOPIC =
        "/uc_bridge/imu"; /**< Default topic from which IMU messages can be
                            received. */

    static const std::string DEFAULT_IMAGE_COLOR_TOPIC =
        "kinect2/qhd/image_color"; /**< Default topic from which qhd color images
                                    can be received. */
    static const std::string DEFAULT_IMAGE_DEPTH_TOPIC =
        "kinect2/sd/image_depth"; /**< Default topic from which sd depth
                                    images can be received. */

    static const std::string DEFAULT_VIDEO_FEED_MODE =
        "Off"; /**< Default video feed mode on startup. */

    static const std::string VIDEO_FEED_MODE_COLOR =
        "Color Image (1280x720)"; /**< Color video feed mode. */
    static const std::string VIDEO_FEED_MODE_DEPTH =
        "Depth Image (640x480)"; /**< Depth video feed mode. */
}





#endif //LANE_DETECTOR_H