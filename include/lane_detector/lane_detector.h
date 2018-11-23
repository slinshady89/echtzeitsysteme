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

#include "trajectory.h"


namespace lane_detector {


    /**
    * @typedef sensor_msgs::Imu imu_msg
    * @brief shortcut for IMU messages
    */
    typedef sensor_msgs::Imu imu_msg;
    
    static const std::string DEFAULT_IMU_TOPIC =
        "/uc_bridge/imu"; /**< Default topic from which IMU messages can be
                            received. */
  
}
class Lane_Detector: Trajectory{
    public:
    size_t getArraySize(size_t m_size);

    private:

    const size_t arraySizeTraj =  10;

    //Trajectory::trajPoints[10];


    size_t setArraySize(size_t m_size);

};





#endif //LANE_DETECTOR_H