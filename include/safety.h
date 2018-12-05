#ifndef SAFETY_H_
#define SAFETY_H_

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include <iostream>
#include <std_msgs/Float64.h>

class CSafety{
public:
    // Callbacks for the left front and right ultrasonic sensors
    void uslCallback(sensor_msgs::Range::ConstPtr uslMsg, sensor_msgs::Range* usl);
    void usfCallback(sensor_msgs::Range::ConstPtr usfMsg, sensor_msgs::Range* usf);
    void usrCallback(sensor_msgs::Range::ConstPtr usrMsg, sensor_msgs::Range* usr);
    void set_US_Dist_front(double _dist){
        maxDistFront = _dist;
    }
    void set_US_Dist_side(double _dist){
        maxDistSide = _dist;
    }
    double get_US_Dist_front(){
        return maxDistFront;
    } 
    double get_US_Dist_side(){
        return maxDistSide;
    }
private:
    double maxDistSide = 0.4;
    double maxDistFront = 0.3;

};



#endif // SAFETY_H_


