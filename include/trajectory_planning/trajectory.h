#include <cmath>
#include "ros/ros.h"
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Imu.h>
#include <vector>
#include <array>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "y_interp.h"//...................................yInterp,<cmath>{sin()}
#include <cstdio>//.....................................................printf()

#include "echtzeitsysteme/points.h"
#include "controller.h"
#include "../safety.h"


// alglib
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

//#include "interpolation.h"
//#include "stdafx.h"


class CPoint{
public:
    CPoint(double x = 0.0, double y = 0.0){
        m_x = x;
        m_y = y;
    };
private:
    double m_x;
    double m_y;
};


class CTrajectory : CPoint
{
public:
    CTrajectory(CPoint point, double acc = 0.0, double curv = 0.0, 
                 double theta = 0.0, double s = 0, double t = 0 )
    {
        m_point = point;    
        m_acceleration = acc;
        m_curvature = curv;
        m_theta = theta;
        m_s = s;
        m_t = t;
    };
    std::vector<CTrajectory> trajPoints; 
    CPoint icc; // 
    CPoint m_point;
    double m_acceleration;
    double m_curvature; // curvature kappa
    double m_theta; // tangential angle of the traj
    double m_s; // driven dist on traj
    double m_t; // time of arrival with actual v
    
    std::vector<std::vector<CTrajectory>> vecTrajectories;

    std::vector<std::vector<CTrajectory>> calculateTrajs(std::array<CPoint,100> _pointsDesiredTraj /*[x,y] points on the desired trajectory*/,
                                                        size_t _stepSize /*_stepSize between points in the array*/,
                                                        double _interpolationError /*_interpolationError that is acceptable*/);

private:
};

