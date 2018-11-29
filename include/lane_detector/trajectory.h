#include <cmath>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Imu.h>
#include <vector>

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


#include <cstdio>//.....................................................printf()
#include "y_interp.h"//...................................yInterp,<cmath>{sin()}
int main(){//<===========A SIMPLE EXAMPLE FOR THE yInterp::CubeInterp() FUNCTION
double X[20];/*<-*/for(int i=0;i<20;++i)X[i]=5*i/20.+5;
double Y[20];/*<-*/for(int i=0;i<20;++i)Y[i]=sin(2*3.14159*X[i]/5)+1;
double x=7.18;
int i=yInterp::BinarySearch(X+1,X+19,x)-X;
double y=yInterp::CubeInterp(X+i,Y+i,x,0.,0.);
printf("At x=%.3f, y is approximately %.3f.\n",x,y);
}//~~~~~~YAGENAUT@GMAIL.COM~~~~~~~~~~~~~~~~~~~~~~~~~LAST~UPDATED~21JUL2014~~~~~~


class CTrajectory{
public:
    friend CPoint;
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

private:
};

