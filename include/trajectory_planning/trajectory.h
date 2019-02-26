#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "ros/ros.h"
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Imu.h>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"

#include "controller.h"
#include "../safety.h"
#include <echtzeitsysteme/points.h>

#include <vector>
#include <array>
//#include <cassert>

// includes for alglib
#include "stdafx.h"
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include "interpolation.h"

class CPoint
{

public:

  CPoint () = default;
  CPoint(CPoint && var) = default;
  CPoint(const CPoint& var) = default;
  CPoint& operator=( CPoint && var) = default;
  CPoint& operator=(const CPoint& var) = default;
  CPoint(double x, double y) : m_x_(x), m_y_(y){};

private:
  double m_x_ = 0.0;
  double m_y_ = 0.0;
};

class CTrajectoryPoint : public CPoint
{
public:

  CTrajectoryPoint () = default;
  CTrajectoryPoint(CTrajectoryPoint && var) = default;
  CTrajectoryPoint(const CTrajectoryPoint& var) = default;
  CTrajectoryPoint& operator=( CTrajectoryPoint && var) = default;
  CTrajectoryPoint& operator=(const CTrajectoryPoint& var) = default;
  CTrajectoryPoint(double x, double y, double acc = 0.0, double curvature = 0.0, double theta = 0.0,
                   double s = 0.0, double t = 0.0) : CPoint(x, y),
                                                     m_point_(CPoint(x, y)),
                                                     m_acceleration_(acc),
                                                     m_curvature_(curvature),
                                                     m_theta_(theta),
                                                     m_s_(s),
                                                     m_t_(t){};

private:
  CPoint icc_= CPoint(0.0,0.0);                       // center point to each point of the trajectory
  CPoint m_point_ = CPoint(0.0,0.0);                  // the trajectory point in vehicle coordinates itself
  double m_acceleration_ = 0.0;                       // acceleration at the given point of the traj
  double m_curvature_ = 0.0;                          // curvature kappa
  double m_theta_ = 0.0;                              // tangential angle of the traj
  double m_s_ = 0.0;                                  // driven dist on traj
  double m_t_ = 0.0;                                  // time of arrival with actual v

};

class CTrajectory /*: public CTrajectoryPoint */
{
public:
  CTrajectory () = default;
  CTrajectory(CTrajectory && var) = default;
  CTrajectory(const CTrajectory& var) = default;
  CTrajectory& operator=( CTrajectory && var) = default;
  CTrajectory& operator=(const CTrajectory& var) = default;
  
  


  CTrajectory(std::vector<double> &_x, std::vector<double> &_y, bool _boundaryType = 0) : vec_x_(_x), vec_y_(_y), natural_bound_type_(_boundaryType)
  {
    v_ = 0.0;
    if (_x.size() == _y.size() && !_x.empty() && !_y.empty())
    {
      x_.setcontent(_x.size(), &_x.front());
      y_.setcontent(_y.size(), &_y.front());
      calcLinLength(_x, _y);
      s_.setcontent(s_lin.size(),&s_lin.front());
      ROS_INFO("ALGLIB in Trajectory Constructor");
      alglib::spline1dbuildcubic(s_, x_ ,vec_x_.size(), natural_bound_type_, 0.0, natural_bound_type_, 0.0, spline_x_);
      alglib::spline1dbuildcubic(s_, y_ ,vec_y_.size(), natural_bound_type_, 0.0, natural_bound_type_, 0.0, spline_y_);
    }
    else
    {
      ROS_INFO("                  NO SPLINE CALCULATED               \n");
      ROS_INFO("number of points for x = %d and y = %d differ \n", int(_x.size()), int(_y.size()));
    }
  }

  /// calcTraj calculates a trajectory between 2 given lines
  // &_other: trajectory for comparision
  // _weighting: weighting factor for this trajectory
  // _offset: defines an offset that should be calculated to the traj in y direction
  CTrajectory calcTraj(CTrajectory &_other, double _weighting, double _offset);




  double calcCurvatureAt(double _s);
  std::vector<double> calcCurvature(double _stepsize);

  void calcLinLength(std::vector<double> _x, std::vector<double> _y);

  // some accessors that might be useful
  double getLinLength() { return spline_lin_length_; }
  std::vector<double> getVecWaypointDists() { return s_lin; }
  alglib::spline1dinterpolant getSplineInterpolant(char _name);

  alglib::ae_int_t getNatural_bound_type_() const;

  void setNatural_bound_type_(alglib::ae_int_t natural_bound_type_);

  geometry_msgs::Point getPointOnTrajAt(double _waypoint);

private:
  std::vector<double> vec_x_, vec_y_;

  std::vector<double> s_lin;
  double v_ = 0.0;                                                  // planned velocity of the trajectory
  double spline_lin_length_ = 0.0;                                  // linear length of the spline

  alglib::real_1d_array x_, y_;                                     // x, y arrays of the trajectory
  alglib::real_1d_array s_;                                         // s: travelled distance on the spline + init dist
  alglib::spline1dinterpolant spline_x_, spline_y_;                 // structures that hold splines of x and y parametrized by s
  alglib::ae_int_t natural_bound_type_ = 2;
};

#endif //TRAJECTORY_H
