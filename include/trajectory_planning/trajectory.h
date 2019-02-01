#ifndef TRAJECTORY_H
#define TRAJECTORY_H

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

#include "controller.h"
#include "../safety.h"

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
  CPoint(double x, double y) : m_x_(x), m_y_(y){};

private:
  double m_x_ = 0.0;
  double m_y_ = 0.0;
};

class CTrajectoryPoint : public CPoint
{
public:
  CTrajectoryPoint(double x, double y, double acc = 0.0, double curvature = 0.0, double theta = 0.0,
                   double s = 0.0, double t = 0.0) : CPoint(x, y),
                                                     m_point_(CPoint(x, y)),
                                                     m_acceleration_(acc),
                                                     m_curvature_(curvature),
                                                     m_theta_(theta),
                                                     m_s_(s),
                                                     m_t_(t){};

private:
  CPoint icc_ = CPoint(0.0, 0.0);     // center point to each point of the trajectory
  CPoint m_point_ = CPoint(0.0, 0.0); // the trajectory point in vehicle coordinates itself
  double m_acceleration_ = 0.0;       // acceleration at the given point of the traj
  double m_curvature_ = 0.0;          // curvature kappa
  double m_theta_ = 0.0;              // tangential angle of the traj
  double m_s_ = 0.0;                  // driven dist on traj
  double m_t_ = 0.0;                  // time of arrival with actual v
};

class CTrajectory /*: public CTrajectoryPoint */
{
public:
  CTrajectory(std::vector<double> &_x, std::vector<double> &_y) : vec_x_(_x), vec_y_(_y)
  {
    v_ = 0.0;
    if (_x.size() == _y.size())
    {
      x_.setcontent(_x.size(), &_x.front());
      y_.setcontent(_y.size(), &_y.front());
      calcLinLength(_x, _y);
      s_.setcontent(s_lin.size(), &s_lin.front());
      auto test = s_.getcontent();
      alglib::spline1dbuildcubic(s_, x_, spline_x_);
      alglib::spline1dbuildcubic(s_, y_, spline_y_);
    }
    else
    {
      printf("                              NO SPLINE CALCULATED                                \n");
      printf("number of points for x = %d and y = %d differ \n", int(_x.size()), int(_y.size()));
    }
  }

  /// calcTraj calculates a trajectory between 2 given lines
  // &_other: trajectory for comparision
  // _weighting: weighting factor for this trajectory
  // _offset: defines an offset that should be calculated to the traj in y direction
  CTrajectory calcTraj(CTrajectory &_other, double _weighting, double _offset)
  {
    std::vector<double> new_x, new_y;
    new_x.reserve(this->vec_x_.size());
    new_y.reserve(this->vec_y_.size());

    if (this->vec_x_.size() == _other.vec_x_.size())
    {
      for (int i = 0; i < this->vec_x_.size(); ++i)
      {
        new_x.emplace_back((this->vec_x_.at(i) * _weighting + _other.vec_x_.at(i)) / (_weighting + 1));
        new_y.emplace_back((this->vec_y_.at(i) * _weighting + _other.vec_y_.at(i)) / (_weighting + 1) + _offset);
      }
    }

    return CTrajectory(new_x, new_y);
  }

  double calcCurvature(double _s)
  {
    double x, dx, ddx, y, dy, ddy;
    spline1ddiff(spline_x_, _s, x, dx, ddx);
    spline1ddiff(spline_y_, _s, y, dy, ddy);
    double nominator = dx * ddy - dy * ddx;
    double denominator = dx * dx + dy * dy;
    if (denominator == 0.0)
    {
      printf("curvature couldn't be calculated \n");
      printf("denom = 0.0 \n");
      return 0.0;
    }
    denominator = std::sqrt(denominator * denominator * denominator);
    return nominator / denominator * M_PI / 180; // use curvature in rad ?!
  };

  void calcLinLength(std::vector<double> _x, std::vector<double> _y)
  {
    double x0(0.0), x1(_x.front()), y0(0.0), y1(_y.front());
    // it is assumed that the vehicle is driving in the same direction like the traj for easier calculation
    s_lin.emplace_back(std::sqrt((x1 - x0) * (x1 - x0)));
    for (size_t i = 0; i < _x.size() - 1; i++)
    {
      x0 = _x.at(i);
      x1 = _x.at(i + 1);
      y0 = _y.at(i);
      y1 = _y.at(i + 1);
      s_lin.emplace_back(std::sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0)) + s_lin.back());
    }
  };

  double getLinLength() { return spline_lin_length_; };

  alglib::spline1dinterpolant getSplineInterpolant(char _name)
  {
    if (_name == 'x' || _name == 'X')
      return spline_x_;
    else
      return spline_y_;
  }

private:
  std::vector<double> vec_x_, vec_y_;

  std::vector<double> s_lin;
  double v_ = 0.0;                 // planned velocity of the trajectory
  double spline_lin_length_ = 0.0; // linear length of the spline

  alglib::real_1d_array x_, y_;                     // x, y arrays of the trajectory
  alglib::real_1d_array s_;                         // s: travelled distance on the spline + init dist
  alglib::spline1dinterpolant spline_x_, spline_y_; // structures that hold splines of x and y parametrized by s
};

#endif //TRAJECTORY_H
