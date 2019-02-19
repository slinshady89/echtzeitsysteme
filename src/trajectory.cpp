
//
// Created by nils on 2/3/19.
//

#include "trajectory_planning/trajectory.h"

void CTrajectory::calcLinLength(std::vector<double> _x, std::vector<double> _y)
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
}

alglib::spline1dinterpolant CTrajectory::getSplineInterpolant(char _name)
{
  if (_name == 'x' || _name == 'X')
    return spline_x_;
  else
    return spline_y_;
}

double CTrajectory::calcCurvatureAt(double _s)
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
  return nominator / denominator /* * M_PI / 180*/; // use curvature in rad ?!
}

CTrajectory CTrajectory::calcTraj(CTrajectory &_other, double _weighting, double _offset)
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

alglib::ae_int_t CTrajectory::getNatural_bound_type_() const
{
  return natural_bound_type_;
}

void CTrajectory::setNatural_bound_type_(alglib::ae_int_t natural_bound_type_)
{
  CTrajectory::natural_bound_type_ = natural_bound_type_;
}

std::vector<double> CTrajectory::calcCurvature(double _stepsize)
{
  printf("calculate curvature with stepsize: %.2f\n", _stepsize);
  std::vector<double> curv;
  auto vec_s = getVecWaypointDists();
  double step;
  if (vec_s.size() > 2)
    step = vec_s.at(1) - vec_s.at(0);
  else
    return curv;
  curv.reserve(static_cast<size_t>(vec_s.size() * step / _stepsize));
  double x, dx, ddx, y, dy, ddy;
  double x1, dx1, ddx1, y1, dy1, ddy1;
  double s, s1;
  for (size_t i = 0; i < vec_s.size() - 1; i++)
  {
    for (double j = vec_s.at(i); j < vec_s.at(i + 1);)
    {
      s = j - _stepsize;
      s1 = j + _stepsize;
      spline1ddiff(spline_x_, s, x, dx, ddx);
      spline1ddiff(spline_y_, s, y, dy, ddy);
      spline1ddiff(spline_x_, s1, x1, dx1, ddx1);
      spline1ddiff(spline_y_, s1, y1, dy1, ddy1);

      // additionally course angle could be calculated here as std::atan2(dy,dx)
      auto phi = dy / dx;
      auto phi1 = dy1 / dx1;
      curv.emplace_back((phi1 - phi) / _stepsize);

      j += _stepsize;
    }
  }

  return curv;
}
