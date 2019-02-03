
//
// Created by nils on 2/3/19.
//

#include "trajectory_planning/trajectory.h"
#include <cmath>

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

double CTrajectory::calcCurvature(double _s)
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

// example trajectory calculation
{
    std::vector<double> left_line_x, left_line_y;
    std::vector<double> right_line_x, right_line_y;

    left_line_x = {0.1, 0.21, 0.30, 0.42, 0.5, 0.65};
    left_line_y = {0.2, 0.25, 0.33, 0.41, 0.48, 0.54};
    right_line_x = {0.11, 0.2, 0.32, 0.43, 0.52, 0.66};
    right_line_y = {-0.2, -0.17, -0.11, -0.02, 0.09, 0.14};

    CTrajectory left_line(left_line_x, left_line_y);
    CTrajectory right_line(right_line_x, right_line_y);

    CTrajectory traj = left_line.calcTraj(right_line, 0.0, 0.0);

    std::vector<double> spline_x, spline_y;

    spline_x.reserve(left_line_x.size());
    spline_y.reserve(left_line_x.size());
    for (auto it : traj.getVecWaypointDists())
    {
        spline_x.emplace_back(alglib::spline1dcalc(traj.getSplineInterpolant('x'), it));
        spline_y.emplace_back(alglib::spline1dcalc(traj.getSplineInterpolant('y'), it));
    }
}