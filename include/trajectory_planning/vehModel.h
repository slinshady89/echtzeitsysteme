//
// Created by nils on 2/1/19.
//

#ifndef SPLINETEST_VEHMODEL_H
#define SPLINETEST_VEHMODEL_H

#include "trajectory.h"

class VehicleModel
{
public:

  VehicleModel () = default;
  VehicleModel(VehicleModel && var) = default;
  VehicleModel(const VehicleModel& var) = default;
  VehicleModel& operator=( VehicleModel && var) = default;
  VehicleModel& operator=(const VehicleModel& var) = default;


  VehicleModel(double _width, double _length, int _maxVelPos, int _maxVelNeg, int _absMaxSteering) :
  width_(_width),
  length_(_length),
  maxVelPos_(_maxVelPos),
  maxVelNeg(_maxVelNeg),
  absMaxSteering_(_absMaxSteering){
    length_rear_axle_to_cog_ = _length * length_to_cog_fact_;
  };

  VehicleModel(double width_, double length_, int absMaxSteering_, int maxVelPos_);

  // calculates the int steering given the curvature of the trajectory
  double calculateSteeringAngleDeg(double _curvatureTraj);
  int steeringAngleDegToSignal(double _steeringAngle);

  // some getter
  double getWidth_() const { return width_; }
  double getLength_() const { return length_; }
  int getAbsMaxSteering_() const { return absMaxSteering_; }
  int getMaxVelPos_() const { return maxVelPos_; }
  int getMaxVelNeg() const { return maxVelNeg; }

  void setWidth_(double width_) { VehicleModel::width_ = width_; }
  void setLength_(double length_) { VehicleModel::length_ = length_; }
  void setAbsMaxSteering_(int absMaxSteering_) { VehicleModel::absMaxSteering_ = absMaxSteering_; }
  void setMaxVelPos_(int maxVelPos_) { VehicleModel::maxVelPos_ = maxVelPos_; }
  void setMaxVelNeg(int maxVelNeg) { VehicleModel::maxVelNeg = maxVelNeg; }

  const std::array<double, 6> &getPolynom() const;

  const CTrajectory &getDesired_trajectory_() const;

  void setDesired_trajectory_(const CTrajectory &desired_trajectory_);

private:
  CTrajectory desired_trajectory_;
  double width_ = 0.0;
  double length_ = 0.0;
  int absMaxSteering_ = 1000;
  int maxVelPos_ = 1000;
  int maxVelNeg = -1000;
  double length_rear_axle_to_cog_ = length_ * 2 / 3;

protected:
  std::array<double, 6> polynom = {-0.0001, -0.0045, 0.0257, 0.9683, -44.0491, 57.8917};
  double length_to_cog_fact_= 0.6078;
};

/**
  alglib::real_1d_array steering_angle_ae__ ;//= "{22,21,19,18,17,16,15,13,11,9.5,8,6,4,3,2,1,-1,-2.5,-4,-6,-7.5,9,-10.5,-13.5,-15.5,-17.5,-20,-20.5,-22}";
  std::vector<double> steering_angle_deg__ = {22,21,19,18,17,16,15,13,11,9.5,8,6,4,3,2,1,-1,-2.5,-4,-6,-7.5,9,-10.5,-13.5,-15.5,-17.5,-20,-20.5,-22};
  std::vector<double> steering_ctrl_sig__ = {-1000, -925, -850, -775, -700, -625, -550, -475, -400, -325, -250, -175, -100, -50, -25, -10, 100, 175, 250, 325, 400, 475, 550, 625, 700, 775, 850, 925, 1000};
  alglib::real_1d_array steering_ctrl_sig_ae__ ;//= "{-1000, -925, -850, -775, -700, -625, -550, -475, -400, -325, -250, -175, -100, -50, -25, -10, 100, 175, 250, 325, 400, 475, 550, 625, 700, 775, 850, 925, 1000}";
  alglib::spline1dinterpolant steering_signal_over_angle;
*/

#endif //SPLINETEST_VEHMODEL_H
