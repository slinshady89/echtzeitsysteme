//
// Created by nils on 2/3/19.
//

#include "trajectory_planning/vehModel.h"

double VehicleModel::calculateSteeringAngleDeg(double _curvatureTraj)
{
  double steering_signal = 0.0;
  // if curvature is to big that simplification has to be assumed
  if ((1 / _curvatureTraj) * (1 / _curvatureTraj) < length_rear_axle_to_cog_ * length_rear_axle_to_cog_)
    return _curvatureTraj * length_;

  double denom = std::sqrt((1 / _curvatureTraj) * (1 / _curvatureTraj) - length_rear_axle_to_cog_ * length_rear_axle_to_cog_);
  steering_signal = std::atan(length_ / denom);

  return std::abs(steering_signal) > absMaxSteering_ ? absMaxSteering_ * std::abs(steering_signal) / steering_signal : steering_signal;
}

int VehicleModel::steeringAngleDegToSignal(double _steeringAngle)
{
  int steering_control = 0;
  auto poly = this->getPolynom();
  double x = _steeringAngle;
  steering_control = static_cast<int>(std::pow(x,5) * poly[0] + std::pow(x,4) * poly[1] + std::pow(x,3) * poly[2] + x*x * poly[3] + x * poly[4] + poly[5]);
  return std::abs(steering_control) > getAbsMaxSteering_() ? getAbsMaxSteering_() : steering_control;
}

const std::array<double, 6> &VehicleModel::getPolynom() const
{
  return polynom;
}

const CTrajectory &VehicleModel::getDesired_trajectory_() const
{
  return desired_trajectory_;
}

void VehicleModel::setDesired_trajectory_(const CTrajectory &desired_trajectory_)
{
  VehicleModel::desired_trajectory_ = desired_trajectory_;
}
