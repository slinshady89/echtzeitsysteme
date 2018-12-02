#include "trajectory_planning/trajectory.h"
#include <cmath>

std::array<CPoint, 100> trajPoints[];

double m_pi_10 = M_PI / 10.0;

for(size_t i = 0; i < 100 ; i++){

    trajPoints[i].x = 0.1*i;
    trajPoints[i].y = (double) sin(m_pi_10*i);
};

std::vector<std::vector<CTrajectory>> CTrajectory::calculateTrajs(std::array<CPoint, 100> _trajPoints, size_t _stepSize, double _interpolationError){

};