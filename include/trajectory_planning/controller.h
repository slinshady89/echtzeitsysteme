#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <array>
#include <sensor_msgs/Range.h>
#include "ros/ros.h"

static const size_t arraySize = 5;

//class CControllerImpl;
class CController
{
public:
    CController(double _K_P /*direct gain to err*/, 
                double _K_I /*gain of integrated err*/, 
                double _K_D /*gain of derived err*/, 
                double _dt /*loop interval time*/, 
                double _limit /*+-steering limit*/):
                K_P(_K_P = 0.0),
                K_I(_K_I = 0.0),
                K_D(_K_D = 0.0),
                dt(_dt = 2.0),
                limit(_limit = 0.0)
    {};

    ~CController();
    /*computes the steering angle of the errors calculated by the the 
    * next 5 timesteps of the desired trajectory in reference to the
    * actually driven path.
    * input: errors of the next 5 timesteps
    * output: steering wheel angle
    */
    double computeSteering(std::array<double,arraySize> _arrTrajErrors);
    void setCtrlParams(double P, double I, double D, double t, double lim);
    double detectCntSteer(double _err, double _preError);
    bool ctrlInit();
    bool ctrlLoop(double _rangeUSL, double _rangeUSR, double _rangeUSF);
    bool ctrlLoop(sensor_msgs::Range _rangeUSL, sensor_msgs::Range _rangeUSR, sensor_msgs::Range _rangeUSF);
    bool ctrlDone();
    
    double getUsMinDist(){        return usMinDist;    };
    void setUsMinDist(double _dist){      usMinDist = _dist;    };

    sensor_msgs::Range range_usl, range_usr, range_usf;

private:
    double usMinDist = 0.3;
    double K_P, K_I, K_D;
    std::array<double,arraySize> arrErrsWeighting; // weighting of errors of desired traj
    double dt;
    double limit;
    double preError;
    double integral;
    double lastError, actError; // subtraction of preError and err in t-1 and t
    double limI;
};


#endif // CONTROLLER_H_