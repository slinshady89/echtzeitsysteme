#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <array>
#include <sensor_msgs/Range.h>
#include "ros/ros.h"
#include <echtzeitsysteme/ControllerConfig.h>
#include <dynamic_reconfigure/server.h>
#include <numeric>

static const size_t arraySize = 5;

//class CControllerImpl;
class CController
{
public:
  CController () = default;
  CController(CController && var) = default;
  CController(const CController& var) = default;
  CController& operator=( CController && var) = default;
  CController& operator=(const CController& var) = default;
  
  
    CController(double _K_P /*direct gain to err*/, 
                double _K_I /*gain of integrated err*/, 
                double _K_D /*gain of derived err*/, 
                double _dt /*loop interval time in s*/, 
                double _limit /*+-steering limit*/):
                K_P(_K_P = 0.0),
                K_I(_K_I = 0.0),
                K_D(_K_D = 0.0),
                dt(_dt = 0.1),
                limit(_limit = 1000),
                dErrorLast(0.0), 
                dErrorAct(0.0),
                deratingWeightFact(0.8),
                integral(0.0),
                errCnt(0)
    {};

    ~CController();
    /*computes the steering angle of the errors calculated by the the 
    * next 5 timesteps of the desired trajectory in reference to the
    * actually driven path.
    * input: errors of the next 5 timesteps
    * output: steering wheel angle
    */
    double computeSteeringErrs(std::array<double,arraySize> _arrTrajErrors);
    double computeSteeringTraj(std::vector<double> _vecTrajErrors);
    double computeSteering(double err);
    void setCtrlParams(double P, double I, double D);
    bool ctrlInit();
    bool ctrlLoop(double _rangeUSL, double _rangeUSR, double _rangeUSF);
    bool ctrlLoop(sensor_msgs::Range _rangeUSL, sensor_msgs::Range _rangeUSR, sensor_msgs::Range _rangeUSF);
    bool ctrlDone();
    //void ctrlParamCallback(echtzeitsysteme::ControllerConfig &config, uint32_t level);

    double getUsMinDist(){        return usMinDist;    };
    void setUsMinDist(double _dist){      usMinDist = _dist;    };

    sensor_msgs::Range range_usl, range_usr, range_usf;

private:
    double usMinDist = 0.25;
    double K_P, K_I, K_D;
    std::array<double,arraySize> arrErrsWeighting; // weighting of errors of desired traj
    std::vector<double> vecErrsWeights;
    std::vector<double> moving_usf{0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> moving_usr{0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> moving_usl{0.0, 0.0, 0.0, 0.0, 0.0};
public:
  const std::vector<double> &getVecErrsWeights() const;

  void setVecErrsWeights(const std::vector<double> &vecErrsWeights);

private:
  double dt;
    double limit;
    double preError;
    double integral;
    double dErrorLast, dErrorAct; // subtraction of preError and err in t-1 and t
    double deratingWeightFact;
    size_t errCnt;
};


#endif // CONTROLLER_H_
