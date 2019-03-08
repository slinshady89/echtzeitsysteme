#include <trajectory_planning/controller.h>
#include <trajectory_planning/trajectory.h>


std::vector<CTrajectory> trajPoints;

bool CController::ctrlInit(){
    arrErrsWeighting[0] =  1.0; //{1.0, 0.8, 0.6, 0.4, 0.2};
    for(size_t i = 1; i < arraySize; i++){
        arrErrsWeighting[i] = arrErrsWeighting[i-1]*0.8;
    }
    return true;
}; 


std::array<double,10> X, Y;
/*
void computeTraj(){

using namespace alglib;
    double lengthOfTraj = lengthOfTraj;
    for(double i = 0; i < 20; i++){
        X[i] = {i};
        Y[i] = {sin(2*M_PI/lengthOfTraj*i)};
    }


    real_1d_array x, y;
    x.setcontent(X.size(), &X[0]);
    y.setcontent(Y.size(), &Y[0]);
    //
    // We use cubic spline to interpolate f(x)=x^2 sampled 
    // at 5 equidistant nodes on [-1,+1].
    //
    // First, we use default boundary conditions ("parabolically terminated
    // spline") because cubic spline built with such boundary conditions 
    // will exactly reproduce any quadratic f(x).
    //
    // Then we try to use natural boundary conditions
    //     d2S(-1)/dx^2 = 0.0
    //     d2S(+1)/dx^2 = 0.0
    // and see that such spline interpolated f(x) with small error.
    //
    //real_1d_array x = "[-1.0,-0.5,0.0,+0.5,+1.0]";
    //real_1d_array y = "[+1.0,0.25,0.0,0.25,+1.0]";
    double t = 2 * M_PI / 20 * 7.0;
    double v;
    spline1dinterpolant s;
    ae_int_t natural_bound_type = 2;
    //
    // Test exact boundary conditions: build S(x), calculare S(0.25)
    // (almost same as original function)
    //
    spline1dbuildcubic(x, y, s);
    v = spline1dcalc(s, t);
    ROS_INFO("%.4f\n", double(v)); // EXPECTED: 0.801016

    //
    // Test natural boundary conditions: build S(x), calculare S(0.25)
    // (small interpolation error)
    //
    spline1dbuildcubic(x, y, 5, natural_bound_type, 0.0, natural_bound_type, 0.0, s);
    v = spline1dcalc(s, t);
    ROS_INFO("%.3f\n", double(v)); // EXPECTED: 0.801016
}
double computeVarErr(){
    for(double i = 0; ; i++){
        return 0.2*sin(2*M_PI/20.0*i);
    }
}

*/


// some validation check should be done here
// calculate trajectory here? 
bool CController::ctrlLoop(sensor_msgs::Range _rangeUSL, sensor_msgs::Range _rangeUSR, sensor_msgs::Range _rangeUSF){
    double minDist = getUsMinDist();
    if (_rangeUSF.range < 1.0)        ROS_INFO("US front: %f",(float) _rangeUSF.range);     
    if (_rangeUSL.range < 1.0)        ROS_INFO("US left: %f",(float) _rangeUSL.range);     
    if (_rangeUSR.range < 1.0)        ROS_INFO("US right: %f",(float) _rangeUSR.range);
  if (_rangeUSL.range <= minDist) {
    if (_rangeUSL.range != 0) return false;
  }
  if (_rangeUSR.range <= minDist) {
    if (_rangeUSR.range != 0) return false;
  }
  if (_rangeUSF.range <= minDist) {
    if (_rangeUSF.range != 0) return false;
  }
};

bool CController::ctrlLoop(double range_usl, double range_usr, double range_usf){
    double minDist = getUsMinDist();
    if (range_usf < 1.0)        ROS_INFO("US front: %f",(float) range_usf);     
    if (range_usf < 1.0)        ROS_INFO("US left: %f",(float) range_usl);     
    if (range_usf < 1.0)        ROS_INFO("US right: %f",(float) range_usr); 
    
    if(range_usl <= minDist || range_usr <= minDist || range_usf <= minDist)
        return false;
    

    return true;
};


/** computeSteeringErrs
 * input: array with errors of planned trajectory and actual path
 * 
 */
double CController::computeSteeringErrs(std::array<double,arraySize> _errs)
{
    double steer = 0.0;
    double err = 0.0;

    // calculate weighted error of the last 5 errors
    size_t i = 0;
    for(auto err_ : _errs)
    {   
        ROS_INFO("CController: err_ = %f", err_);
        ROS_INFO("CController: weighted err = %f", err);
        err = err_ * arrErrsWeighting[i++];
        // necessary check since arrays are all initiated with arraySize
        if(i == arraySize-1)
            continue;
    }
    dErrorAct = (err - preError);
    // limit to the I-part of the controller so it stops raising if a countersteering onto the traj is detected
    if(abs(dErrorAct) >= abs(dErrorLast))      integral += dt*err;
    else                                    integral -= dt*err;
    
    // maybe don't use err but kappa directly instead?
    double Pout = K_P * err;
    double Iout = K_I * integral;
    double derivate = 0.0;
    if(dt != 0.0) derivate = dErrorAct / dt;
    double Dout = K_D *derivate;

    steer = Pout + Iout + Dout;
    if(steer > limit) steer = limit;
    if(steer < -limit) steer = -limit;

    dErrorLast = (err - preError);

    preError = err;
    ctrlDone();
    return steer;
};

double CController::computeSteering(double& _err)
{
    double steer = 0.0;

    dErrorAct = (_err - preError);
    // limit to the I-part of the controller so it stops raising if a counter-steering onto the traj is detected
    if(abs(dErrorAct) >= abs(dErrorLast))       integral += dt*_err;
    else                                        integral -= dt*_err;
    
    if(_err < FLT_EPSILON && integral > FLT_EPSILON) errCnt++;
    else errCnt = 0;
    if(errCnt > 5) integral = 0.0; 

    // maybe don't use err but kappa directly instead?
    double p_out = K_P *_err;
    double i_out = K_I * integral;
    double d_err(0.0);
    if(dt != 0.0) d_err = dErrorAct / dt;
    double d_out = K_D * d_err;
   
    steer = p_out + i_out + d_out;

    if(steer > limit)   steer = limit;
    if(steer < -limit)  steer = -limit;

    dErrorLast = _err - preError;

    preError = _err;
    ctrlDone();
    ROS_INFO("error: %f   ==>   P: %f  I: %f  D: %f   ==>  steer: %f", _err, p_out, i_out, d_out, steer);
    return steer;
};




bool CController::ctrlDone(){
 //   for (size_t i = 0; i < arraySize ; i++){   }
    return true;
}

void CController::setCtrlParams(double P, double I, double D, double t, double lim){
    K_P = P;
    K_I = I;
    K_D = D;
    dt = t;
    limit = lim;
}
void CController::setCtrlParams(double P, double I, double D){
    K_P = P;
    K_I = I;
    K_D = D;
}

CController::~CController()
{
    integral = 0;
}

double CController::computeSteeringTraj(std::vector<double> _vecTrajErrors) {
    double steer = 0.0;
    double err = 0.0;

    for(size_t i = 0; i < vecErrsWeights.size();i++)
    {
        err = _vecTrajErrors.at(i) * vecErrsWeights.at(i);
    }
    dErrorAct = (err - preError);
    // limit to the I-part of the controller so it stops raising if a countersteering onto the traj is detected
    if(abs(dErrorAct) >= abs(dErrorLast))      integral += dt*err;
    else                                    integral -= dt*err;

    // maybe don't use err but kappa directly instead?
    double Pout = K_P * err;
    double Iout = K_I * integral;
    double derivate = 0.0;
    if(dt != 0.0) derivate = dErrorAct / dt;
    double Dout = K_D *derivate;

    steer = Pout + Iout + Dout;
    if(steer > limit) steer = limit;
    if(steer < -limit) steer = -limit;

    dErrorLast = (err - preError);

    preError = err;
    ctrlDone();
    return steer;
}

const std::vector<double> &CController::getVecErrsWeights() const {
    return vecErrsWeights;
}

void CController::setVecErrsWeights(const std::vector<double> &vecErrsWeights) {
    CController::vecErrsWeights = vecErrsWeights;
}
