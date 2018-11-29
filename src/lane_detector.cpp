#include <lane_detector/lane_detector.h>
#include <lane_detector/controller.h>

std::vector<CTrajectory> trajPoints;




bool CController::ctrlInit(){
    arrErrsWeighting = {1.0, 0.8, 0.6, 0.4, 0.2};
    return true;
}; 

// some validation check should be done here
// calculate trajectory here? 
bool CController::ctrlLoop(){
    
    return true;
};

double CController::computeSteering(std::array<double,arraySize> _errs){
    double steer = 0.0;
    double err = 0.0;

    // calculate weighted error of the last 5 errors
    size_t i = 0;
    for(auto err_ : _errs)
    {
        err = err_ * arrErrsWeighting[i++];
        // necessary check since arrays are all initiated with arraySize
        if(i == arraySize-1)
            continue;
    }
    actError = (err - preError);
    // limit to the I-part of the controller so it stops raising if a countersteering onto the traj is detected
    if(abs(actError) >= abs(lastError))      integral += dt*err;
    else                                    integral -= dt*err;
    
    // maybe dont use err but kappa directly instead?
    double Pout = K_P * err;
    double Iout = K_I * integral;
    double derivate = 0.0;
    if(dt != 0.0) derivate = lastError / dt;
    double Dout = K_D *derivate;

    steer = Pout + Iout + Dout;
    if(steer > limit) steer = limit;
    if(steer < -limit) steer = -limit;

    lastError = (err - preError);

    preError = err;
    ctrlDone();
    return steer;
};

bool CController::ctrlDone(){
    for (size_t i = 0; i < arraySize ; i++){

    }
}

void CController::setCtrlParams(double P, double I, double D, double t, double lim){
    K_P = P;
    K_I = I;
    K_D = D;
    dt = t;
    limit = lim;
}