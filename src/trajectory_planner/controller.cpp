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

/*
    @param range_usl, range_usr, range_usf input ranges from ultrasound sensors
    @return if something is detected near vehicle
*/
bool CController::ctrlLoop(double range_usl, double range_usr, double range_usf){
    moving_usf.insert(moving_usf.end(), range_usf);
    moving_usr.insert(moving_usr.end(), range_usr);
    moving_usl.insert(moving_usl.end(), range_usl);

    moving_usf.erase(moving_usf.begin());
    moving_usr.erase(moving_usr.begin());
    moving_usl.erase(moving_usl.begin());

    double usf_avg = std::accumulate(moving_usf.begin(), moving_usf.end(), 0.0) / moving_usf.size();
    double usr_avg = std::accumulate(moving_usr.begin(), moving_usr.end(), 0.0) / moving_usr.size();
    double usl_avg = std::accumulate(moving_usl.begin(), moving_usl.end(), 0.0) / moving_usl.size();

    double minDist = getUsMinDist();
    ROS_INFO("minDist = %f", minDist);

    if (usf_avg < 1.0) ROS_INFO("US front: %f",(float) usf_avg);     
    if (usr_avg < 1.0) ROS_INFO("US left: %f",(float) usr_avg);     
    if (usl_avg < 1.0) ROS_INFO("US right: %f",(float) usl_avg); 
    
    if(usf_avg < minDist || usr_avg < minDist || usl_avg < minDist)
    {
        ROS_INFO("Ultrasonic sensor is detecting something closer than: %f", minDist);
        return false;
    }

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

double CController::computeSteering(double _err)
{
    double steer = 0.0;
    //if(_err > 0) _err *= 2;
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
    return true;
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
