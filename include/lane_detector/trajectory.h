
#include <vector>

class CPoint{
public:
    CPoint(double x = 0.0, double y = 0.0){
        m_x = x;
        m_y = y;
    };
private:
    double m_x;
    double m_y;
};

class CTrajectory : CPoint{
public:
    CTrajectory(CPoint point, double acc = 0.0, double curv = 0.0, 
                 double theta = 0.0, double s = 0)
    {
        m_point = point;    
        m_acceleration = acc;
        m_curvature = curv;
        m_theta = theta;
        m_s = s;
    };
private:
    CPoint m_point;
    double m_acceleration;
    double m_curvature; // curvature kappa
    double m_theta; // tangential angle of the traj
    double m_s; // driven dist on traj
};

class CController{
public:
    CController(double _K_P, double _K_I, double _K_D, double _dt, 
                double _limit, std::vector<double> _vecSums,
                std::vector<double> _vecErrors)
    {
        K_P = _K_P;
        K_I = _K_I;
        K_D = _K_D;
        dt = _dt;
        limit = _limit;
        previous_sums = _vecSums;
        previous_errors = _vecErrors;
    };
    double computeSteering(std::vector<double> _vecTrajErrors);
    void setCtrlParams(double P, double I, double D, double t);
    bool initCtrl();

    bool ctrlInit();
    bool ctrlLoop();
    bool ctrlDone();

private:
    double K_P, K_I, K_D;
    double arrErrs[5];
    double dt;
    double limit;
    std::vector<double> previous_sums;
    std::vector<double> previous_errors;
};