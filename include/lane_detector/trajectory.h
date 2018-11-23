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
    CTrajectory(CPoint point, double acc = 0.0, double curv = 0.0, double theta = 0.0, double s = 0){
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