#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Point.h>
#include <trajectory_planning/trajectory.h>
#include <trajectory_planning/vehModel.h>
#include "trajectory_planning/polynomialRegression.h"

int vel = 0;
int steer = 0;
int ctrl_dist = 0;
std::vector<double> left_line_x, left_line_y, center_line_x, center_line_y, right_line_x, right_line_y;

void clearLineVecs();


/*
 * callback function fpr trajectory custom points message
 * iterate over each Point in msg and save it in trajectory
 */
void leftLineCallback(const echtzeitsysteme::points::ConstPtr &msg)
{
  ROS_INFO("LL Callback");
  if(!msg->points.empty())
  {
    for(auto it : msg->points) {
      left_line_x.emplace_back(it.x);
      left_line_y.emplace_back(it.y);
    }
  }
}
void rightLineCallback(const echtzeitsysteme::points::ConstPtr &msg)
{
  ROS_INFO("RL Callback");
  if(!msg->points.empty())
  {
    for(auto it : msg->points) {
      right_line_x.emplace_back(it.x);
      right_line_y.emplace_back(it.y);
    }
  }
}
void centerLineCallback(const echtzeitsysteme::points::ConstPtr &msg)
{
  ROS_INFO("CL Callback");
  if(!msg->points.empty())
  {
    for(auto it : msg->points) {
      center_line_x.emplace_back(it.x);
      center_line_y.emplace_back(it.y);
    }
  }
}

void uslCallback(const sensor_msgs::Range::ConstPtr &uslMsg, sensor_msgs::Range *usl)
{
  *usl = *uslMsg;
}

void usfCallback(const sensor_msgs::Range::ConstPtr &usfMsg, sensor_msgs::Range *usf)
{
  *usf = *usfMsg;
}

void usrCallback(const sensor_msgs::Range::ConstPtr &usrMsg, sensor_msgs::Range *usr)
{
  *usr = *usrMsg;
}

void ctrlParamCallback(echtzeitsysteme::ControllerConfig &config, CController *ctrl)
{
  ROS_INFO("Reconfigure Request: %f %f %f %d %d %d %d",
           config.K_P,
           config.K_I,
           config.K_D,
           config.vel,
           config.steering,
           config.size,
           config.ctrlDist);

  ctrl->setCtrlParams(config.K_P, config.K_I, config.K_D);
  vel = config.vel;
  steer = config.steering;
  ctrl_dist = config.ctrlDist;
}

/**
 * Here comes the tracejtory planning magic
 */
int main(int argc, char **argv)
{

  std::vector<double> sums, errors;
  double looptime = .2;

  std_msgs::Int16 velocity, steering;
  sensor_msgs::Range usr, usf, usl;

  velocity.data = 0;
  steering.data = 0;

  ros::init(argc, argv, "trajectory_planning");

  dynamic_reconfigure::Server<echtzeitsysteme::ControllerConfig> server;
  dynamic_reconfigure::Server<echtzeitsysteme::ControllerConfig>::CallbackType f;

  ros::NodeHandle nh;
  ros::Publisher motorCtrl = nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
  ros::Publisher steeringCtrl = nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);

  ros::Subscriber leftLineSub = nh.subscribe("left_line", 1, leftLineCallback);
  ros::Subscriber rightLineSub = nh.subscribe("right_line", 1, rightLineCallback);
  ros::Subscriber centerLineSub = nh.subscribe("center_line", 1, centerLineCallback);


  echtzeitsysteme::points trajectory_points;
  ros::Publisher trajectory = nh.advertise<echtzeitsysteme::points>("trajectory", 1);   //TODO: change buffer size

  // generate subscriber for us-sensor messages
  ros::Subscriber usrSub = nh.subscribe<sensor_msgs::Range>("/uc_bridge/usr", 1, boost::bind(usrCallback, _1, &usr));
  ros::Subscriber uslSub = nh.subscribe<sensor_msgs::Range>("/uc_bridge/usl", 1, boost::bind(uslCallback, _1, &usl));
  ros::Subscriber usfSub = nh.subscribe<sensor_msgs::Range>("/uc_bridge/usf", 1, boost::bind(usfCallback, _1, &usf));

  CController ctrl(17.5, 3.5, 0.1, 0.1, 1000);

  // Loop starts here:
  ros::Rate loop_rate(1 / looptime);
  
  f = boost::bind(&ctrlParamCallback, _1, &ctrl);
  server.setCallback(f);

  //Check if vectors are filled with data
/*
  while(left_line_x.size()<2 || left_line_y.size()<2)
  {
    ROS_INFO("Waiting for left line...");
    ros::spinOnce();
    loop_rate.sleep();
  }

<<<<<<< HEAD
=======
  right_line_x = {0.6, 0.79, 0.98, 1.15, 1.37, 1.56, 1.74};
  right_line_y = {-0.2, -0.19, -0.2, -0.19, -0.21, -0.2, -0.21};
>>>>>>> master

  //wait until the received message has enough points to build a cubic spline
  while(right_line_x.size()<2 || right_line_y.size()<2)
  {
    ROS_INFO("Waiting for left line...");
    ros::spinOnce();
    loop_rate.sleep();
  }

 */
  /*
  while(center_line_x.empty() && center_line_y.empty())
  {
    ROS_INFO("Waiting for center line...");
    ros::spinOnce();
    loop_rate.sleep();
  }
   */


  ROS_INFO("Loop start!");

  while (ros::ok())
  {
      //if (right_line_x.size()>2 && right_line_y.size()>2)
      {
        // calculate splines of the given set of points
        // TODO: a test for size > 2 should be done here

        right_line_x = {0.6, 0.79, 0.98, 1.15, 1.37, 1.56, 1.74};
        right_line_y = {-0.2, -0.19, -0.2, -0.19, -0.21, -0.2, -0.21};
        //CTrajectory left_line(left_line_x, left_line_y);
        CTrajectory rl = CTrajectory(right_line_x, right_line_y);
        //CTrajectory ll = CTrajectory(left_line_x, left_line_y);
        //CTrajectory center_line(center_line_x, center_line_y);
        CTrajectory traj = rl.calcTraj(rl, 1.0, 0.25);

        std::vector<double> curv_traj;
        // some validation check should be done!
        if (!ctrl.ctrlLoop(usl, usr, usf)) {
          ROS_INFO("Ultrasonic sensor is detecting something closer than: %f", ctrl.getUsMinDist());
          velocity.data = 0;
        } else {
          double v = 1.0; // 1m/s
          // this way the trajectories curvature is calculated at every
          // point this task is called if no new points at the are available the next point could be evaluated
          curv_traj = traj.calcCurvature(v / looptime);
        }

        VehicleModel veh(15.5, 25.5, 1000, -1000, 1000);

        std::vector<double> steering_deg;
        std::vector<int> steering_ctrl;
        steering_deg.reserve(traj.getVecWaypointDists().size());
        steering_ctrl.reserve(traj.getVecWaypointDists().size());

        for (auto it : curv_traj) {
          steering_deg.emplace_back(veh.calculateSteeringAngleDeg(it));
          //printf("steering_angle_deg : %.2f\n", steering_deg.back());
        }
        for (auto it : steering_deg) {
          steering_ctrl.emplace_back(veh.steeringAngleDegToSignal(it));
          //printf("steering_sig_ctrl : %.2f\n", steering_ctrl.back());
        }

        veh.setDesired_trajectory_(traj);

        ROS_INFO("calculated Steering: %.2f", (float) steering_ctrl.front());
        velocity.data = static_cast<short>(vel);
        ROS_INFO("vel: %d\n", velocity.data);
        motorCtrl.publish(velocity);

        trajectory_points.points.clear();
        // calc trajectory in aquidistant distances...

        std::vector<double> tX, tY;

        auto delta_dist = traj.getVecWaypointDists().back() / 100;
        for (auto waypoint = 0.0; waypoint < traj.getVecWaypointDists().back(); )  {
          trajectory_points.points.emplace_back(traj.getPointOnTrajAt(waypoint));
          tX.emplace_back(trajectory_points.points.back().x);
          tY.emplace_back(trajectory_points.points.back().y);
          //ROS_INFO("%.4f;%.4f \n", trajectory_points.points.back().x, trajectory_points.points.back().y);
          waypoint += delta_dist;
        }


        ROS_INFO("ctrl_dist = %.2f", ctrl_dist/100.0f);


        trajectory.publish(trajectory_points);
        
        std::vector<double> polynom;
        if(steer == 0)  steer = 5;
        ROS_INFO("Order of the Polynom (steer) = %d", steer);
        size_t order = steer;

        PolynomialRegression<double> poly;
        bool lq = poly.fitIt(tX,tY, order, polynom);


        // calc difference of spline and traj at supportive places of the spline
        /*
        int i = 0;
        auto diff = 0.0;
        for (auto x : tX){
          auto y = 0.0;
          for (size_t i = 0; i < polynom.size(); i++){
            y += polynom[i]* std::pow(x,i);
          }
          diff += y - tY[i];
          i++;
        }

         */


        // differentiation of polynom

        auto poly_y = 0.0;

        for (size_t i = 0; i < polynom.size(); i++){
          poly_y += polynom[i]* std::pow(ctrl_dist / 100,i);
        }
        auto poly_dy = 0.0;
        for (size_t i = 1; i < polynom.size(); i++){
          poly_dy += polynom[i]* std::pow(ctrl_dist / 100,i-1)*i;
        }

        auto poly_ddy = 0.0;
        for (size_t i = 2; i < polynom.size(); i++){
          poly_ddy += polynom[i]* std::pow(ctrl_dist / 100,i-2)*(i-1);
        }



        auto poly_denom = (1+poly_dy)*(1+poly_dy);
        poly_denom = std::sqrt(std::pow(poly_denom,3));
        auto poly_test_curv = poly_ddy / poly_denom;
        auto steering_angle_poly = veh.calculateSteeringAngleDeg(poly_test_curv);
        int steering_ctrl_poly(0);
        if (steering_angle_poly < 1 && steering_angle_poly > -1)
        {
          steering_ctrl_poly = -70;
        } else
        {
          steering_ctrl_poly = veh.steeringAngleDegToSignal(steering_angle_poly);
        }

        //auto curv_at = traj.calcCurvatureAt(ctrl_dist);
        ROS_INFO("calculated cruv: %.2f \n", poly_test_curv);
        //auto steering_angle_at = veh.calculateSteeringAngleDeg(curv_at);
        ROS_INFO("calculated steering angle: %.2f \n", steering_angle_poly);
        //auto steering_ctrl_at = veh.steeringAngleDegToSignal(steering_angle_at);
        ROS_INFO("calculated steering ctrl: %.d \n", steering_ctrl_poly);

        //ROS_INFO("Length of trajectory %.2f \n", float(dist + (traj.getVecWaypointDists()).back()));
        //ROS_INFO("number of points %d \n", (int) (traj.getVecWaypointDists()).size());

        // publishs the steering input at the first
        steering.data = static_cast<short>(steering_ctrl_poly);


        steeringCtrl.publish(steering);
      }
    clearLineVecs();
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();

  return 0;
}


void clearLineVecs(){
  left_line_x.clear();
  left_line_y.clear();
  center_line_x.clear();
  center_line_y.clear();
  right_line_x.clear();
  right_line_y.clear();
}

