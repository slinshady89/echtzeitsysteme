#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Point.h>
#include <trajectory_planning/trajectory.h>
#include <trajectory_planning/vehModel.h>
#include "trajectory_planning/polynomialRegression.h"

int vel = 0;
int steer = 0;
int ctrl_dist = 0;
std::vector<double> left_line_x, left_line_y, center_line_x, center_line_y, right_line_x, right_line_y, used_line_x, used_line_y;

void clearLineVecs();

/*
 * callback function fpr trajectory custom points message
 * iterate over each Point in msg and save it in trajectory
 */
void leftLineCallback(const echtzeitsysteme::points::ConstPtr &msg)
{
  //ROS_INFO("LL Callback");
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
  //ROS_INFO("RL Callback");
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
  //ROS_INFO("CL Callback");
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

 */
  //right_line_x = {0.6, 0.79, 0.98, 1.15, 1.37, 1.56, 1.74};
  //right_line_y = {-0.2, -0.19, -0.2, -0.19, -0.21, -0.2, -0.21};
  /*

  //wait until the received message has enough points to build a cubic spline
  while(right_line_x.size()<2 || right_line_y.size()<2)
  {
    ROS_INFO("Waiting for left line...");
    ros::spinOnce();
    loop_rate.sleep();
  }

  while(center_line_x.empty() && center_line_y.empty())
  {
    ROS_INFO("Waiting for center line...");
    ros::spinOnce();
    loop_rate.sleep();
  }
   */

  bool usedLeftLine = false;
  ROS_INFO("Loop start!");

  while (ros::ok())
  {
    if (left_line_x.size() > right_line_x.size() && left_line_y.size() > right_line_y.size())
    {
      used_line_x = left_line_x;
      used_line_y = left_line_y;
      usedLeftLine = true;
    } else
    {
      used_line_x = right_line_x;
      used_line_y = right_line_y;
      usedLeftLine = false;
    }

    if (used_line_x.size()>2 && used_line_y.size()>2)
      {
        // calculate splines of the given set of points

        //CTrajectory left_line(left_line_x, left_line_y);
        //CTrajectory rl = CTrajectory(right_line_x, right_line_y);
        //CTrajectory ll = CTrajectory(left_line_x, left_line_y);
        //CTrajectory center_line(center_line_x, center_line_y);
        CTrajectory usedLine = CTrajectory(used_line_x, used_line_y);
        CTrajectory traj = usedLine.calcTraj(usedLine, 1.0, (usedLeftLine == true ? -0.2 < : 0.2));

        std::vector<double> curv_traj;
        // some validation check should be done!
        // Funktioniert überhaupt nicht!
        if (!ctrl.ctrlLoop(usl, usr, usf)) {
          ROS_INFO("Ultrasonic sensor is detecting something closer than: %f", ctrl.getUsMinDist());
          velocity.data = 0;
        } else {
          double v = 1.0; // 1m/s
          // this way the trajectories curvature is calculated at every
          // point this task is called if no new points at the are available the next point could be evaluated
          velocity.data = static_cast<short>(vel);
          curv_traj = traj.calcCurvature(v / looptime);
        }

        VehicleModel veh(15.5, 25.5, 1000, -1000, 1000);

        veh.setDesired_trajectory_(traj);

        // remove for collision detection
        velocity.data = static_cast<short>(vel);
        ROS_INFO("vel: %d\n", velocity.data);
        motorCtrl.publish(velocity);

        trajectory_points.points.clear();
        // calc trajectory in aquidistant distances...

        std::vector<double> tX, tY, curv;

        auto delta_dist = traj.getVecWaypointDists().back() / 100;
        for (auto waypoint = 0.0; waypoint < traj.getVecWaypointDists().back(); )  {
          trajectory_points.points.emplace_back(traj.getPointOnTrajAt(waypoint));
          tX.emplace_back(trajectory_points.points.back().x);
          tY.emplace_back(trajectory_points.points.back().y);
          curv.emplace_back(traj.calcCurvatureAt(waypoint));
          //ROS_INFO("%.4f;%.4f, %.4f", trajectory_points.points.back().x, trajectory_points.points.back().y, curv.back());
          waypoint += delta_dist;
        }

        ROS_INFO("ctrl_dist = %.2f", ctrl_dist/100.0f);

        trajectory.publish(trajectory_points);
        
        std::vector<double> polynom;

        PolynomialRegression<double> poly;
        bool lq = poly.fitIt(tX,tY, /*order*/5, polynom);

        // calculate steering angle in an area around the
        double weightDecreasingFact = 0.90;
        std::vector<double> weights;
        std::vector<double> errs;
        errs.emplace_back(tY.front());
        weights.emplace_back(1.0);
        for (double s = delta_dist; s < ctrl_dist/100.0f;){
          weights.emplace_back(weights.back()*weightDecreasingFact);
          auto err = traj.getPointOnTrajAt(s).y;
          errs.emplace_back(err);
          s += delta_dist;
        }
        ctrl.setVecErrsWeights(weights);

        auto steer_rescue = ctrl.computeSteeringTraj(errs);
        ROS_INFO("Steering with vecTraj: %f\n", steer_rescue);

        ROS_INFO("ERR AT CTRL_DIST: %.3f\n", (errs.back()));
        auto steer_single_point = ctrl.computeSteering(errs.back());
        ROS_INFO("Steering with single Point: %d\n", int(steer_single_point));

        auto poly_test_curv = poly.calcCurv(polynom, ctrl_dist / 100.0f);
        auto steering_angle_poly = veh.calculateSteeringAngleDeg(poly_test_curv);
        auto steering_ctrl_poly = veh.steeringAngleDegToSignal(steering_angle_poly);

        auto steering_ctrl = veh.steeringAngleDegToSignal(steer_single_point);

        //auto curv_at = traj.calcCurvatureAt(ctrl_dist);
        //ROS_INFO("calculated cruv: %.2f \n", poly_test_curv);
        //auto steering_angle_at = veh.calculateSteeringAngleDeg(curv_at);
        //ROS_INFO("calculated steering angle: %.2f \n", steering_angle_poly);
        //auto steering_ctrl_at = veh.steeringAngleDegToSignal(steering_angle_at);
        ROS_INFO("calculated steering ctrl: %.d \n", steering_ctrl);

        //ROS_INFO("Length of trajectory %.2f \n", float(dist + (traj.getVecWaypointDists()).back()));
        //ROS_INFO("number of points %d \n", (int) (traj.getVecWaypointDists()).size());

        // publishs the steering input at the first
        //steering.data = static_cast<short>(steering_ctrl_poly);

        // steer with PID onto a point
        steering.data = static_cast<short>(steering_ctrl);

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

