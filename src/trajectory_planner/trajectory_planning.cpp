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
double offset;

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
  ROS_INFO("Reconfigure Request: %f %f %f %d %d %d %d %f",
           config.K_P,
           config.K_I,
           config.K_D,
           config.vel,
           config.steering,
           config.size,
           config.ctrlDist,
           config.offset);

  ctrl->setCtrlParams(config.K_P, config.K_I, config.K_D);
  vel = config.vel;
  steer = config.steering;
  ctrl_dist = config.ctrlDist;
  offset = config.offset;
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

  CController ctrl(17.5, 3.5, 0.1, looptime, 1000);

  ros::Rate loop_rate(1 / looptime);
  
  f = boost::bind(&ctrlParamCallback, _1, &ctrl);
  server.setCallback(f);


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
        CTrajectory usedLine = CTrajectory(used_line_x, used_line_y);
        CTrajectory traj = usedLine.calcTraj(usedLine, 1.0, usedLeftLine ? -offset : offset);
        ROS_INFO("Offset for trajectory calculation = %f", offset);

        std::vector<double> curv_traj;
        ctrl.setUsMinDist(0.35);
        if (!ctrl.ctrlLoop(usl.range, usr.range, usf.range)) {
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
        ROS_INFO("vel: %d\n", velocity.data);
        motorCtrl.publish(velocity);

        trajectory_points.points.clear();

        // calc points on the trajectory in aquidistant distances to publish them
        auto delta_dist = traj.getVecWaypointDists().back() / 100;
        for (auto waypoint = 0.0; waypoint < traj.getVecWaypointDists().back(); )  {
          trajectory_points.points.emplace_back(traj.getPointOnTrajAt(waypoint));
          waypoint += delta_dist;
        }

        ROS_INFO("ctrl_dist = %.2f", ctrl_dist/100.0f);

        trajectory.publish(trajectory_points);

        ROS_INFO("ERR AT CTRL_DIST: %.3f\n", (traj.getPointOnTrajAt(ctrl_dist/100.0f).y));
        auto steer_single_point = ctrl.computeSteering(traj.getPointOnTrajAt(ctrl_dist/100.0f).y);


        auto steering_ctrl = veh.steeringAngleDegToSignal(steer_single_point);

        ROS_INFO("calculated steering ctrl: %.d \n", steering_ctrl);

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

