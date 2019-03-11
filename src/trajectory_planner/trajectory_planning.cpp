#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Point.h>
#include <trajectory_planning/trajectory.h>
#include <trajectory_planning/vehModel.h>
#include <trajectory_planning/polynomialRegression.h>

int vel = 0;
int steer = 0;
int ctrl_dist = 0;
std::vector<double> left_line_x, left_line_y, center_line_x, center_line_y, right_line_x, right_line_y, used_line_x, used_line_y;
double offset;

void clearLineVecs();

/*!
 * @brief callback function for detected left line 
 * @param[in] ros msg containing custom msg type for left line data points 
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

/*!
 * @brief callback function for detected right line 
 * @param[in] ros msg containing custom msg type for right line data points 
 */
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

/*!
 * @brief callback function for detected center line 
 * @param[in] ros msg containing custom msg type for center line data points 
 */
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

/*!
 * Callback function for the left ultrasound sensor range.
 * @param[in] ros msg containing the range msg from the left sensor
 */
void uslCallback(const sensor_msgs::Range::ConstPtr &uslMsg, sensor_msgs::Range *usl)
{
  *usl = *uslMsg;
}

/*!
 * Callback function for the front ultrasound sensor range.
 * @param[in] ros msg containing the range msg from the front sensor
 */
void usfCallback(const sensor_msgs::Range::ConstPtr &usfMsg, sensor_msgs::Range *usf)
{
  *usf = *usfMsg;
}

/*!
 * Callback function for the right ultrasound sensor range.
 * @param[in] ros msg containing the range msg from the right sensor
 */
void usrCallback(const sensor_msgs::Range::ConstPtr &usrMsg, sensor_msgs::Range *usr)
{
  *usr = *usrMsg;
}

/*!
 * @brief callback function for dynamic reconfigure service
 * @param custom configuration for controller settings from rqt_reconfigure
 */
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

/*
 * Here starts the tracejtory planning magic
 */
int main(int argc, char **argv)
{

  std::vector<double> sums, errors;
  double looptime = .2;

  std_msgs::Int16 velocity, steering;
  sensor_msgs::Range usr, usf, usl;

  velocity.data = 0;
  steering.data = 0;

  //! controller object
  CController ctrl(17.5, 3.5, 0.1, looptime, 1000);

  ros::init(argc, argv, "trajectory_planning");

  dynamic_reconfigure::Server<echtzeitsysteme::ControllerConfig> server;
  dynamic_reconfigure::Server<echtzeitsysteme::ControllerConfig>::CallbackType f;

  ros::NodeHandle nh;

  /*
   * register publishing msgs at ros master
   */
  ros::Publisher motorCtrl = nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
  ros::Publisher steeringCtrl = nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);

  /*
   * subscribe to ros msgs and define callback functions
   */
  ros::Subscriber leftLineSub = nh.subscribe("left_line", 1, leftLineCallback);
  ros::Subscriber rightLineSub = nh.subscribe("right_line", 1, rightLineCallback);
  ros::Subscriber centerLineSub = nh.subscribe("center_line", 1, centerLineCallback);

  //! custom points msg
  echtzeitsysteme::points trajectory_points;
  ros::Publisher trajectory = nh.advertise<echtzeitsysteme::points>("trajectory", 1);   //TODO: change buffer size

  //! generate subscriber for ultrasound sensors messages with a second attribute
  ros::Subscriber usrSub = nh.subscribe<sensor_msgs::Range>("/uc_bridge/usr", 1, boost::bind(usrCallback, _1, &usr));
  ros::Subscriber uslSub = nh.subscribe<sensor_msgs::Range>("/uc_bridge/usl", 1, boost::bind(uslCallback, _1, &usl));
  ros::Subscriber usfSub = nh.subscribe<sensor_msgs::Range>("/uc_bridge/usf", 1, boost::bind(usfCallback, _1, &usf));

  //! define rqt_reconfigure server callback function
  f = boost::bind(&ctrlParamCallback, _1, &ctrl);
  server.setCallback(f);

  //! set rate for while loop
  ros::Rate loop_rate(1 / looptime);
  
  bool usedLeftLine = false;

  ROS_INFO("Loop start!");
  while (ros::ok())
  {
    //! use the longest detected line for trajectory calculation
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

    //! only calculation trajectory if the size of the detected line is grater than 2
    //! this is needed by spline interpolation
    if (used_line_x.size()>2 && used_line_y.size()>2)
      {
        //! generate trajectory object based on the longest detected line and calculate trajectory
        CTrajectory usedLine = CTrajectory(used_line_x, used_line_y);
        CTrajectory traj = usedLine.calcTraj(usedLine, 1.0, usedLeftLine ? -offset : offset);
        ROS_INFO("Offset for trajectory calculation = %f", offset);

        std::vector<double> curv_traj;
        ctrl.setUsMinDist(0.35);
        if (!ctrl.ctrlLoop(usl.range, usr.range, usf.range)) {
          //velocity.data = 0;
        } else {
          double v = 1.0; // 1m/s
          // this way the trajectories curvature is calculated at every
          // point this task is called if no new points at the are available the next point could be evaluated
          velocity.data = static_cast<short>(vel);
          curv_traj = traj.calcCurvature(v / looptime);
        }
        velocity.data = static_cast<short>(vel);

        //! setup vehicle model object and set calculated trajectory
        VehicleModel veh(15.5, 25.5, 1000, -1000, 1000);
        veh.setDesired_trajectory_(traj);

        // remove for collision detection
        ROS_INFO("vel: %d\n", velocity.data);
        motorCtrl.publish(velocity);

        //! calc points on the trajectory in equidistant distances to publish them
        auto delta_dist = traj.getVecWaypointDists().back() / 100;
        for (auto waypoint = 0.0; waypoint < traj.getVecWaypointDists().back(); )  {
          trajectory_points.points.emplace_back(traj.getPointOnTrajAt(waypoint));
          waypoint += delta_dist;
        }

        trajectory.publish(trajectory_points);

        // calculate steering angle in an area around the
        double weightDecreasingFact = 0.95;
        std::vector<double> weights;
        std::vector<double> errs;
        errs.emplace_back(trajectory_points.points.front().y);
        weights.emplace_back(1.0);
        for (double s = delta_dist; s < ctrl_dist/100.0f;){
          weights.emplace_back(weights.back()*weightDecreasingFact);
          auto err = traj.getPointOnTrajAt(s).y;
          errs.emplace_back(err);
          s += delta_dist;
        }
        std::reverse(weights.begin(), weights.end());
        ctrl.setVecErrsWeights(weights);

        auto steer_rescue = ctrl.computeSteeringTraj(errs);
        ROS_INFO("Steering with vecTraj: %f\n", steer_rescue);

        ROS_INFO("ERR AT CTRL_DIST: %.3f\n", (errs.back()));

        ROS_INFO("ERR AT CTRL_DIST: %.3f\n", (traj.getPointOnTrajAt(ctrl_dist/100.0f).y));
        auto steer_single_point = ctrl.computeSteering(traj.getPointOnTrajAt(ctrl_dist/100.0f).y);
        //auto steer_single_point = ctrl.computeSteering(errs.back());

        auto steering_ctrl = veh.steeringAngleDegToSignal(steer_single_point);

        ROS_INFO("calculated steering ctrl: %.d \n", steering_ctrl);

        steering.data = static_cast<short>(steering_ctrl);

        //steeringCtrl.publish(steering);
        steering.data = static_cast<short>(steering_ctrl);

        trajectory_points.points.clear();
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

