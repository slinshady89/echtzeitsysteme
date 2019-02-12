#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Point.h>
#include <echtzeitsysteme/points.h>
#include <trajectory_planning/trajectory.h>

int vel = 0;
int steer = 0;
struct point
{
  double x;
  double y;
} trajectory[10]; // size can be changed

/*
 * callback function fpr trajectory custom points message
 * iterate over each Point in msg and save it in trajectory
 */
void trajectoryCallback(const echtzeitsysteme::points::ConstPtr &msg)
{
  trajectory[0].x = (double)msg->points[0].x;
  trajectory[0].y = (double)msg->points[0].y;
  ROS_INFO("trajectory planning is hearing %f %f ", trajectory[0].x, trajectory[0].y);
}

void uslCallback(sensor_msgs::Range::ConstPtr uslMsg, sensor_msgs::Range *usl)
{
  *usl = *uslMsg;
}

void usfCallback(sensor_msgs::Range::ConstPtr usfMsg, sensor_msgs::Range *usf)
{
  *usf = *usfMsg;
}

void usrCallback(sensor_msgs::Range::ConstPtr usrMsg, sensor_msgs::Range *usr)
{
  *usr = *usrMsg;
}

void ctrlParamCallback(echtzeitsysteme::ControllerConfig &config, CController *ctrl)
{
  ROS_INFO("Reconfigure Request: %f %f %f %d %d %d",
           config.K_P,
           config.K_I,
           config.K_D,
           config.vel,
           config.steering,
           config.size);
  ctrl->setCtrlParams(config.K_P, config.K_I, config.K_D);
  vel = config.vel;
  steer = config.steering;
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

  ros::Subscriber sub = nh.subscribe("trajectory", 1, trajectoryCallback);

  // generate subscriber for us-sensor messages
  ros::Subscriber usrSub = nh.subscribe<sensor_msgs::Range>("/uc_bridge/usr", 5, boost::bind(usrCallback, _1, &usr));
  ros::Subscriber uslSub = nh.subscribe<sensor_msgs::Range>("/uc_bridge/usl", 5, boost::bind(uslCallback, _1, &usl));
  ros::Subscriber usfSub = nh.subscribe<sensor_msgs::Range>("/uc_bridge/usf", 5, boost::bind(usfCallback, _1, &usf));

  CController ctrl(17.5, 3.5, 0.1, 0.1, 1000);

  // Loop starts here:
  ros::Rate loop_rate(1 / looptime);

  f = boost::bind(&ctrlParamCallback, _1, &ctrl);
  server.setCallback(f);

  while (ros::ok())
  {

    // some validation check should be done!
    /*
    if(!ctrl.ctrlLoop(usl, usr, usf))    {
      ROS_INFO("Ultrasonic sensor is detecting something closer than: %f", ctrl.getUsMinDist());
      velocity.data = 0;
    }else*/
    {
      //ctrl.setCtrlParams(serverK_P, serverK_I, serverK_D, 0.1, 1000);
      //velocity.data = 500;
      //ROS_INFO("error: %.4f", 0.2*sin(2*M_PI/20.0*i));
      //double err = (0.2*sin(2*M_PI/20.0*i++));
      //double err2 = 0.07*exp(1.0 - 1/20.0*(i - 10)*(i - 10));
      //i++;
      //ROS_INFO("error: %.4f", err2);
      steering.data = (int16_t)-ctrl.computeSteering(trajectory[0].x); //std::array<double, arraySize>{{0.0,.0,.0,.2,.1}} );
      //ROS_INFO("calculated Steering: %.4f\n", (float) steering.data);
    }

    ROS_INFO("calculated Steering: %.2f\n", (float)steering.data);
    velocity.data = vel;
    ROS_INFO("vel: %d\n", velocity.data);
    motorCtrl.publish(velocity);

    steeringCtrl.publish(steering);
    ros::spinOnce();

    loop_rate.sleep();
  }

  ros::spin();

  return 0;
}
