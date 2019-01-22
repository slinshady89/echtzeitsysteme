#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Point.h>
#include <echtzeitsysteme/points.h>
#include <trajectory_planning/trajectory.h>


int vel = 0;
struct point
{
  double x;
  double y;
} trajectory[10]; // size can be changed

struct trajLeft{
  alglib::real_1d_array x;
  alglib::real_1d_array y;
};


/*
 * callback function fpr trajectory custom points message
 * iterate over each Point in msg and save it in trajectory
 */
void trajectoryCallback(const echtzeitsysteme::points::ConstPtr& msg)
{
  trajectory[0].x = (double) msg->points[0].x;
  trajectory[0].y = (double) msg->points[0].y;
  ROS_INFO("trajectory planning is hearing %f %f ", trajectory[0].x, trajectory[0].y);
}


void uslCallback(sensor_msgs::Range::ConstPtr uslMsg, sensor_msgs::Range* usl)
{
  *usl = *uslMsg;
}

void usfCallback(sensor_msgs::Range::ConstPtr usfMsg, sensor_msgs::Range* usf)
{
  *usf = *usfMsg;
}

void usrCallback(sensor_msgs::Range::ConstPtr usrMsg, sensor_msgs::Range* usr)
{
  *usr = *usrMsg;
}


void ctrlParamCallback(echtzeitsysteme::ControllerConfig &config, CController *ctrl) {
  ROS_INFO("Reconfigure Request: %f %f %f %d %d",
            config.K_P, config.K_I,
            config.K_D,
            config.vel,
            config.size);
  ctrl->setCtrlParams(config.K_P, config.K_I, config.K_D);
  vel = config.vel;
}


/**
 * Here comes the tracejtory planning magic
 */
int main(int argc, char **argv)
{


  size_t i = 0;
  std::vector<double> sums, errors;
  double looptime = .2;

  std_msgs::Int16 velocity, steering;
  sensor_msgs::Range usr, usf, usl;

  velocity.data = 0;
  steering.data = 0;

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "trajectory_planning");



  dynamic_reconfigure::Server<echtzeitsysteme::ControllerConfig> server;
  dynamic_reconfigure::Server<echtzeitsysteme::ControllerConfig>::CallbackType f;


  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;
  ros::Publisher motorCtrl = nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
  ros::Publisher steeringCtrl = nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);

  double v_mops = 0;
  double serverK_P(0.0);  // range: 0...100
  double serverK_I(0.0);  // range: 0...50
  double serverK_D(0.0);  // range: 0...100
  double server_dt (0.0);
  double steeringLimitAbs(1000);  // range: 0...1000

  if(nh.getParam("v_mops", v_mops))
  {
    ROS_INFO("v_mops = %f", v_mops);
  }
   if(nh.getParam("K_P", serverK_P))
  {
    ROS_INFO("K_P = %f", serverK_P);
  }
 if(nh.getParam("K_I", serverK_I))
  {
    ROS_INFO("K_I = %f", serverK_I);
  }
 if(nh.getParam("K_D", serverK_D))
  {
    ROS_INFO("K_D = %f", serverK_D);
  }
 if(nh.getParam("dt", server_dt))
  {
    ROS_INFO("dt = %f", server_dt);
  }
 if(nh.getParam("steeringLimitAbs", steeringLimitAbs))
  {
    ROS_INFO("steeringLimitAbs = %f", steeringLimitAbs);
  }



  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = nh.subscribe("trajectory", 1, trajectoryCallback);

  // generate subscriber for us-sensor messages
  ros::Subscriber usrSub = nh.subscribe<sensor_msgs::Range>("/uc_bridge/usr", 5,  boost::bind( usrCallback, _1, &usr ));
  ros::Subscriber uslSub = nh.subscribe<sensor_msgs::Range>("/uc_bridge/usl", 5,  boost::bind( uslCallback, _1, &usl ));
  ros::Subscriber usfSub = nh.subscribe<sensor_msgs::Range>("/uc_bridge/usf", 5,  boost::bind( usfCallback, _1, &usf ));

  CController ctrl(17.5, 3.5, 0.1, 0.1, 1000);

  // Loop starts here:
  ros::Rate loop_rate(1/looptime);

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
      steering.data = (int16_t) -ctrl.computeSteering( trajectory[0].y );//std::array<double, arraySize>{{0.0,.0,.0,.2,.1}} );
      //ROS_INFO("calculated Steering: %.4f\n", (float) steering.data);
    }
  ROS_INFO("calculated Steering: %.2f\n", (float) steering.data);
velocity.data = vel;
  ROS_INFO("vel: %d\n",  velocity.data);
    motorCtrl.publish(velocity);
    steeringCtrl.publish(steering);
    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();
  }

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
