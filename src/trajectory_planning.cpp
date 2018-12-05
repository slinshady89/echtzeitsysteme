#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Point.h"
#include "echtzeitsysteme/points.h"
#include "trajectory_planning/trajectory.h"
#include "y_interp.h"//...................................yInterp,<cmath>{sin()}
#include <cstdio>//.....................................................printf()



struct point
{
  float x;
  float y;
} trajectory[100];

/*
 * callback function fpr trajectory custom points message
 * iterate over each Point in msg and save it in trajectory
 */
void trajectoryCallback(const echtzeitsysteme::points::ConstPtr& msg)
{
  trajectory[0].x = msg->points[0].x;
  trajectory[0].y = msg->points[0].y;
  trajectory[1].x = msg->points[1].x;
  trajectory[1].y = msg->points[1].y;
  ROS_INFO("trajectory planning is hearing %f %f %f %f", trajectory[0].x, trajectory[0].y, trajectory[1].x, trajectory[1].y);
}





/**
 * Here comes the tracejtory planning magic
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "LKA");
  std::vector<double> sums, errors;
  double looptime = .2;

  std_msgs::Int16 velocity, steering;
  sensor_msgs::Range usr, usf, usl;



  steering.data = 0;


  // tests for trajectory

  //double X[20];/*<-*/for(int i=0;i<20;++i)X[i]=5*i/20.+5;
  //double Y[20];/*<-*/for(int i=0;i<20;++i)Y[i]=sin(2*3.14159*X[i]/5)+1;
  //double x=7.18;
  //int i=yInterp::BinarySearch(X+1,X+19,x)-X;
  //double y=yInterp::CubeInterp(X+i,Y+i,x,0.,0.);
  //printf("At x=%.3f, y is approximately %.3f.\n",x,y);





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

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;
  ros::Publisher motorCtrl =
	  nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
	ros::Publisher steeringCtrl =
	  nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);

    // generate subscriber for sensor messages
  ros::Subscriber usrSub = nh.subscribe<sensor_msgs::Range>(
      "/uc_bridge/usr", 10, boost::bind(CSafety::usrCallback, _1, &usr));
  ros::Subscriber uslSub = nh.subscribe<sensor_msgs::Range>(
      "/uc_bridge/usl", 10, boost::bind(CSafety::uslCallback, _1, &usl));
  ros::Subscriber usfSub = nh.subscribe<sensor_msgs::Range>(
      "/uc_bridge/usf", 10, boost::bind(CSafety::usfCallback, _1, &usf));




	ROS_INFO("trajectory_planning");
  
  CController ctrl(2.0,0.0,0.0,100,0.2);

  // Loop starts here:
  ros::Rate loop_rate(1/looptime);
  while (ros::ok())
  {
    // some validation check should be done!
    if(!ctrl.ctrlLoop(usl, usr, usf))    {
      ROS_INFO("Ultrasonic sensor is detecting something closer than: %s", ctrl.getUsMinDist());
      velocity.data = 0;
    }else{
      ctrl.setCtrlParams(2.0,0.0,0.0,100,0.0);
      velocity.data = 500;
      steering.data = (int16_t) ctrl.computeSteering( std::array<double, arraySize>{{0.0,.0,.0,.2,.1}} );      
    }
    motorCtrl.publish(velocity);
    steeringCtrl.publish(steering);
    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();
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

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}