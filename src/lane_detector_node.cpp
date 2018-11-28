#include <lane_detector/lane_detector.h>
#include <lane_detector/controller.h>

int main(int argc, char** argv)
{

  ros::init(argc, argv, "LKA");
  ros::NodeHandle nh;
  std::vector<double> sums, errors;
  double looptime = .2;

  std_msgs::Int16 velocity;
  std_msgs::Int16 steering;
  steering.data = 0;


  ros::Publisher motorCtrl =
	  nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
	ros::Publisher steeringCtrl =
	  nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);
	ROS_INFO("LKA");



  CController ctrl(2.0,0.0,0.0,100,0.2);

  // Loop starts here:
  ros::Rate loop_rate(1/looptime);
  while (ros::ok())
  {
    // some validation check should be done!
    if(!ctrl.ctrlLoop())    
          ROS_ERROR("error calculating steering angle");

    ctrl.setCtrlParams(2.0,0.0,0.0,100,0.0);

    velocity.data = 500;
    steering.data = (int16_t) ctrl.computeSteering({0.0,0.0,0.1,0.2,0.25});
    
    motorCtrl.publish(velocity);
    steeringCtrl.publish(steering);
    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();

  }
  ros::spin();
}