#include <lane_detector/lane_detector.h>

int main(int argc, char** argv)
{

  ros::NodeHandle nh;
  double looptime = .2;


  // Loop starts here:
  ros::Rate loop_rate(1/looptime);
  while (ros::ok())
  {

    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();

  }
  ros::spin();
}