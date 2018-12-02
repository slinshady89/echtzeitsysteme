#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "echtzeitsysteme/points.h"

/**
 * Here comes the cv stuff
 */
int main(int argc, char **argv)
{
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
  ros::init(argc, argv, "lane_detection");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;

  /**
   * Init ROS Publisher here. Can set to be a fixed array
   */
  echtzeitsysteme::points trajectory;
  float counter = 0;
  //trajectory.data.clear();

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher trajectory_pub = nh.advertise<echtzeitsysteme::points>("trajectory", 1);  //TODO: change buffer size

  ros::Rate loop_rate(10); //TODO: Hz anpassen

  while (ros::ok())
  {
    // clear points array every loop
    trajectory.points.clear();

    /**
     * create geometry_msgs/Point message for every entry in custom points msg and push it
     */
    geometry_msgs::Point point1;
    point1.x = counter;
    point1.y = counter + 1;
    
    geometry_msgs::Point point2;
    point2.x = counter + 2;
    point2.y = counter + 3;

    trajectory.points.push_back(point1);
    trajectory.points.push_back(point2);   

    ROS_INFO("lane_detection runs");

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    trajectory_pub.publish(trajectory);

    ++counter;

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}