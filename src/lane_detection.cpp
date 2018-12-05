#include "geometry_msgs/Point.h"
#include "echtzeitsysteme/points.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <math.h>
#include <iomanip>

using namespace cv;
using namespace std;

int lane() {
  // value can either be 1 or 0 depending on which camera you want to use --> if there is only one camera value should be 0
  int num = 0;
  VideoCapture cap = VideoCapture("LangsameRunde.mkv");
  Mat frame, resized, cut, ipm;

  if (!cap.isOpened()) {
    cerr << "Unable to collect video feed number: " << num;
    return -1;
  }

  while (true) {
    // skip frames
    int skip = 4;
    for (int i = 0; i < skip; i++) {
      cap.read(frame);
    }
    if (frame.empty())
      break;

    imshow("input", frame);

    // resize the image for limited resources
    resize(frame, resized, Size(800, 450), 0, 0, 5);
    imshow("resized", resized);

    //cut the image to only see the lane --> parameters need to be adjusted
    cut = resized.clone();
    cut = cut(Rect(0, 250, resized.cols, 250));
    imshow("cut", cut);
    
    //Transformation mat
    Mat transform(2, 4, CV_32FC1);

    /*the function warpPerspective() needs a 3x3 matrix to change the perspective of the image
    the function getPerspectiveTransform() creates this matrix with the four corner points of the source image and the desired image*/
    Point2f inputvalues[4];
    Point2f outputvalues[4];

    //source points
    inputvalues[0] = Point2f(0, 0);
    inputvalues[1] = Point2f(cut.cols, 0);
    inputvalues[2] = Point2f(cut.cols, cut.rows);
    inputvalues[3] = Point2f(0, cut.rows);

    //desired points
    outputvalues[0] = Point2f(0, 0);
    outputvalues[1] = Point2f(resized.cols, 0);
    outputvalues[2] = Point2f(resized.cols - 300, cut.rows);
    outputvalues[3] = Point2f(300, cut.rows);

    //transformation matrix is generated
    transform = getPerspectiveTransform(inputvalues, outputvalues);

    //transformation occurs and is stored on ipm
    warpPerspective(resized, ipm, transform, resized.size());
    imshow("IPM", ipm);
  }
  return -1;

}
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