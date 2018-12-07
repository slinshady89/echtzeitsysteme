#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <echtzeitsysteme/points.h>
#include <opencv2/opencv.hpp>
#include <CameraReader.hpp>
#include <image_processor.hpp>
#include <stdio.h>

using namespace cv;

//#define TEST_PICTURE_PATH "camera_reading_test/images/calibration_test_2.jpg"
//#define TEST_PICTURE_PATH "camera_reading_test/images/track_straight.jpg"
#define TEST_PICTURE_PATH "camera_reading_test/images/track_calibration_1.jpg"


#define USE_TEST_PICTURE
#define LOOP_RATE_IN_HERTZ 2
//#define DRAW_GRID

#define PARAMS_1 59.0,84.0,30.0,640,480,Point(0,366),Point(632,363),Point(404,238),Point(237,237),Point(151,639),Point(488,639),Point(488,0),Point(151,0)
#define PARAMS_2 59.0,84.0,20,640,991,Point(43,387),Point(583,383),Point(404,189),Point(234,190),Point(95,990),Point(545,990),Point(545,350),Point(95,350)


const Point2i POINT_1 = Point2i(320,0);
const Point2i POINT_2 = Point2i(320,990);
const Point2i POINT_3 = Point2i(20,460);


// for calibration / centering the car
void drawGrid(Mat& mat)
{
  int width = mat.cols;
  int height = mat.rows;
  line(mat, Point(0, (height-1)/2), Point(width-1, (height-1)/2), Scalar(0,0,255), 1);
  line(mat, Point((width-1)/2, 0), Point((width-1)/2, height-1), Scalar(0,0,255), 1);
}

// for debugging
void printWorldCoords(Point2i pxPoint, int pointId, ImageProcessor& proc)
{
  Point2d worldCoords1 = proc.getWorldCoordinates(pxPoint);
  ROS_INFO("Car coordinates of image point %d (%d,%d): (%f,%f)", pointId, pxPoint.x, pxPoint.y, worldCoords1.x, worldCoords1.y);
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
  Mat frame;

#ifdef USE_TEST_PICTURE
  frame = imread(TEST_PICTURE_PATH, IMREAD_COLOR);
  if (frame.empty()) {
    ROS_ERROR("Test image could not be opened!");
  }
#endif
  ROS_INFO("VIDEO READING TEST");
  char dir_name[100];
  getcwd(dir_name, 100);
  ROS_INFO("Current directory is: %s", dir_name);
#ifndef USE_TEST_PICTURE
  CameraReader reader;

  ROS_INFO("FPS: %f", reader.getVideoCapture().get(CV_CAP_PROP_FPS));
  //ROS_INFO("Buffer size: %f", reader.getVideoCapture().get(CV_CAP_PROP_BUFFERSIZE));
#endif
  
  // TODO: for more meaningful testing, move object creation in the loop
  ImageProcessor imageProcessor(frame);
  imageProcessor.calibrateCameraImage(PARAMS_2);

  imshow("CameraFrame", frame);
  waitKey(0);

  frame = imageProcessor.transformTo2D();
  
  ROS_INFO("Open up window...");
  //namedWindow("CameraFrame", WINDOW_AUTOSIZE);


  /**
   * Init ROS Publisher here. Can set to be a fixed array
   */
  echtzeitsysteme::points trajectory;
  trajectory.data.clear();
  float counter = 0.0;

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

  ros::Rate loop_rate(LOOP_RATE_IN_HERTZ); //TODO: Hz anpassen

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

    trajectory.points.push_back(point1);  

    ROS_INFO("lane_detection runs");

    //reader.readImage();
    //ROS_INFO("Number of frames: %f", reader.getNumberOfFrames());
    
    ROS_INFO("Show frame.");
#ifndef USE_TEST_PICTURE
    frame = reader.readImage();
#endif
#ifdef DRAW_GRID
    drawGrid(frame);
#endif

    printWorldCoords(POINT_1, 1, imageProcessor);
    imageProcessor.drawPoint(POINT_1);
    printWorldCoords(POINT_2, 2, imageProcessor);
    imageProcessor.drawPoint(POINT_2);
    printWorldCoords(POINT_3, 3, imageProcessor);
    frame = imageProcessor.drawPoint(POINT_3);


    imshow("CameraFrame", frame);
    waitKey(1); // set to 0 for manual continuation (key-press) or specify auto-delay in milliseconds
    ROS_INFO("Showed frame.");

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    trajectory_pub.publish(trajectory);

    ++counter;

    // clear input/output buffers
    ros::spinOnce();

    // this is needed to ensure a const. loop rate
    loop_rate.sleep();
  }


  return 0;
}