#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <echtzeitsysteme/points.h>
#include <opencv2/opencv.hpp>
#include <lane_detection/CameraReader.hpp>
#include <lane_detection/image_processor.hpp>
#include <stdio.h>
#include <echtzeitsysteme/ImageProcessingConfig.h>

using namespace cv;

#define SHOW_IMAGES

//#define TEST_PICTURE_PATH "camera_reading_test/images/calibration_test_2.jpg"
//#define TEST_PICTURE_PATH "camera_reading_test/images/track_straight.jpg"
#define TEST_PICTURE_PATH "echtzeitsysteme/images/my_photo-2.jpg"


//#define USE_TEST_PICTURE
#define LOOP_RATE_IN_HERTZ 10
//#define DRAW_GRID

#define PARAMS_1 59.0,84.0,30.0,640,480,Point(0,366),Point(632,363),Point(404,238),Point(237,237),Point(151,639),Point(488,639),Point(488,0),Point(151,0)
#define PARAMS_2 59.0,84.0,20,640,991,Point(43,387),Point(583,383),Point(404,189),Point(234,190),Point(95,990),Point(545,990),Point(545,350),Point(95,350)

// my_photo-2.jpg
#define PARAMS_3 59.0,84.0,20,180,180,Point(549,799),Point(1384,786),Point(1129,490),Point(800,493),5

const Point2i POINT_1 = Point2i(320,0);
const Point2i POINT_2 = Point2i(320,990);
const Point2i POINT_3 = Point2i(20,460);

/* configuration parameters */
int low_H, low_S, low_V, high_H, high_S, high_V;
double y_dist_cm, lane_dist_cm;
int loop_rate;

Mat processImage(Mat input, ImageProcessor& proc);

void configCallback(echtzeitsysteme::ImageProcessingConfig &config, uint32_t level) {
  low_H = config.low_H;
  low_S = config.low_S;
  low_V = config.low_V;
  high_H = config.high_V;
  high_S = config.high_S;
  high_V = config.high_V;
  y_dist_cm = config.y_dist_cm;
  lane_dist_cm = config.lane_dist_cm;
  loop_rate = config.loop_rate;

  ROS_INFO("Updated configuration.");
}


/**
 * Here comes the real magic
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
  if (frame.empty())
  {
    ROS_ERROR("Test image could not be opened!");
  }
#endif
  ROS_INFO("LANE DETECTION NODE");
  char dir_name[100];
  getcwd(dir_name, 100);
  ROS_INFO("Current directory is: %s", dir_name);

#ifndef USE_TEST_PICTURE
  CameraReader reader;
  ROS_INFO("FPS: %f", reader.getVideoCapture().get(CV_CAP_PROP_FPS));
  //ROS_INFO("Buffer size: %f", reader.getVideoCapture().get(CV_CAP_PROP_BUFFERSIZE));
#endif
  
  // TODO: for more meaningful testing, move object creation in the loop
  ImageProcessor imageProcessor(frame, BGR);
  //imageProcessor.calibrateCameraImage(PARAMS_2);
  imageProcessor.calibrateCameraImage(PARAMS_3);

  imshow("CameraFrame", frame);

/*
  frame = imageProcessor.resize(800,450);
  imshow("resized", frame);
  waitKey(0);


  Mat copy = frame.clone(); // for further imshow and sign detection

  frame = imageProcessor.regionOfInterest(0,0,600,400);
  imshow("region of interest", frame);
  waitKey(0);
*/

  /**
   * Init ROS Publisher here. Can set to be a fixed array
   */
  echtzeitsysteme::points trajectory;
  trajectory.points.clear();
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

  ros::Rate loop_rate(loop_rate); //TODO: Hz anpassen

  while (ros::ok())
  {
    //reader.readImage();
    //ROS_INFO("Number of frames: %f", reader.getNumberOfFrames());
    
    ROS_INFO("Show frame.");
#ifndef USE_TEST_PICTURE
    frame = reader.readImage();
#endif
#ifdef DRAW_GRID
    drawGrid(frame);
#endif
    Mat processedImage = processImage(frame, imageProcessor);
    Point2i trajPoint = imageProcessor.singleTrajPoint(lane_dist_cm, y_dist_cm);

    ROS_INFO("Calculated traj point.");
#ifdef SHOW_IMAGES
    imshow("traj point", imageProcessor.drawPoint(trajPoint));
#endif

    // clear points array every loop
    trajectory.points.clear();

    /**
     * create geometry_msgs/Point message for every entry in custom points msg and push it
     */
    geometry_msgs::Point point1;
    point1.x = trajPoint.x;
    point1.y = trajPoint.y;

    trajectory.points.push_back(point1);

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

Mat processImage(Mat input, ImageProcessor& proc) {
  Mat output;
  proc.setImage(input, BGR);
  output = proc.transformTo2D();
#ifdef SHOW_IMAGES
  imshow("2D input", output);
#endif
  ROS_INFO("H_low: %d, S_low: %d, V_low: %d, H_high: %d, S_high: %d, V_high: %d", low_H, low_S, low_V, high_H, high_S, high_V);
  proc.convertToHSV();
  output = proc.filterColor(Scalar(low_H, low_S, low_V),
                                            Scalar(high_H, high_S, high_V));
#ifdef SHOW_IMAGES
  imshow("green filtered", output);
#endif

  output = proc.removeNoise(5,5);
#ifdef SHOW_IMAGES
  imshow("green with noise removed", output);
#endif

/*
  // noise removal
  output = proc.edgeDetection(sel.getLowCannyThresh(), sel.getHighCannyThresh());
  imshow("edges detected", output);
*/
  Point2i trajPoint = proc.singleTrajPoint(40, 100);
  imshow("traj point", proc.drawPoint(trajPoint));

  return output;
}