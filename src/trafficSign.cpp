// Language: Denglisch     C++

//include Files
#include "ros/ros.h"
#include "opencv2/opencv.hpp"   // noch hinzufügen


//namespace
using namespace cv;

/**
 *@brief main runs the main algorithm of the traffic sign recognition.
 *@brief It reads images from the Webcam
 *@brief The System learns and used to classify new signs features found by the robot.
 *@brief The result of the detection (type of sign, size) are published using a custom message.
 *@param argc is the number of arguments.
 *@param argv is the arguments characters array.
 *@return 0
 */
int main(int argc, char **argv) {

// Node zum pushen erstellen
    ros::init(argc, argv, "classification");
    ros::NodeHandle n;



//algorithm for traffic sign recorgnition
 //   classifier visual;  // von Microsoft





// Image Input from Webcam
    VideoCapture cap;

    // opens default camera else tipe different number than 0
    if(!cap.open(0))
        // Cancel if camera can not be accessed
        return 0;

    // endless Schleife with "ESC" brake    
    for(;;){
          Mat frame;
          // captured Image will be sent to the frame with is opened in a new Window
          cap >> frame;
          if( frame.empty() ) 
            // end of video stream
            break; 
          imshow("Camera Input Picture !#!", frame);
          if( waitKey(10) == 27 ){
            // stop capturing by pressing the emergency Button :) "ESC"
            cap.close();  // ????? maybe this work   ????
            break;
          }
    }
    // the camera will be closed automatically upon exit
    // cap.close();
    



// Custom Message Publisher (Result with sign, size)
    ros::Publisher signPublishing = n.advertise<//



}





// Source: 
// https://www.roboternetz.de/community/threads/70315-C-C-Beispielprogramme-mit-USB-Cam-und-openCV
// https://github.com/MichiMaestre/Traffic-Sign-Recognition-for-Autonomous-Cars/blob/master/src/vision_node.cpp
