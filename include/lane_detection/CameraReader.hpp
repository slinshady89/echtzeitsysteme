#ifndef CAMERA_READER_HPP_
#define CAMERA_READER_HPP_

#define INIT_VIDEO_WIDTH 1920
#define INIT_VIDEO_HEIGHT 1080

// read a file "test.mp4" instead of the webcam input if active
//#define DEBUG

#include <opencv2/opencv.hpp>
using namespace cv;


class CameraReader {
    public:

        CameraReader();
        ~CameraReader(){};


        Mat readImage();
        /**
         * Sets the frame to be returned to the specified position (0 for beginning, 1 end of the file / buffer).
         * Set to 1 for most recent webcam picture.
         */
        void setRelativePositionInVideo(double position);
        double getNumberOfFrames();

        VideoCapture& getVideoCapture();    // TODO const reference (read-only)

    private:
        VideoCapture cap;
        Mat image;
};



#endif // CAMERA_READER_HPP_