#ifndef CAMERA_READER_HPP_
#define CAMERA_READER_HPP_

// default camera resolution
#define INIT_VIDEO_WIDTH 1920
#define INIT_VIDEO_HEIGHT 1080

// read a file "test.mp4" instead of the webcam input if active
//#define DEBUG

#include <opencv2/opencv.hpp>
using namespace cv;


/**
 * Convenience class for opening an OpenCV video stream from a camera / webcam.
 */
class CameraReader {
    public:

        CameraReader() : CameraReader(INIT_VIDEO_WIDTH, INIT_VIDEO_HEIGHT) {};
        /**
         * Initializes an OpenCV VideoCapture object with the available (web)cam
         * and also sets it to the desired frame resolution.
         * @param width the resolution width
         * @param height the resolution height
         */
        CameraReader(int width, int height);
        ~CameraReader(){};


        Mat readImage();
        /**
         * Sets the frame to be returned to the specified position (0 for beginning, 1 end of the file / buffer).
         * Set to 1 for most recent webcam picture.
         */
        void setRelativePositionInVideo(double position);
        double getNumberOfFrames();
        void setCameraSettings(double brightness, double contrast, double saturation, double hue);

        VideoCapture& getVideoCapture();    // TODO const reference (read-only)

    private:
        VideoCapture cap;
        Mat image;
};



#endif // CAMERA_READER_HPP_