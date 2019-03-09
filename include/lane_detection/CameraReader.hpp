#ifndef CAMERA_READER_HPP_
#define CAMERA_READER_HPP_

// default camera resolution
#define INIT_VIDEO_WIDTH 1280
#define INIT_VIDEO_HEIGHT 720

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


        /**
         * Reads the most recent frame in the buffer of the contained video capture.
         * @return most recent frame
         */
        Mat readImage();

        void setCameraSettings(double brightness, double contrast, double saturation, double hue);

        VideoCapture& getVideoCapture();

    private:
        VideoCapture cap;
        Mat image;
};



#endif // CAMERA_READER_HPP_