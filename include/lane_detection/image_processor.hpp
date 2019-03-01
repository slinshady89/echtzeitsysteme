#ifndef IMAGE_PROCESSOR_HPP_
#define IMAGE_PROCESSOR_HPP_

#include <opencv2/opencv.hpp>
#include <chrono>
#include <ros/ros.h>
#include "utils/time_profiling.hpp"
#include "CameraCalibration.hpp"

using namespace cv;

enum ColorType { BGR, HSV, GREY };

/**
 * Image processor to perform multiple operation on OpenCV Mat objects.
 * NOTE: calibrateCameraImage() must have been called in order to use most methods!
 */
class ImageProcessor {
    public:
        ImageProcessor(Mat img, CameraCalibration cal) : image(img), calibration(cal) { colorType = BGR; };
        ImageProcessor(Mat img, ColorType type, CameraCalibration cal) : image(img), colorType(type), calibration(cal) {};
        ~ImageProcessor(){};

    Mat transformTo2D();
        Mat convertToHSV();

        Mat filterColor(Scalar lowHSVColor, Scalar highHSVColor);
        Mat edgeDetection(int lowThresh, int highThresh);

        Mat removeNoise(int kwidth, int kheight);
        Mat resize(int width, int height);
        Mat regionOfInterest(int x, int y, int width, int height);
        //Mat convertToGrayscale();


        Mat& getImage();    // TODO: remove? getter not necessary?
        void setImage(Mat img, ColorType type); // TODO: remove this method later when testing is over!

        /**
         * Calculates a single (inaccurate) trajectory point by using a fixed distance to the right lane in the given distance (y_cm).
         */
        // TODO: extract this method to own class - ImageProcessor should not be responsible for calculating trajectory or lane points!
        Point2d singleTrajPoint(double rightLaneDist_cm, double y_cm, int colorThreshold);


        // Methods for finding the position of white pixels in a grayscale / black-white image

        Point2i firstMatchFromRight(int pxY);
        Point2i firstMatchFromLeft(int pxY);
        /* searches for the next white pixel around the x-position of the previous match */
        Point2i nextMatch(int pxY, Point2i lastMatch);

        /* search for the last pixel in left / right direction from start that is still white */
        Point2i lastMatchLeft(Point2i start);
        Point2i lastMatchRight(Point2i start);

        // debugging methods
        Mat drawPoint(Point2i point);
        Mat drawPoint(Point2i point, Scalar color);

    private:
        Mat image;
        CameraCalibration calibration;
    ColorType colorType;
        bool calibrated = false;


};



#endif // IMAGE_PROCESSOR_HPP_