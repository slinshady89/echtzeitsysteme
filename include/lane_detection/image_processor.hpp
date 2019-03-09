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
 */
class ImageProcessor {
    public:
        ImageProcessor(Mat img, CameraCalibration cal) : image(img), calibration(cal) { colorType = BGR; };
        ImageProcessor(Mat img, ColorType type, CameraCalibration cal) : image(img), colorType(type), calibration(cal) {};
        ~ImageProcessor(){};

        /**
         * Perspective transform the contained image, using the (formerly computed) transformation matrix.
         * @return a copy of the image after applying the transformation
         */
        Mat transformTo2D();
        /**
         * Converts the contained image to the HSV color space.
         * (Especially useful for comparing color thresholds which is easier in HSV then in BGR.)
         * @return a copy of the image converted to HSV colors
         */
        Mat convertToHSV();

        /**
         * Creates a binary image with only those pixel being white whose colors in the original image are in the
         * range of the two given thresholds.
         * Note: EVERY single color channel (i.e. H / S / V) must match to the range of the
         * corresponding H / S / V threshold values in order to produce a white pixel.
         * @param lowHSVColor the low HSV threshold
         * @param highHSVColor the high HSV threshold
         * @return a copy of the produced binary image (which only contains a single channel)
         */
        Mat filterColor(Scalar lowHSVColor, Scalar highHSVColor);

        Mat edgeDetection(int lowThresh, int highThresh);

        /**
         * Removes noise in the contained image by applying gaussian blur with the specified kernel width and height.
         * @param kwidth kernel width
         * @param kheight kernel height
         * @return copy of the denoised image
         */
        Mat removeNoise(int kwidth, int kheight);
        /**
         * Resizes the contained image.
         * @param width new width
         * @param height new height
         * @return copy of the resized image
         */
        Mat resize(int width, int height);
        /**
         * Cuts out a section of the contained image.
         * @param x x value of the top-left corner of the rectangle
         * @param y y value of the top-left corner of the rectangle
         * @param width width of the area to cut out
         * @param height height of the area to cut out
         * @return copy of the image which only contains the section described by the given parameters
         */
        Mat regionOfInterest(int x, int y, int width, int height);

        Mat& getImage();
        // resets the contained image to the given one, with the given color type
        void setImage(Mat img, ColorType type);


        // Methods for finding the position of white pixels in a grayscale / black-white image

        Point2i firstMatchFromRight(int pxY);
        Point2i firstMatchFromLeft(int pxY);
        /**
         * Searches for the next white pixel at a given y position with the x position of the previous
         * match being the starting point.
         * @param pxY row
         * @param lastMatch last match
         * @return white pixel which is closest to the x-value of the previous point
         */
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