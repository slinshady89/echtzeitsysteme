#ifndef IMAGE_PROCESSOR_HPP_
#define IMAGE_PROCESSOR_HPP_

#include <opencv2/opencv.hpp>

using namespace cv;

/**
 * Image processor to perform multiple operation on OpenCV Mat objects.
 * NOTE: calibrateCameraImage() must have been called in order to use most methods!
 */
class ImageProcessor {
    public:
        ImageProcessor(Mat img) : image(img) {};
        ~ImageProcessor(){};
        
        /**
         * Calibrate the camera image according to the world coordinates of the calibration rectangle.
         * World coordinates are in centimeters, origin is the origin of the local coordinate system of the car.
         * offsetToOrigin_cm is the distance of the lower edge of the test rectangle to the car's origin (world coordinates).
         * The points specify the pixel coordinates (beware: y-down coordinates) of the test rectangle's corners in the source image
         * and in the destination image after applying a perspective transformation to a 2D plane.
         * Order of the points: bottom-left, bottom-right, top-right, top-left
         */
        void calibrateCameraImage(double testRectWidth_cm, double testRectHeight_cm, double offsetToOrigin_cm,
                                    int targetWidth, int targetHeight,
                                    Point srcP1_px, Point srcP2_px, Point srcP3_px, Point srcP4_px,
                                    Point dstP1_px, Point dstP2_px, Point dstP3_px, Point dstP4_px);

        Point2d getWorldCoordinates(Point2i imageCoordinates);
        Point2i getImageCoordinates(Point2d worldCoordinates);

        Mat transformTo2D();
        Mat filterColor();

        Mat& getImage();    // TODO: remove? getter not necessary?

        // debugging methods
        Mat drawPoint(Point2i point);

    private:
        Mat image;
        bool calibrated = false;

        Point srcP1, srcP2, srcP3, srcP4, dstP1, dstP2, dstP3, dstP4;
        Point2f srcPoints[4];
        Point2f dstPoints[4];
        int dstWidth, dstHeight;

        // variables for calculating coordinates
        double height_px_per_cm, height_cm_per_px, width_px_per_cm, width_cm_per_px;
        // pixel offsets to the test plane edges in the 2D-transformed image
        int px_os_bottom, px_os_top, px_os_left, px_os_right;

        int offset_cm;
        

};



#endif // IMAGE_PROCESSOR_HPP_