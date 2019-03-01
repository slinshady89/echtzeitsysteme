#ifndef PROJECT_CAMERACALIBRATION_H
#define PROJECT_CAMERACALIBRATION_H

#include <opencv2/opencv.hpp>

using namespace cv;

class CameraCalibration {
public:
    int getDstWidth() const;

    int getDstHeight() const;

    double getHeight_px_per_cm() const;

    double getHeight_cm_per_px() const;

    double getWidth_px_per_cm() const;

    double getWidth_cm_per_px() const;

    int getPx_os_bottom() const;

private:
    Mat transformMatr;
    Mat invTransformMatr;
    Point srcP1;
    Point srcP2;
    Point srcP3;
    Point srcP4;
    Point dstP1;
    Point dstP2;
    Point dstP3;
    Point dstP4;
    Point2f srcPoints[4];
    Point2f dstPoints[4];
    int dstWidth;
    int dstHeight;

    // variables for calculating coordinates
    double height_px_per_cm;
    double height_cm_per_px;
    double width_px_per_cm;
    double width_cm_per_px;

    // pixel offsets to the test plane edges in the 2D-transformed image
    int px_os_bottom;
    int px_os_top;
    int px_os_left;
    int px_os_right;
    int offset_cm;
public:
    int getOffset_cm() const;

public:
        CameraCalibration(double testRectWidth_cm, double testRectHeight_cm, double offsetToOrigin_cm,
                                int targetWidth_cm, int targetHeight_cm,
                                Point srcP1_px, Point srcP2_px, Point srcP3_px, Point srcP4_px,
                                double px_per_cm)
        {
            calibrateCameraImage(testRectWidth_cm, testRectHeight_cm, offsetToOrigin_cm,
                    targetWidth_cm, targetHeight_cm,
                    srcP1_px, srcP2_px, srcP3_px, srcP4_px,
                    px_per_cm);
        };

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

    /**
         * determines the size of the target image automatically, placing the test rectangle at the bottom of the image
         */
    void calibrateCameraImage(double testRectWidth_cm, double testRectHeight_cm, double offsetToOrigin_cm,
                                int targetWidth_cm, int targetHeight_cm,
                                Point srcP1_px, Point srcP2_px, Point srcP3_px, Point srcP4_px,
                                double px_per_cm);

    Point2d getWorldCoordinates(Point2i imageCoordinates);

    Point2i getImageCoordinates(Point2d worldCoordinates);


    Mat getTransformMatr();
};

#endif //PROJECT_CAMERACALIBRATION_H
