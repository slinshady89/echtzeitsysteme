#include <lane_detection/image_processor.hpp>
#include <CameraCalibration.hpp>
#include <lane_detection/CameraCalibration.hpp>
#include <opencv2/opencv.hpp>


void CameraCalibration::calibrateCameraImage(double testRectWidth_cm, double testRectHeight_cm, double offsetToOrigin_cm,
                                    int targetWidth, int targetHeight,
                                    Point srcP1_px, Point srcP2_px, Point srcP3_px, Point srcP4_px,
                                    Point dstP1_px, Point dstP2_px, Point dstP3_px, Point dstP4_px)
{
    offset_cm = offsetToOrigin_cm;

    int rect_height_px = dstP1_px.y-dstP4_px.y;   // y-down coordinates
    int rect_width_px = dstP2_px.x-dstP1_px.x;

    px_os_bottom = targetHeight-1 - dstP1_px.y;
    px_os_top = dstP4_px.y;
    px_os_left = dstP1_px.x;
    px_os_right = targetWidth-1 - dstP2_px.x;

    dstWidth = targetWidth;
    dstHeight = targetHeight;

    height_px_per_cm = rect_height_px/testRectHeight_cm;
    height_cm_per_px = testRectHeight_cm/rect_height_px;
    width_px_per_cm = rect_width_px/testRectWidth_cm;
    width_cm_per_px = testRectWidth_cm/rect_width_px;

    // TODO: does this implicit conversion to float points work?
    srcPoints[0] = Point2f(srcP1_px.x, srcP1_px.y);
    srcPoints[1] = Point2f(srcP2_px.x, srcP2_px.y);
    srcPoints[2] = Point2f(srcP3_px.x, srcP3_px.y);
    srcPoints[3] = Point2f(srcP4_px.x, srcP4_px.y);
    dstPoints[0] = Point2f(dstP1_px.x, dstP1_px.y);
    dstPoints[1] = Point2f(dstP2_px.x, dstP2_px.y);
    dstPoints[2] = Point2f(dstP3_px.x, dstP3_px.y);
    dstPoints[3] = Point2f(dstP4_px.x, dstP4_px.y);

    // TODO: are the points (as integer structures) needed at all?
    srcP1 = srcP1_px;
    srcP2 = srcP2_px;
    srcP3 = srcP3_px;
    srcP4 = srcP4_px;
    dstP1 = dstP1_px;
    dstP2 = dstP2_px;
    dstP3 = dstP3_px;
    dstP4 = dstP4_px;

    // compute transformation matrix for perspective warping + inverse matrix
    transformMatr = getPerspectiveTransform(srcPoints,dstPoints);
    invTransformMatr = transformMatr.inv();
}

void CameraCalibration::calibrateCameraImage(double testRectWidth_cm, double testRectHeight_cm, double offsetToOrigin_cm,
                                             int targetWidth_cm, int targetHeight_cm,
                                             Point srcP1_px, Point srcP2_px, Point srcP3_px, Point srcP4_px,
                                             double px_per_cm)
{
    int dstWidth_px = targetWidth_cm * px_per_cm;
    int dstHeight_px = targetHeight_cm * px_per_cm;

    int testRectWidth_px = testRectWidth_cm * px_per_cm;
    int testRectHeight_px = testRectHeight_cm * px_per_cm;

    // "place" test rectangle in the center (bottom) of the image
    Point dstP1_px = Point((dstWidth_px/2)-(testRectWidth_px/2), dstHeight_px-1);
    Point dstP2_px = Point((dstWidth_px/2)+(testRectWidth_px/2), dstHeight_px-1);
    Point dstP3_px = Point((dstWidth_px/2)+(testRectWidth_px/2), dstHeight_px-1-testRectHeight_px);
    Point dstP4_px = Point((dstWidth_px/2)-(testRectWidth_px/2), dstHeight_px-1-testRectHeight_px);

    return calibrateCameraImage(testRectWidth_cm, testRectHeight_cm, offsetToOrigin_cm,
                                dstWidth_px, dstHeight_px,
                                srcP1_px, srcP2_px, srcP3_px, srcP4_px,
                                dstP1_px, dstP2_px, dstP3_px, dstP4_px);
}

Point2d CameraCalibration::getWorldCoordinatesFrom2DImageCoordinates(Point2i imageCoordinates2D) {
    // adapt pixel coordinates to the orientation and position of the world coordinate system
    double x_unscaledLocal = dstHeight - imageCoordinates2D.y - px_os_bottom;
    double y_unscaledLocal = -(imageCoordinates2D.x - (dstWidth / 2.0));
    // scale pixel coordinates to actual world units and add distance offset
    double x_world = x_unscaledLocal * height_cm_per_px + offset_cm;
    double y_world = (y_unscaledLocal * width_cm_per_px);

    return Point2d(x_world, y_world);
}
std::vector<Point2d>
CameraCalibration::getWorldCoordinatesFrom2DImageCoordinates(std::vector<Point2i> &imageCoordinates2D) {
    std::vector<Point2d> converted;
    converted.reserve(imageCoordinates2D.size());
    for (auto it : imageCoordinates2D) {
        converted.emplace_back(getWorldCoordinatesFrom2DImageCoordinates(it));
    }
    return converted;
}
Point2i CameraCalibration::get2DImageCoordinatesFromWorldCoordinates(Point2d worldCoordinates) {
    // scale to pixel distance units after shifting according to the world offset (but keep orientation)
    double x_unscaledLocal = ((worldCoordinates.x - offset_cm) * height_px_per_cm);
    double y_unscaledLocal = (worldCoordinates.y * width_px_per_cm);
    // change orientation to the image coordinate system and adapt to test plane offset (in the calibration image)
    double x_image = -y_unscaledLocal + (dstWidth / 2.0);
    double y_image = dstHeight - x_unscaledLocal - px_os_bottom;
    return Point2i((int)x_image,(int)y_image);
}

std::vector<Point2i>
CameraCalibration::get2DImageCoordinatesFromWorldCoordinates(std::vector<Point2d> worldCoordinates) {
    std::vector<Point2i> converted;
    converted.reserve(worldCoordinates.size());
    for (auto it : worldCoordinates) {
        converted.emplace_back(get2DImageCoordinatesFromWorldCoordinates(it));
    }
    return converted;
}

/*
Point2i CameraCalibration::get3DFrom2DImageCoordinates(Point2i imageCoordinates2D) {
    Mat& m = transformMatr;

    int type = m.type();
    int invType = invTransformMatr.type();
    int x = imageCoordinates2D.x;
    int y = imageCoordinates2D.y;
    int m11 = m.at<uchar>(1,1);
    int m12 = m.at<uchar>(1,2);
    int m13 = m.at<uchar>(1,3);
    int m21 = m.at<uchar>(2,1);
    int m22 = m.at<uchar>(2,2);
    int m23 = m.at<uchar>(2,3);
    uchar tmp = m.at<uchar>(3,1);
    int m31 = m.at<uchar>(3,1);
    int m32 = m.at<uchar>(3,2);
    int m33 = m.at<uchar>(3,3);
    int transformedX = (int) (m11*x + m12*y + m13)/(m31*x + m32*y + m33);
    int transformedY = (int) (m21*x + m22*y + m23)/(m31*x + m32*y + m33);
    return cv::Point2i(transformedX, transformedY);
}
 */

std::vector<Point2i> CameraCalibration::transformPoints(std::vector<Point2i> &points, Mat &m) {
    std::vector<Point2d> input;
    input.reserve(points.size());
    for (auto it : points) {
        input.emplace_back(Point2d(it.x, it.y));
    }
    std::vector<Point2d> output;
    output.reserve(points.size());
    cv::perspectiveTransform(input, output, m);
    std::vector<Point2i> ret;
    ret.reserve(output.size());
    for (auto it : output) {
        ret.emplace_back(Point2i((int)it.x, (int)it.y));
    }
    return ret;
}

std::vector<Point2i>
CameraCalibration::get3DFrom2DImageCoordinates(std::vector<Point2i>& imageCoordinates2D) {
    return transformPoints(imageCoordinates2D, invTransformMatr);
}

std::vector<Point2i> CameraCalibration::get2DFrom3DImageCoordinates(std::vector<Point2i> &imageCoordinates3D) {
    return transformPoints(imageCoordinates3D, transformMatr);
}

std::vector<Point2d>
CameraCalibration::getWorldCoordinatesFrom3DImageCoordinates(std::vector<Point2i> &imageCoordinates3D) {
    std::vector<Point2i> imageCoordinates2D = get2DFrom3DImageCoordinates(imageCoordinates3D);
    return getWorldCoordinatesFrom2DImageCoordinates(imageCoordinates2D);
}

std::vector<Point2i>
CameraCalibration::get3DImageCoordinatesFromWorldCoordinates(std::vector<Point2d> &worldCoordinates) {
    std::vector<Point2i> imageCoordinates2D = get2DImageCoordinatesFromWorldCoordinates(worldCoordinates);
    return get3DFrom2DImageCoordinates(imageCoordinates2D);
}

Mat CameraCalibration::getTransformMatr() {
    return transformMatr;
}

/* getters */

int CameraCalibration::getDstWidth() const {
    return dstWidth;
}

int CameraCalibration::getDstHeight() const {
    return dstHeight;
}

double CameraCalibration::getHeight_px_per_cm() const {
    return height_px_per_cm;
}

double CameraCalibration::getHeight_cm_per_px() const {
    return height_cm_per_px;
}

double CameraCalibration::getWidth_px_per_cm() const {
    return width_px_per_cm;
}

double CameraCalibration::getWidth_cm_per_px() const {
    return width_cm_per_px;
}

int CameraCalibration::getPx_os_bottom() const {
    return px_os_bottom;
}

int CameraCalibration::getOffset_cm() const {
    return offset_cm;
}
