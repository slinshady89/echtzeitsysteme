#include <image_processor.hpp>

void ImageProcessor::calibrateCameraImage(double testRectWidth_cm, double testRectHeight_cm, double offsetToOrigin_cm,
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

    calibrated = true;
}

Mat ImageProcessor::transformTo2D() {
    Mat transformMatr = getPerspectiveTransform(srcPoints,dstPoints);
    Mat output = Mat::zeros(Size(dstWidth,dstHeight),image.type());
    warpPerspective(image, output, transformMatr, output.size()); // TODO: good idea to write back to the same image? allow a different image size than the original one?
    image = output;
    return image;
}

Mat ImageProcessor::filterColor() {
    // TODO: implement
    return Mat();
}

Point2d ImageProcessor::getWorldCoordinates(Point2i imageCoordinates) {
    Point2i unscaledCoordinates = Point2i(imageCoordinates.x - (image.cols/2), image.rows - imageCoordinates.y - px_os_bottom);
    return Point2d(unscaledCoordinates.x * width_cm_per_px, (unscaledCoordinates.y * height_cm_per_px) + offset_cm);
}

Point2i ImageProcessor::getImageCoordinates(Point2d worldCoordinates) {
    // TODO: implement
    return Point2i(-1,-1);
}

// debugging
Mat ImageProcessor::drawPoint(Point2i point) {
    circle(image, point, 4, Scalar(0,0,255), -1);
    return image;
}


Mat& ImageProcessor::getImage() {
    return image;
}