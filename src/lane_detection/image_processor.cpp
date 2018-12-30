#include <lane_detection/image_processor.hpp>

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

    // compute transformation matrix for perspective warping + inverse matrix
    transformMatr = getPerspectiveTransform(srcPoints,dstPoints);
    invTransformMatr = transformMatr.inv();

    calibrated = true;
}
void ImageProcessor::calibrateCameraImage(double testRectWidth_cm, double testRectHeight_cm, double offsetToOrigin_cm,
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

#include <ros/ros.h>
#include <time.h>

Mat ImageProcessor::transformTo2D() {
    TIMER_INIT
    /*
    TIMER_INIT
    TIMER_START
    transformMatr = getPerspectiveTransform(srcPoints,dstPoints);
    TIMER_STOP
    TIMER_EVALUATE(getPerspectiveTransform)
    */  
    TIMER_START
    Mat output = Mat::zeros(Size(dstWidth,dstHeight),image.type());
    warpPerspective(image, output, transformMatr, output.size()); // TODO: good idea to write back to the same image? allow a different image size than the original one?
    TIMER_STOP
    TIMER_EVALUATE(warpPerspective)
    
    TIMER_START
    image = output;
    TIMER_STOP
    TIMER_EVALUATE(copy image)

    return image;
}

Mat ImageProcessor::filterColor(Scalar lowHSVColor, Scalar highHSVColor) {
    TIMER_INIT
    TIMER_START
    if (colorType != HSV) {
        ROS_WARN("Image was not in HSV color! Converting to HSV...");
        convertToHSV();
    }
    inRange(image, lowHSVColor, highHSVColor, image);
    TIMER_STOP
    TIMER_EVALUATE(filterColor)
    return image;
}

Mat ImageProcessor::edgeDetection(int lowThresh, int highThresh) {
    Canny(image, image, lowThresh, highThresh);
    return image;
}

Point2d ImageProcessor::getWorldCoordinates(Point2i imageCoordinates) {
    TIMER_INIT
    TIMER_START
    Point2i unscaledCoordinates = Point2i(imageCoordinates.x - (image.cols/2), image.rows - imageCoordinates.y - px_os_bottom);
    double x = unscaledCoordinates.x * width_cm_per_px;
    double y = (unscaledCoordinates.y * height_cm_per_px) + offset_cm;
    TIMER_STOP
    TIMER_EVALUATE(getWorldCoordinates)
    return Point2d(x, y);
}

Point2i ImageProcessor::getImageCoordinates(Point2d worldCoordinates) {
    // TODO: implement
    return Point2d(-1,-1);
}

// debugging
Mat ImageProcessor::drawPoint(Point2i point) {
    TIMER_INIT
    TIMER_START
    circle(image, point, 4, Scalar(255,255,255), -1);
    TIMER_STOP
    TIMER_EVALUATE(drawPoint)
    return image;
}


Mat& ImageProcessor::getImage() {
    return image;
}

Mat ImageProcessor::resize(int width, int height) {
    TIMER_INIT
    TIMER_START
    Mat resized;
    cv::resize(image, resized, Size(width, height), 0, 0, INTER_LINEAR); //size is variable
    image = resized;
    TIMER_STOP
    TIMER_EVALUATE(resize)
    return image;
}

Mat ImageProcessor::regionOfInterest(int x, int y, int width, int height) {
    image = image(Rect(x, y, width, height));
    return image;
}

Mat ImageProcessor::getTransformMatr() {
    return transformMatr;
}

Mat ImageProcessor::removeNoise(int kwidth, int kheight) {
    TIMER_INIT
    TIMER_START
    blur(image, image, Size(kwidth, kheight));
    TIMER_STOP
    TIMER_EVALUATE(removeNoise)
    return image;
}

Mat ImageProcessor::convertToHSV() {
    TIMER_INIT
    TIMER_START
    if (!colorType==BGR) {
        std::cerr << "Conversion to HSV not possible!" << std::endl;
        return image;
    }
    cvtColor(image, image, COLOR_BGR2HSV);
    colorType = HSV;
    TIMER_STOP
    TIMER_EVALUATE(convertToHSV)
    return image;
}

void ImageProcessor::setImage(Mat img, ColorType type) {
    image = img;
    colorType = type;
}

Point2d ImageProcessor::singleTrajPoint(double rightLaneDist_cm, double y_cm, int colorThreshold) {
    TIMER_INIT
    TIMER_START
    int pxHeight = (y_cm - offset_cm) * height_px_per_cm;
    int pxDistLane = int(rightLaneDist_cm * width_px_per_cm);

    int y = image.rows - pxHeight;
    int width = image.cols;
    if (y>=0 && y<image.rows) { // validity check
        Mat imageRow = image.row(y);
        for (int i=width-1; i>=0; i--) {
            Scalar pixel = imageRow.at<uchar>(0,i);

            if (pixel.val[0]>colorThreshold) {
                TIMER_STOP
                TIMER_EVALUATE(singleTrajPoint)
                return Point2d(i-pxDistLane,y);
            }
        }
    }
    std::cerr << "No trajectory point found." << std::endl;
    TIMER_STOP
    TIMER_EVALUATE(singleTrajPoint)
    return Point2d(-1,-1);
}

Point2i ImageProcessor::firstMatchFromRight(Scalar lowHSV, Scalar highHSV, int pxY) {
    if (image.channels()!=1) {
        ROS_WARN("Pixel match search for a non-grayscale image!");
    }
    uchar* data = image.ptr<uchar>(pxY);
    for (int x=image.cols-1; x>=0; x--) {
        if (data[x] != 0) {
            return Point2i(x, pxY);
        }
    }
    return Point2i(-1, -1);
}
Point2i ImageProcessor::firstMatchFromLeft(Scalar lowHSV, Scalar highHSV, int pxY) {
    if (image.channels()!=1) {
        ROS_WARN("Pixel match search for a non-grayscale image!");
    }
    uchar* data = image.ptr<uchar>(pxY);
    for (int x=0; x<image.cols; x++) {
        if (data[x] != 0) {
            return Point2i(x, pxY);
        }
    }
    return Point2i(-1, -1);
}
Point2i ImageProcessor::nextMatch(Scalar lowHSV, Scalar highHSV, int pxY, Point2i lastMatch) {
    if (image.channels()!=1) {
        ROS_WARN("Pixel match search for a non-grayscale image!");
    }
    int startX = lastMatch.x;
    uchar* data = image.ptr<uchar>(pxY);
    for (int i=startX; i<image.cols-startX || i<=startX-1; i++) {
        if (data[startX+i] != 0) {
            return Point2i(startX+i, pxY);
        }
        if (data[startX-1-i] != 0) {
            return Point2i(startX-1-i, pxY);
        }
    }
    return Point2i(-1, -1);

}
