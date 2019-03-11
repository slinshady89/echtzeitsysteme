#include <lane_detection/image_processor.hpp>

#include <ros/ros.h>
#include <time.h>
#include "CameraCalibration.hpp"

Mat ImageProcessor::transformTo2D() {
    Mat output = Mat::zeros(Size(calibration.getDstWidth(), calibration.getDstHeight()), image.type());
    if(image.cols != 0 && image.rows != 0)
        warpPerspective(image, output, calibration.getTransformMatr(), output.size()); // TODO: good idea to write back to the same image? allow a different image size than the original one?

    image = output;

    return image;
}

Mat ImageProcessor::filterColor(Scalar lowHSVColor, Scalar highHSVColor) {
    if (colorType != HSV) {
        ROS_WARN("Image was not in HSV color! Converting to HSV...");
        convertToHSV();
    }
    inRange(image, lowHSVColor, highHSVColor, image);
    return image;
}

Mat ImageProcessor::edgeDetection(int lowThresh, int highThresh) {
    Canny(image, image, lowThresh, highThresh);
    return image;
}

// debugging
Mat ImageProcessor::drawPoint(Point2i point) {
    return drawPoint(point, Scalar(255, 255, 255));
}


Mat ImageProcessor::drawPoint(Point2i point, Scalar color) {
    circle(image, point, 4, color, -1);
    return image;
}

Mat& ImageProcessor::getImage() {
    return image;
}

Mat ImageProcessor::resize(int width, int height) {
    Mat resized;
    cv::resize(image, resized, Size(width, height), 0, 0, INTER_LINEAR); //size is variable
    image = resized;
    return image;
}

Mat ImageProcessor::regionOfInterest(int x, int y, int width, int height) {
    image = image(Rect(x, y, width, height));
    return image;
}

Mat ImageProcessor::removeNoise(int kwidth, int kheight) {
    blur(image, image, Size(kwidth, kheight));
    return image;
}

Mat ImageProcessor::convertToHSV() {
    if (!colorType==BGR) {
        std::cerr << "Conversion to HSV not possible!" << std::endl;
        return image;
    }
    cvtColor(image, image, COLOR_BGR2HSV);
    colorType = HSV;
    return image;
}

void ImageProcessor::setImage(Mat img, ColorType type) {
    image = img;
    colorType = type;
}

Point2i ImageProcessor::firstMatchFromRight(int pxY) {
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
Point2i ImageProcessor::firstMatchFromLeft(int pxY) {
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

Point2i ImageProcessor::nextMatch(int pxY, Point2i lastMatch) {
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
Point2i ImageProcessor::lastMatchLeft(Point2i start) {
    uchar* data = image.ptr<uchar>(start.y);
    for (int x=start.x; x>=0; x--) {
        if (data[x] == 0) { // black pixel found
            return Point2i(x+1, start.y);
        }
    }
    return Point2i(-1, -1);
}

Point2i ImageProcessor::lastMatchRight(Point2i start) {
    uchar* data = image.ptr<uchar>(start.y);
    for (int x=start.x; x<image.cols; x++) {
        if (data[x] == 0) {
            return Point2i(x-1, start.y);
        }
    }
    return Point2i(-1, -1);
}
