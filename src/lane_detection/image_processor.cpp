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

Mat ImageProcessor::transformTo2D() {
    transformMatr = getPerspectiveTransform(srcPoints,dstPoints);
    Mat output = Mat::zeros(Size(dstWidth,dstHeight),image.type());
    warpPerspective(image, output, transformMatr, output.size()); // TODO: good idea to write back to the same image? allow a different image size than the original one?
    image = output;
    return image;
}

Mat ImageProcessor::filterColor(Scalar lowHSVColor, Scalar highHSVColor) {
    inRange(image, lowHSVColor, highHSVColor, image);
    return image;
}

Mat ImageProcessor::edgeDetection(int lowThresh, int highThresh) {
    Canny(image, image, lowThresh, highThresh);
    return image;
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
    circle(image, point, 4, Scalar(255,255,255), -1);
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

Mat ImageProcessor::getTransformMatr() {
    return transformMatr;
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

Point2i ImageProcessor::singleTrajPoint(int rightLaneDist_cm, int y_cm) {
    int pxHeight = (y_cm - offset_cm) * height_px_per_cm;
    int pxDistLane = rightLaneDist_cm * width_px_per_cm;

    int y = image.rows - pxHeight;
    int width = image.cols;
    Mat imageRow = image.row(y);
    for (int i=width-1; i>=0; i--) {
        Scalar pixel = imageRow.at<uchar>(0,i);

        if (pixel.val[0]!=0) {
            return Point2i(i-pxDistLane,y);
        }
    }
    std::cerr << "No trajectory point found." << std::endl;
    return Point2i(-1,-1);
}