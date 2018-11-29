#ifndef IMAGE_PREPROCESSOR_H_
#define IMAGE_PREPROCESSOR_H_

#include <opencv2/opencv.hpp>
using namespace cv;


/**
 * This class expects an Open CV image in Mat format and processes it
 * in a state suitable for detecting lines.
 */
class ImagePreprocessor {
    public:
        ImagePreprocessor(Mat in, Point bottomLeft, Point bottomRight, Point topLeft, Point topRight):  // TODO: specify output points too?
            input(in),
            bl(bottomLeft = Point(0,in.rows-1)),
            br(bottomRight = Point(in.cols-1,0)),
            tl(topLeft = Point(0,0)),
            tr(topRight = Point(in.cols-1, in.rows-1))
        {};
        // TODO: overload constructor with camera angle and distance to ground as input?

        ~ImagePreprocessor();

        bool convertToGrayscale();
        bool transformPerspective();    // TODO: specify input: vanishing point? camera angle?

        Mat process();

    private:
        Mat input;
        Mat workingCopy;
        Point bl, br, tl, tr;
};

#endif // IMAGE_PREPROCESSOR_H_