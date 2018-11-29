#include <image_preprocessor.h>

Mat ImagePreprocessor::process() {
    convertToGrayscale();
    transformPerspective();
    return workingCopy;
}

bool ImagePreprocessor::convertToGrayscale() {
    cvtColor(input, workingCopy, CV_BGR2GRAY);
    return true;
}

bool ImagePreprocessor::transformPerspective() {
    Point2f srcPts[] = {tl, tr, bl, br};
    Point2f dstPts[] = {Point2f(0.0f, 0.0f), Point2f(input.cols-1, 0.0f), Point2f(0.0f, input.rows-1), Point2f(input.cols-1, input.rows-1)}; // TODO: replace with constants or dynamic values
    Mat transformMatr = Mat::zeros(input.rows, input.cols, input.type());
    transformMatr = getPerspectiveTransform(srcPts, dstPts);
    warpPerspective(workingCopy, workingCopy, transformMatr, workingCopy.size());   // works?
}