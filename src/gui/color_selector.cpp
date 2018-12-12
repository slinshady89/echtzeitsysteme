#include <gui/color_selector.hpp>
#include <functional>


static void on_low_H(int, void *) {
}
static void on_low_S(int, void *) {
}
static void on_low_V(int, void *) {
}
static void on_high_H(int, void *) {
}
static void on_high_S(int, void *) {
}
static void on_high_V(int, void *) {
}
static void on_low_canny_thresh(int, void *) {
}
static void on_high_canny_thresh(int, void *) {
}


ColorSelector::ColorSelector(std::string windowName) {
    namedWindow(windowName, WINDOW_AUTOSIZE);
    createTrackbar("low H", windowName, &lowH, 179, on_low_H);
    createTrackbar("low S", windowName, &lowS, 255, on_low_S);
    createTrackbar("low V", windowName, &lowV, 255, on_low_V);

    createTrackbar("high H", windowName, &highH, 179, on_high_H);
    createTrackbar("high S", windowName, &highS, 255, on_high_S);
    createTrackbar("high V", windowName, &highV, 255, on_high_V);

    createTrackbar("low canny threshold", windowName, &lowCannyThresh, 100, on_low_canny_thresh);
    createTrackbar("high canny threshold", windowName, &highCannyThresh, 300, on_high_canny_thresh);

}

int ColorSelector::getLowH() { return lowH; }
int ColorSelector::getLowS() { return lowS; }
int ColorSelector::getLowV() { return lowV; }
int ColorSelector::getHighH() { return highH; }
int ColorSelector::getHighS() { return highS; }
int ColorSelector::getHighV() { return highV; }
int ColorSelector::getLowCannyThresh() { return lowCannyThresh; }
int ColorSelector::getHighCannyThresh() { return highCannyThresh; }