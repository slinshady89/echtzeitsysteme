#include <CameraReader.hpp>
#include <lane_detection/CameraReader.hpp>


#define FRAMES_TO_DISCARD 4


CameraReader::CameraReader(int width, int height): cap(
#ifdef DEBUG
            VideoCapture("test.mp4")
#else
            VideoCapture()
#endif
            ) {
#ifndef DEBUG
    cap.open(0);
    if (!cap.isOpened()) {
        cap.open(1);
    }
    if (!cap.isOpened()) {
        std::cerr << "No video opened" << std::endl;
    }
    cap.set(CV_CAP_PROP_FRAME_WIDTH, width);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, height);
#endif
}

Mat CameraReader::readImage() {
    /* the video capture keeps the latest 5 frames -
       discard the first 4 (the older ones) in the buffer */
    for (int i=0; i<FRAMES_TO_DISCARD; i++) {
        cap.grab();
    }
    cap.read(image);
    return image;
}

VideoCapture& CameraReader::getVideoCapture() {
    return cap;
}

void CameraReader::setCameraSettings(double brightness, double contrast, double saturation, double hue) {
    cap.set(CV_CAP_PROP_BRIGHTNESS, brightness);
    cap.set(CV_CAP_PROP_CONTRAST, contrast);
    cap.set(CV_CAP_PROP_SATURATION, saturation);
    cap.set(CV_CAP_PROP_HUE, hue);
}
