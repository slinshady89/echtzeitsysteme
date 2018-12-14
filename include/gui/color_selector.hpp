#include <opencv2/opencv.hpp>

using namespace cv;

class ColorSelector {
    public:
        ColorSelector(std::string windowName);
        ~ColorSelector() {};

        int getLowH();
        int getLowS();
        int getLowV();
        int getHighH();
        int getHighS();
        int getHighV();

        int getLowCannyThresh();
        int getHighCannyThresh();


        // static ColorSelector getInstance() {
        //     if (&instance==NULL) {
        //         instance = ColorSelector("greenFilter");
        //     }
        //     return instance;
        // }



    private:
        

        // static ColorSelector instance;

        int lowH = 70;
        int lowS = 126;
        int lowV = 11;

        int highH = 132;
        int highS = 255;
        int highV = 118;

        int lowCannyThresh;
        int highCannyThresh;
};