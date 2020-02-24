#ifndef STEREO3D_H
#define STEREO3D_H

#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace cv;

class Stereo3D {
    public:

        Stereo3D(std::string stereo_camera, std::string stereo_calibration, std::string stereo_matcher);

        void run();
};

#endif  // STEREO3D_H