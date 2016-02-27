#ifndef RECONSTRUCTOR_H
#define RECONSTRUCTOR_H

#include "CalibrationData.h"

#include <opencv2/opencv.hpp>

class Triangulator {
    public:
        Triangulator(CalibrationData _calibration);
        CalibrationData getCalibration(){return calibration;}
        ~Triangulator(){}
        // Reconstruction
        void triangulate(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading, cv::Mat &pointCloud);
        void triangulate(cv::Mat &up0, cv::Mat &vp0, cv::Mat &mask0, cv::Mat &shading0, cv::Mat &up1, cv::Mat &vp1, cv::Mat &mask1, cv::Mat &shading1, cv::Mat &pointCloud);
    private:
        void triangulateFromUp(cv::Mat &up, cv::Mat &xyz);
        void triangulateFromVp(cv::Mat &vp, cv::Mat &xyz);
        void triangulateFromUpVp(cv::Mat &up, cv::Mat &vp, cv::Mat &xyz);
        void triangulateFromUpVp(cv::Mat &up0, cv::Mat &vp0, cv::Mat &up1, cv::Mat &vp1, cv::Mat &xyz);
        CalibrationData calibration;
        cv::Mat determinantTensor;
        cv::Mat uc, vc;
        cv::Mat lensMap1, lensMap2; //FOR Camera0
        cv::Mat lensMap1_P, lensMap2_P; // for camera1
};

#endif
