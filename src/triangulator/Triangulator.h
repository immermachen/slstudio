#ifndef RECONSTRUCTOR_H
#define RECONSTRUCTOR_H

#include "CalibrationData.h"

#include <opencv2/opencv.hpp>

typedef struct intersection{
    // Mat.at(x, y) = (vp, up)
    //TODO:  x,y can be float for subpixel accuracy, later do this!!!
    unsigned short x;//row of image coordinate; unsigned short=65,535
    unsigned short y;//col of image coordiante;
    float vp;//row phase value of image coordinate; unsigned short=65,535
    float up;//col phase value of image coordiante;
    float distance;//distance to corresponding pixel in another image;
    unsigned int id;   //32 bit integer enough: index for one image: 2050*24478=50,179,900 unsigned long= unsigned int=4,294,967,295.
    intersection() : x(0), y(0), up(0), vp(0), id(0){}
    intersection(unsigned short _x, unsigned short _y, float _up, float _vp, unsigned int _id) :
        x(_x), y(_y),up(_up), vp(_vp),id(_id){}
} intersection;

static bool sortingLarger(intersection i,intersection j){ return (i.id<j.id);}
static bool sortingLargerDistance(intersection i,intersection j){ return (i.distance<j.distance);}
static bool sortingEqual(intersection i,intersection j){ return (i.id==j.id);}

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
        void triangulateFromPhaseCorrelate(std::vector<intersection> &matches0, std::vector<intersection> &matches1, cv::Mat &xyz);
        void phasecorrelate(cv::Mat &up0, cv::Mat &vp0, cv::Mat &up1, cv::Mat &vp1, std::vector<intersection> matches0, std::vector<intersection> matches1);

        CalibrationData calibration;
        cv::Mat determinantTensor;
        cv::Mat uc, vc;
        cv::Mat lensMap1, lensMap2; //FOR Camera0
        cv::Mat lensMap1_P, lensMap2_P; // for camera1
};

#endif
