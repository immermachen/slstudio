//
// Copyright (c) 2009-2011  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp)
//  and the National Institute of Advanced Industrial Science and Technology
//
// $Id: GrayCode.h 4590 2011-05-30 22:13:45Z shun $
//

//[Yang] Declaration
//I have convert Yamazaki's from Intel Math Kernel Library (MKL) into OpenCV Library.

#ifndef GrayPHASE_H
#define GrayPHASE_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <fstream>

#define USE_8BIT_DUMP

using namespace cv;

namespace slib
{
    //------------------------------------------------------------
    // gray code
    //------------------------------------------------------------

    int ConvertGrayToBinary(const unsigned long graycode);

    // generate a single bit plane of gray-code pattern
    void GenerateGrayCodeImage(const int direction, const int level, Mat& bmp);

    void DecodeGrayCodeImages(const std::vector<Mat>& bmps, Mat& result); //Field<2,float>& result

    // update valid region by thresholding
    //void CountGraycodeUncertainty(const Field<2,float> &diff, const float threshold, Field<2,int> &uncertainty);
    void CountGraycodeUncertainty(const Mat& diff, const float threshold, Mat& uncertainty);

    //------------------------------------------------------------
    // phase-shifting code
    //------------------------------------------------------------

    // generate moire pattern images.
    // 'period' is the phase period of sinusoidal curve in pixel
    //void GeneratePhaseCodeImage(const int direction, const int period, const int phase, Field<2,float> &bmp);
    void GeneratePhaseCodeImage(const int direction, const int period, const int phase, Mat& bmp);

    Mat GeneratePhaseCode(const unsigned int length, const int period, const int phase);

    // generate phase image from moire pattern images.
    //void DecodePhaseCodeImages(const std::vector<Field<2,float>> &images, Field<2,float>& result);
    void DecodePhaseCodeImages(const std::vector<Mat> &images, Mat& result);

    //------------------------------------------------------------
    // phase unwrapping
    //------------------------------------------------------------

    // unwrap phase
    // 'period' is phase period of sinusoidal curve in pixel
    // 'reference' is reference integer code
    // 'tolerance' is max correctable error in reference global code (must be less than half of period)
    //void UnwrapPhase(const Field<2,float> &phase, const int period, const Field<2,float> &reference, Field<2,float>& result, Field<2,float>& unwrap_error);
    void UnwrapPhase(const Mat& phase, const int period, const Mat& reference, Mat& result, Mat& unwrap_error);

    void bilinearInterpolation(cv::Mat &input, cv::Mat &mask, ushort window);

    inline float linearInterpolation(float q, float q0, float q1, float v0, float v1);

    inline float bilinearInterpolation_onePixel(float q11, float q12, float q21, float q22, float x1, float x2, float y1, float y2, float x, float y);

//    //------------------------------------------------------------
//    // for debug
//    //------------------------------------------------------------

//    // dump a spatial pattern in color code
//    //template <typename T> void WriteCorrespondenceMap(const Field<2, T>&code, const Field<2,float> &mask, const std::string & filename, float scale);
//    template <typename T> void WriteCorrespondenceMap(const Mat& code, const Mat &mask, const std::string & filename, float scale);

//    //template <typename T> void ExportCorrespondencePlot(const Field<2, T>&code, const Field<2,float> &mask, const std::string & filename);
//    template <typename T> void ExportCorrespondencePlot(const Mat&code, const Mat &mask, const std::string & filename);


    void writeMatToFile(cv::Mat& m, const std::string& filename, unsigned int dir);
    void writeMatToFile(cv::Mat& m, const std::string& filename, unsigned int dir, int value);
} // namespace slib
#endif // GrayPHASE_H
