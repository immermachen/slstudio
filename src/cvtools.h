#ifndef CVTOOLS_H
#define CVTOOLS_H

#include <opencv2/opencv.hpp>

namespace cvtools{
    void phaseCorrelate(const cv::Mat &im1, const cv::Mat &im2, float &scale, float &angle, cv::Point2f &shift);
    cv::Mat logPolar(const cv::Mat &image, float scale);
    void initDistortMap(const cv::Matx33f cameraMatrix, const cv::Vec<float, 5> distCoeffs, const cv::Size size, cv::Mat &map1, cv::Mat &map2);
    cv::Mat diamondDownsample(cv::Mat &pattern);
    void imshow(const char *windowName, cv::Mat im, unsigned int x, unsigned int y);
    void imagesc(const char* windowName, cv::Mat im);
    cv::Mat histimage(cv::Mat histogram);
    void hist(const char *windowName, cv::Mat im, unsigned int x, unsigned int y);
    void writeMat( cv::Mat const& mat, const char* filename, const char* varName = "A", bool bgr2rgb = true);

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
}

#endif // CVTOOLS_H
