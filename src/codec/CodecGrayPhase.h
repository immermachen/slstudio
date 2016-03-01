#ifndef CodecGrayPHASE_H
#define CodecGrayPHASE_H

#include "Codec.h"
#include "GrayPhase.h"

class EncoderGrayPhase : public Encoder {
    public:
        EncoderGrayPhase(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir);
        // Encoding
        cv::Mat getEncodingPattern(unsigned int depth);
    private:
        std::vector<cv::Mat> patterns;

};

class DecoderGrayPhase : public Decoder {
    public:
        DecoderGrayPhase(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir);
        // Decoding
        void setFrame(unsigned int depth, const cv::Mat frame);
        void decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading);
        void decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading, int numCam);
    private:
        std::vector<cv::Mat> frames;

        Mat m_gray_map[2];  //CV_32F
        Mat m_gray_error[2]; //CV_8UC1 and have [0 11]
        Mat m_phase_map[2]; //CV_32F
        Mat m_phase_error[2]; //CV_32F
        Mat m_reliable_mask[2]; //CV_8UC1 and only [0 1]
        Mat m_mask[2]; //CV_8UC1 and only [0 1]
        void convert_reliable_map(int direction);
        void decode_gray(const std::vector<Mat>& images, int direction);
        void decode_phase(const std::vector<Mat>& images, int direction);
        //void apply_mask(const slib::Field<2,float>& mask, slib::Field<2,slib::CVector<3,float>>& img) const;
        void generate_mask(int direction);
        void writeMatToFile(cv::Mat& m, const std::string& filename, unsigned int dir, int value); // template TODO
        void writeMatToFile(cv::Mat& m, const std::string& filename, unsigned int dir);
};

#endif // CodecGrayPHASE_H
