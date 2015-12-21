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
    private:
        std::vector<cv::Mat> frames;
};

#endif // CodecGrayPHASE_H
