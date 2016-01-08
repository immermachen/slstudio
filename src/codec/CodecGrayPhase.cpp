#include "CodecGrayPhase.h"
#include <cmath>
#include <iomanip>

#include "cvtools.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

static unsigned int nPhases = 16;
static unsigned int Nhorz = 0;
static unsigned int Nvert = 0;
static unsigned int num_fringes = 8;
static float intensity_threshold = 0.1;
static unsigned int fringe_interval = 1;



#ifndef log2f
#define log2f(x) (log(x)/log(2.0))
#endif

using namespace std;


static int get_num_bits(int direction,   unsigned int screenval) {

    if (direction)
        return  ceilf(logf(screenval) / logf(2));//projector_height
    else
        return  ceilf(logf(screenval) / logf(2));//projector_width
}

/*
 * The purpose of this function is to convert an unsigned
 * binary number to reflected binary Gray code.
 *
 * The operator >> is shift right. The operator ^ is exclusive or.
 * Source: http://en.wikipedia.org/wiki/Gray_code
 */
static unsigned int binaryToGray(unsigned int num)
{
    return (num >> 1) ^ num;
}

/*
 * From Wikipedia: http://en.wikipedia.org/wiki/Gray_code
 * The purpose of this function is to convert a reflected binary
 * Gray code number to a binary number.
 */
static unsigned grayToBinary(unsigned num, unsigned numBits)
{
    for (unsigned shift = 1; shift < numBits; shift <<= 1){
        num ^= num >> shift;
    }
    return num;
}

/*
 * Function takes the decimal number
 * Function takes the Nth bit (1 to 31)
 * Return the value of Nth bit from decimal
 * Source: http://icfun.blogspot.com/2009/04/get-n-th-bit-value-of-any-integer.html
 */
static int get_bit(int decimal, int N){

    // Shifting the 1 for N-1 bits
    int constant = 1 << (N-1);

    // If the bit is set, return 1
    if( decimal & constant ){
        return 1;
    }

    // If the bit is not set, return 0
    return 0;
}

static inline int powi(int num, unsigned int exponent){
    // NOT EQUIVALENT TO pow()
    if(exponent == 0)
        return 1;

    float res = num;
    for(unsigned int i=0; i<exponent-1; i++)
        res *= num;

    return res;
}

// -----------------------------------------------Encoder-----------------------------------------------------------------

EncoderGrayPhase::EncoderGrayPhase(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Encoder(_screenCols, _screenRows, _dir)
{

    Nhorz = get_num_bits(0, _screenCols);
    Nvert = get_num_bits(1, _screenRows);
    N = 2;

    // Set total pattern number
    if(dir & CodecDirHorizontal)
        this->N += Nhorz * 2 + num_fringes;

    if(dir & CodecDirVertical)
        this->N += Nvert * 2 + num_fringes;

    if(dir & CodecDirHorizontal)
    {
        // Precompute horizontally encoding patterns
        // gray code
        for(unsigned int p=0; p<Nhorz; p++)
        {
            cv::Mat patternP(1, screenCols, CV_8UC3);
            cv::Mat patternP_inverse(1,screenCols,CV_8UC3);
            // Loop through columns in first row
            for(unsigned int j=0; j<screenCols; j++)
            {
                unsigned int jGray = binaryToGray(j);
                // Amplitude of channels
                float amp = get_bit(jGray, Nhorz-p);
                float amp_inv = 1.0-amp;
                patternP.at<cv::Vec3b>(0,j) = cv::Vec3b(255.0*amp,255.0*amp,255.0*amp);
                patternP_inverse.at<cv::Vec3b>(0,j) = cv::Vec3b(255.0*amp_inv,255.0*amp_inv,255.0*amp_inv);
            }
            patterns.push_back(patternP);
            patterns.push_back(patternP_inverse);
        }
        // phase shift: Horizontally encoding patterns
        for(unsigned int i=0; i<num_fringes; i++)
        {
            int phase = i;
            cv::Mat patternI = slib::GeneratePhaseCode(screenCols,num_fringes,phase);
            patternI = patternI.t();
            patterns.push_back(patternI);
        }

    }
    if(dir & CodecDirVertical)
    {
        // Precompute vertical encoding patterns
        for(unsigned int p=0; p<Nvert; p++)
        {
            cv::Mat patternP(screenRows, 1, CV_8UC3);
            cv::Mat patternP_inverse(screenRows, 1, CV_8UC3);
            // Loop through rows in first column
            for(unsigned int i=0; i<screenRows; i++)
            {
                unsigned int iGray = binaryToGray(i);
                // Amplitude of channels
                float amp = get_bit(iGray, Nvert-p);
                float amp_inv = 1.0-amp;
                patternP.at<cv::Vec3b>(i,0) = cv::Vec3b(255.0*amp,255.0*amp,255.0*amp);
                patternP_inverse.at<cv::Vec3b>(i,0) = cv::Vec3b(255.0*amp_inv,255.0*amp_inv,255.0*amp_inv);
            }
            patterns.push_back(patternP);
            patterns.push_back(patternP_inverse);
        }

        // phase shift: Vertically encoding patterns
        for(unsigned int i=0; i<num_fringes; i++)
        {
            int phase = i;
            cv::Mat patternI = slib::GeneratePhaseCode(screenRows,num_fringes,phase);
            patterns.push_back(patternI);
        }
    }

    cv::Mat patternOn(1, 1, CV_8UC3, cv::Scalar(0));
    patternOn.at<cv::Vec3b>(0,0) = cv::Vec3b(255, 255, 255);
    patterns.push_back(patternOn);
    cv::Mat patternOff(1, 1, CV_8UC3, cv::Scalar(0));
    patterns.push_back(patternOff);

    #if 0
        for(unsigned int i=0; i<patterns.size(); i++)
        {
            std::stringstream fileNameStream;
            fileNameStream << "pattern_" << std::setw(2) << std::setfill('0') << i << ".bmp";
            cv::imwrite(fileNameStream.str(), patterns[i]);
        }
    #endif
}

cv::Mat EncoderGrayPhase::getEncodingPattern(unsigned int depth)
{
    return patterns[depth];
}


//------------------------------------------------Decoder-------------------------------------------------------------------------
DecoderGrayPhase::DecoderGrayPhase(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Decoder(_screenCols, _screenRows, _dir)
{

    Nhorz = get_num_bits(0, _screenCols);
    Nvert = get_num_bits(1, _screenRows);

    N = 2;

    if(dir & CodecDirHorizontal)
        this->N += Nhorz * 2 + num_fringes;

    if(dir & CodecDirVertical)
        this->N += Nvert * 2 + num_fringes;

    frames.resize(N);
}

void DecoderGrayPhase::setFrame(unsigned int depth, const cv::Mat frame)
{
    //scale 225 to [0 1] float type
    Mat curFrame = frame.clone();
    curFrame.convertTo(curFrame,CV_32FC1);
    frames[depth] = curFrame;
}

void DecoderGrayPhase::convert_reliable_map(int direction)
{
    float maxerror = 2.0/num_fringes;
    m_reliable_mask[direction] = Mat::zeros(m_phase_error[direction].size(), CV_8U);
    Mat& phase_error = m_phase_error[direction];

    for (int x=0; x<phase_error.rows; x++) {
        for (int y=0; y<phase_error.cols; y++) {
            //if (phase_error.at<float>(x,y) < maxerror && m_gray_error[direction].at<uchar>(x,y) < 2) {
            if (phase_error.at<float>(x,y) < maxerror && m_mask[direction].at<uchar>(x,y) == 1) {
                m_reliable_mask[direction].at<uchar>(x,y) = 1;
            } else {
                m_reliable_mask[direction].at<uchar>(x,y) = 0;
            }
        }
    }
}

void DecoderGrayPhase::decode_gray(const std::vector<Mat>& images, int direction)
{
    uchar nbits = direction ? Nvert : Nhorz;
    std::vector<Mat> diff(nbits);

    m_gray_error[direction] = Mat::zeros(images[0].size(), CV_8UC1);

    for (int bit = 0; bit<nbits; bit++)
    {
        double minVal0, minVal1,maxVal0, maxVal1;
        cv::minMaxIdx(images[2*bit], &minVal0, &maxVal0);
        cv::minMaxIdx(images[2*bit+1], &minVal1, &maxVal1);

        float maxval = std::max(maxVal0, maxVal1);
        diff[nbits-1-bit] = images[2*bit]-images[2*bit+1];

        // count error
        float threshold = intensity_threshold * maxval;
        slib::CountGraycodeUncertainty(diff[nbits-1-bit], threshold, m_gray_error[direction]);
    }

    // decode graycode
    slib::DecodeGrayCodeImages(diff, m_gray_map[direction]);
}

void DecoderGrayPhase::decode_phase(const std::vector<Mat>& images, int direction)
{
    slib::DecodePhaseCodeImages(images, m_phase_map[direction]);
    slib::UnwrapPhase(m_phase_map[direction], fringe_interval * num_fringes, m_gray_map[direction], m_phase_map[direction], m_phase_error[direction]);
}

void DecoderGrayPhase::generate_mask(int direction)
{
    uchar nbits = direction ? Nvert : Nhorz;
    m_mask[direction] = Mat::zeros(m_gray_error[direction].size(), CV_8U);

    for (int x=0; x<m_gray_error[direction].rows; x++) {
        for (int y=0; y<m_gray_error[direction].cols; y++)
            //if (m_gray_error[direction].at<uchar>(x,y) < nbits-1)
            if (m_gray_error[direction].at<uchar>(x,y) < 2)
                m_mask[direction].at<uchar>(x,y) = 1;
            else
                m_mask[direction].at<uchar>(x,y) = 0;
    }
}

void DecoderGrayPhase::decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading)
{

    int framesize=frames.size();
    shading = frames[framesize-2];    // Get shading (max) image
    shading.convertTo(shading, CV_8UC1);

    cv::Mat minImage = frames[framesize-1]; // Get min image

    std::cout<< "decodeFrames begin.......";

    if(dir & CodecDirHorizontal){ // Construct up image.
        vector<cv::Mat> images(frames.begin(), frames.begin() + Nhorz*2);
        decode_gray(images,0);
        generate_mask(0);
        vector<cv::Mat> phases(frames.begin() + Nhorz*2, frames.begin() + Nhorz*2 + num_fringes);

        //decode_phase(phases,0);
        slib::DecodePhaseCodeImages(phases, m_phase_map[0]);

        //debug
        {
#if 1
        double minVal,maxVal;
        Mat tmp = m_gray_map[0].clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "DecodeFrame: Max-Min = " << maxVal << "-" << minVal << std::endl;
        tmp.convertTo(tmp,CV_16U, 65535/(maxVal-minVal),-65535*minVal/(maxVal-minVal));
        cv::imwrite("am_map_gray0.png", tmp);   // gray_map is CV_16U using PNG

        tmp = m_gray_error[0].clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "DecodeFrame: Max-Min = " << maxVal << "-" << minVal << std::endl;
        tmp.convertTo(tmp,CV_8U, 255/(maxVal-minVal),-225*minVal/(maxVal-minVal));
        cv::imwrite("am_gray_error0.png", tmp);  //gray_error is CV_8U using BMP

        tmp = m_phase_map[0].clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "DecodeFrame: Max-Min = " << maxVal << "-" << minVal << std::endl;
        tmp.convertTo(tmp,CV_16U, 65535/(maxVal-minVal),-65535*minVal/(maxVal-minVal));
        cv::imwrite("am_phase_map0.png", tmp);   // gray_map is CV_16U using PNG

        tmp = m_mask[0].clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "DecodeFrame: Max-Min = " << maxVal << "-" << minVal << std::endl;
        //tmp.convertTo(tmp,CV_8U, 255/(maxVal-minVal),-255*min/(maxVal-minVal));
        tmp.convertTo(tmp,CV_8U,255.0/(maxVal-minVal),-255.0*minVal/(maxVal-minVal));
        cv::imwrite("am_mask0.BMP", tmp);  //gray_error is CV_8U using BMP
#endif
        }

        slib::UnwrapPhase(m_phase_map[0], fringe_interval * num_fringes, m_gray_map[0], m_phase_map[0], m_phase_error[0]);
        convert_reliable_map(0);

        //debug
        {
#if 1
        double minVal,maxVal;

        Mat tmp = m_phase_map[0].clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "m_phase_map: Max-Min = " << maxVal << "-" << minVal << std::endl;
        tmp.convertTo(tmp,CV_16U, 65535/(maxVal-minVal),-65535*minVal/(maxVal-minVal));
        //tmp.convertTo(tmp,CV_16U, 65535/(maxVal),0);
        cv::imwrite("am_map_phase_unwraped.png", tmp);   // gray_map is CV_16U using PNG

        tmp = m_phase_error[0].clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "m_phase_error: Max-Min = " << maxVal << "-" << minVal << std::endl;
        tmp.convertTo(tmp,CV_16U, 65535/(maxVal-minVal),-65535*minVal/(maxVal-minVal));
        //tmp.convertTo(tmp,CV_16U, 65535/(maxVal),0);
        cv::imwrite("am_phase_error0.png", tmp);   // gray_map is CV_16U using PNG

        tmp = m_reliable_mask[0].clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "m_reliable_mask: Max-Min = " << maxVal << "-" << minVal << std::endl;
        tmp.convertTo(tmp,CV_8U,255.0/(maxVal-minVal),-255.0*minVal/(maxVal-minVal));
        //tmp.convertTo(tmp,CV_8U,255,0);
        cv::imwrite("am_mask_reliable0.BMP", tmp);  //gray_error is CV_8U using BMP
#endif
        }

    }

//    cvtools::writeMat(up, "up.mat", "up");

    //std::cout<< "decodeFrames begin00-CodecDirVertical=." << CodecDirVertical <<std::endl;

    if(dir & CodecDirVertical){
        // Construct vp image.
        vector<cv::Mat> images(frames.begin() + Nhorz*2 + num_fringes, frames.begin() + Nhorz*2 + num_fringes + Nvert*2);
        decode_gray(images,1);
        generate_mask(1);
        vector<cv::Mat> phases(frames.begin() + Nhorz*2 + num_fringes + Nvert*2, frames.begin() + Nhorz*2 + num_fringes + Nvert*2 + num_fringes);

        slib::DecodePhaseCodeImages(phases, m_phase_map[1]);

        //debug
        {
#if 1
        double minVal,maxVal;
        Mat tmp = m_gray_map[1].clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "DecodeFrame: Max-Min = " << maxVal << "-" << minVal << std::endl;
        tmp.convertTo(tmp,CV_16U, 65535/(maxVal-minVal),-65535*minVal/(maxVal-minVal));
        cv::imwrite("am_map_gray1.png", tmp);   // gray_map is CV_16U using PNG

        tmp = m_gray_error[1].clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "DecodeFrame: Max-Min = " << maxVal << "-" << minVal << std::endl;
        tmp.convertTo(tmp,CV_8U, 255/(maxVal-minVal),-225*minVal/(maxVal-minVal));
        cv::imwrite("am_gray_error1.png", tmp);  //gray_error is CV_8U using BMP

        tmp = m_phase_map[1].clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "DecodeFrame: Max-Min = " << maxVal << "-" << minVal << std::endl;
        tmp.convertTo(tmp,CV_16U, 65535/(maxVal-minVal),-65535*minVal/(maxVal-minVal));
        cv::imwrite("am_phase_map1.png", tmp);   // gray_map is CV_16U using PNG

        tmp = m_mask[1].clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "DecodeFrame: Max-Min = " << maxVal << "-" << minVal << std::endl;
        //tmp.convertTo(tmp,CV_8U, 255/(maxVal-minVal),-255*min/(maxVal-minVal));
        tmp.convertTo(tmp,CV_8U,255.0/(maxVal-minVal),-255.0*minVal/(maxVal-minVal));
        cv::imwrite("am_mask1.BMP", tmp);  //gray_error is CV_8U using BMP
#endif
        }

        slib::UnwrapPhase(m_phase_map[1], fringe_interval * num_fringes, m_gray_map[1], m_phase_map[1], m_phase_error[1]);
        convert_reliable_map(1);

        //debug
        {
#if 1
        double minVal,maxVal;

        Mat tmp = m_phase_map[1].clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "m_phase_map: Max-Min = " << maxVal << "-" << minVal << std::endl;
        tmp.convertTo(tmp,CV_16U, 65535/(maxVal-minVal),-65535*minVal/(maxVal-minVal));
        //tmp.convertTo(tmp,CV_16U, 65535/(maxVal),0);
        cv::imwrite("am_map_phase_unwraped1.png", tmp);   // gray_map is CV_16U using PNG

        tmp = m_phase_error[1].clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "m_phase_error: Max-Min = " << maxVal << "-" << minVal << std::endl;
        tmp.convertTo(tmp,CV_16U, 65535/(maxVal-minVal),-65535*minVal/(maxVal-minVal));
        //tmp.convertTo(tmp,CV_16U, 65535/(maxVal),0);
        cv::imwrite("am_phase_error1.png", tmp);   // gray_map is CV_16U using PNG

        tmp = m_reliable_mask[1].clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "m_reliable_mask: Max-Min = " << maxVal << "-" << minVal << std::endl;
        tmp.convertTo(tmp,CV_8U,255.0/(maxVal-minVal),-255.0*minVal/(maxVal-minVal));
        //tmp.convertTo(tmp,CV_8U,255,0);
        cv::imwrite("am_mask_reliable1.BMP", tmp);  //gray_error is CV_8U using BMP
#endif
        }

    }

    //std::cout<< "decodeFrames begin00-CodecDirBoth=." << CodecDirBoth <<std::endl;


    // merge masks and reliable maps
    if (dir & CodecDirBoth)
    {
        for (int x = 0; x < m_mask[0].rows; x++)
        {
            for (int y = 0; y < m_mask[0].cols; y++)
            {
                if (!m_mask[1].at<uchar>(x, y))
                    m_mask[0].at<uchar>(x, y) = 0;
                m_reliable_mask[0].at<uchar>(x,y) = std::min(m_reliable_mask[0].at<uchar>(x,y),m_reliable_mask[1].at<uchar>(x,y));
            }
        }
    }


    std::cout<< "decodeFrames ennnd." <<std::endl;
    up = m_phase_map[0]*screenCols;
    vp = m_phase_map[1]*screenRows;
    mask = m_reliable_mask[0]>0;

    //debug
    {
#if 1
    double minVal,maxVal;
    Mat tmp = mask.clone();
    cv::minMaxIdx(tmp,&minVal,&maxVal);
    //std::cout<< "m_mask_final: Max-Min = " << maxVal << "-" << minVal << std::endl;
    tmp.convertTo(tmp,CV_8U,255.0/(maxVal-minVal),-255.0*minVal/(maxVal-minVal));
    cv::imwrite("am_mask_final.BMP", tmp);  //gray_error is CV_8U using BMP
#endif
    }
}
