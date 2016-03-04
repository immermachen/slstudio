//Yang Zhang: The combination of Gray code and phase shift from:
//Shuntaro Yamazaki, Masaaki Mochimaru, and Takeo Kanade, "Simultaneous
//self-calibration of a projector and a camera using structured light",
//In Proc. Projector Camera Systems 2011, pp. 67-74, June 2011

#include "CodecGrayPhase.h"
#include <cmath>
#include <iomanip>

#include "cvtools.h"
#include <QString>
#include <fstream>


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

    //std::cout<< "decodeFrames begin.......";

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
        std::cout<< "m_phase_map0: Max-Min = " << maxVal << "-" << minVal << std::endl;

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
        std::cout<< "m_phase_map1: Max-Min = " << maxVal << "-" << minVal << std::endl;

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

    if (dir & CodecDirHorizontal)
    {
        up = m_phase_map[0]*screenCols;
        //up = m_phase_map[0];
        mask = m_reliable_mask[0]>0;
    }
    if (dir & CodecDirVertical)
    {
        vp = m_phase_map[1]*screenRows; //good
        //vp = m_phase_map[1];          //bad ????
        mask = m_reliable_mask[1]>0;
    }

    // merge masks and reliable maps
    //    (a)   Logical-and:   if ( ( a>1 ) && (b<0) ) ...
    //    (b)   Bitwise-and:   x = a&b;   Corresponding bits are and'ed (e.g. 0&1 -> 0)
    if (dir == CodecDirBoth)  //Bitwise-and
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

        mask = m_reliable_mask[0]>0;
    }

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

void DecoderGrayPhase::decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading, int numCam)
{
/*
    int framesize=frames.size();
    shading = frames[framesize-2];    // Get shading (max) image
    shading.convertTo(shading, CV_8UC1);

    cv::Mat minImage = frames[framesize-1]; // Get min image

    //std::cout<< "decodeFrames begin.......";

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
        QString filename = QString("am_map_gray0_C%1.png").arg(numCam, 1);
        cv::imwrite(filename.toStdString(), tmp);   // gray_map is CV_16U using PNG

        tmp = m_gray_error[0].clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "DecodeFrame: Max-Min = " << maxVal << "-" << minVal << std::endl;
        tmp.convertTo(tmp,CV_8U, 255/(maxVal-minVal),-225*minVal/(maxVal-minVal));
        filename = QString("am_gray_error0_C%1.png").arg(numCam, 1);
        cv::imwrite(filename.toStdString(), tmp);  //gray_error is CV_8U using BMP

        tmp = m_phase_map[0].clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "DecodeFrame: Max-Min = " << maxVal << "-" << minVal << std::endl;
        tmp.convertTo(tmp,CV_16U, 65535/(maxVal-minVal),-65535*minVal/(maxVal-minVal));
        filename = QString("am_phase_map0_C%1.png").arg(numCam, 1);
        cv::imwrite(filename.toStdString(), tmp);   // gray_map is CV_16U using PNG

        tmp = m_mask[0].clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "DecodeFrame: Max-Min = " << maxVal << "-" << minVal << std::endl;
        //tmp.convertTo(tmp,CV_8U, 255/(maxVal-minVal),-255*min/(maxVal-minVal));
        tmp.convertTo(tmp,CV_8U,255.0/(maxVal-minVal),-255.0*minVal/(maxVal-minVal));
        filename = QString("am_mask0_C%1.BMP").arg(numCam, 1);
        cv::imwrite(filename.toStdString(), tmp);  //gray_error is CV_8U using BMP
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

        Mat Dst(tmp, Rect(0,1000,2448,1)); // Rect_(x, y, width,height);
        writeMatToFile(Dst,QString("am_map_phase_unwraped0_0_1000_2448_1_C%1.txt").arg(numCam, 1).toStdString().c_str(), 0);
        Mat Dst1(tmp, Rect(0,1010,2448,1)); // Rect_(x, y, width,height);
        writeMatToFile(Dst1,QString("am_map_phase_unwraped0_0_1010_2448_1_C%1.txt").arg(numCam, 1).toStdString().c_str(), 0);

        tmp.convertTo(tmp,CV_16U, 65535/(maxVal-minVal),-65535*minVal/(maxVal-minVal));
        //tmp.convertTo(tmp,CV_16U, 65535/(maxVal),0);
        QString filename = QString("am_map_phase_unwraped0_C%1.png").arg(numCam, 1);
        cv::imwrite(filename.toStdString(), tmp);   // gray_map is CV_16U using PNG

        tmp = m_phase_error[0].clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "m_phase_error: Max-Min = " << maxVal << "-" << minVal << std::endl;
        tmp.convertTo(tmp,CV_16U, 65535/(maxVal-minVal),-65535*minVal/(maxVal-minVal));
        //tmp.convertTo(tmp,CV_16U, 65535/(maxVal),0);
        filename = QString("am_phase_error0_C%1.png").arg(numCam, 1);
        cv::imwrite(filename.toStdString(), tmp);

        tmp = m_reliable_mask[0].clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "m_reliable_mask: Max-Min = " << maxVal << "-" << minVal << std::endl;
        tmp.convertTo(tmp,CV_8U,255.0/(maxVal-minVal),-255.0*minVal/(maxVal-minVal));
        //tmp.convertTo(tmp,CV_8U,255,0);
        filename = QString("am_mask_reliable0_C%1.BMP").arg(numCam, 1);
        cv::imwrite(filename.toStdString(), tmp);
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
        QString filename = QString("am_map_gray1_C%1.png").arg(numCam, 1);
        cv::imwrite(filename.toStdString(), tmp);   // gray_map is CV_16U using PNG

        tmp = m_gray_error[1].clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "DecodeFrame: Max-Min = " << maxVal << "-" << minVal << std::endl;
        tmp.convertTo(tmp,CV_8U, 255/(maxVal-minVal),-225*minVal/(maxVal-minVal));
        filename = QString("am_gray_error1_C%1.png").arg(numCam, 1);
        cv::imwrite(filename.toStdString(), tmp);  //gray_error is CV_8U using BMP

        tmp = m_phase_map[1].clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "DecodeFrame: Max-Min = " << maxVal << "-" << minVal << std::endl;
        tmp.convertTo(tmp,CV_16U, 65535/(maxVal-minVal),-65535*minVal/(maxVal-minVal));
        filename = QString("am_phase_map1_C%1.png").arg(numCam, 1);
        cv::imwrite(filename.toStdString(), tmp);

        tmp = m_mask[1].clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "DecodeFrame: Max-Min = " << maxVal << "-" << minVal << std::endl;
        //tmp.convertTo(tmp,CV_8U, 255/(maxVal-minVal),-255*min/(maxVal-minVal));
        tmp.convertTo(tmp,CV_8U,255.0/(maxVal-minVal),-255.0*minVal/(maxVal-minVal));
        filename = QString("am_mask1_C%1.BMP").arg(numCam, 1);
        cv::imwrite(filename.toStdString(), tmp);
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
        //std::cout<< "m_phase_map: Max-Min = " << maxVal << "-" << minVal << std::endl; //inner a subprocess can not use std::cout

        //How to convert cv::Mat to matlab
        //FileStorage file(QString("am_map_phase_unwraped1_1000_0_1_2050_C%1.yml").arg(numCam, 1).toStdString(), cv::FileStorage::WRITE);
        //file<<"am_map_phase_unwraped1"<<Dst;
        //FileStorage file1(QString("am_map_phase_unwraped1_minVal_%2_maxVal_%3_C%1.yml").arg(numCam, 1).arg(minVal).arg(maxVal).toStdString(), cv::FileStorage::WRITE);
        //file1<<"am_map_phase_unwraped1"<<tmp;
        Mat Dst(tmp, Rect(1000,0,1,2050)); // Rect_(x, y, width,height);
        writeMatToFile(Dst, QString("am_map_phase_unwraped1_1000_0_1_2050_C%1.txt").arg(numCam, 1).toStdString().c_str(), 1);
        Mat Dst1(tmp, Rect(1050,0,1,2050)); // Rect_(x, y, width,height);
        writeMatToFile(Dst1, QString("am_map_phase_unwraped1_1050_0_1_2050_C%1.txt").arg(numCam, 1).toStdString().c_str(), 1);

        tmp.convertTo(tmp,CV_16U, 65535/(maxVal-minVal),-65535*minVal/(maxVal-minVal));
        //tmp.convertTo(tmp,CV_16U, 65535/(maxVal),0);
        QString filename = QString("am_map_phase_unwraped1_C%1.png").arg(numCam, 1);
        cv::imwrite(filename.toStdString(), tmp);   // gray_map is CV_16U using PNG

        tmp = m_phase_error[1].clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "m_phase_error: Max-Min = " << maxVal << "-" << minVal << std::endl;
        tmp.convertTo(tmp,CV_16U, 65535/(maxVal-minVal),-65535*minVal/(maxVal-minVal));
        //tmp.convertTo(tmp,CV_16U, 65535/(maxVal),0);
        filename = QString("am_phase_error1_C%1.png").arg(numCam, 1);
        cv::imwrite(filename.toStdString(), tmp);   // gray_map is CV_16U using PNG

        tmp = m_reliable_mask[1].clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "m_reliable_mask: Max-Min = " << maxVal << "-" << minVal << std::endl;
        tmp.convertTo(tmp,CV_8U,255.0/(maxVal-minVal),-255.0*minVal/(maxVal-minVal));
        //tmp.convertTo(tmp,CV_8U,255,0);
        filename = QString("am_mask_reliable1_C%1.BMP").arg(numCam, 1);
        cv::imwrite(filename.toStdString() , tmp);  //gray_error is CV_8U using BMP
#endif
        }

    }

    //std::cout<< "decodeFrames begin00-CodecDirBoth=." << CodecDirBoth <<std::endl;


    // merge masks and reliable maps
    if (dir == CodecDirBoth)
    {
        mask = Mat::zeros(m_reliable_mask[0].size(), CV_8U);
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

//    std::cout<< "decodeFrames ennnd." <<std::endl;
    up = m_phase_map[0];                   //why? here is good, but flat and curve flache , just different from above
//     up = m_phase_map[0]*screenCols;      // here is bad
    vp = m_phase_map[1];
//     vp = m_phase_map[1]*screenRows;
    mask =  m_reliable_mask[0]>0;

    //debug
    {
#if 1
    double minVal,maxVal;
    Mat tmp = mask.clone();
    cv::minMaxIdx(tmp,&minVal,&maxVal);
    //std::cout<< "m_mask_final: Max-Min = " << maxVal << "-" << minVal << std::endl;
    tmp.convertTo(tmp,CV_8U,255.0/(maxVal-minVal),-255.0*minVal/(maxVal-minVal));
    QString filename = QString("am_mask_final_C%1.BMP").arg(numCam, 1);
    cv::imwrite(filename.toStdString() , tmp);  //gray_error is CV_8U using BMP

    //apply mask: in_mat.copyTo(out_mat, mask_mat);
    //Note that the input and output image should not be the same! OpenCV does not throw an error, but the behavior is undefined.
    Mat unwrapped_up; //masked up
    Mat unwrapped_vp;
    up.copyTo(unwrapped_up, mask);
    vp.copyTo(unwrapped_vp, mask);

    cv::minMaxIdx(unwrapped_up,&minVal,&maxVal);
    Mat temp = unwrapped_up.clone();
    temp.convertTo(temp,CV_16U, 65535/(maxVal-minVal),-65535*minVal/(maxVal-minVal));
    filename = QString("am_map_phase_unwraped0_C%1_maksed.png").arg(numCam, 1);
    cv::imwrite(filename.toStdString(), temp);

    Mat Dst(unwrapped_up, Rect(0,1000,2448,1)); // Rect_(x, y, width,height);
    writeMatToFile(Dst,QString("am_map_phase_unwraped0_0_1000_2448_1_C%1_masked.txt").arg(numCam, 1).toStdString().c_str(), 0);
    Mat Dst1(unwrapped_up, Rect(0,1010,2448,1)); // Rect_(x, y, width,height);
    writeMatToFile(Dst1,QString("am_map_phase_unwraped0_0_1010_2448_1_C%1_masked.txt").arg(numCam, 1).toStdString().c_str(), 0);

    cv::minMaxIdx(unwrapped_vp,&minVal,&maxVal);
    temp = unwrapped_vp.clone();
    temp.convertTo(temp,CV_16U, 65535/(maxVal-minVal),-65535*minVal/(maxVal-minVal));
    filename = QString("am_map_phase_unwraped1_C%1_masked.png").arg(numCam, 1);
    cv::imwrite(filename.toStdString(), temp);

    Mat Dst2(unwrapped_vp, Rect(1000,0,1,2050)); // Rect_(x, y, width,height);
    writeMatToFile(Dst2, QString("am_map_phase_unwraped1_1000_0_1_2050_C%1_masked.txt").arg(numCam, 1).toStdString().c_str(), 1);
    Mat Dst3(unwrapped_vp, Rect(1050,0,1,2050)); // Rect_(x, y, width,height);
    writeMatToFile(Dst3, QString("am_map_phase_unwraped1_1050_0_1_2050_C%1_masked.txt").arg(numCam, 1).toStdString().c_str(), 1);



#endif
    }
*/
    //debug save yml and load
    {
#if 1
        if(numCam==1)
        {
#if 0
            {
                cv::FileStorage up1("aa_up1.yml", cv::FileStorage::WRITE);
                up1<<"up1" <<up;
                up1.release();
                cv::FileStorage vp1("aa_vp1.yml", cv::FileStorage::WRITE);
                vp1<< "vp1"<<vp;
                vp1.release();

                cv::FileStorage mask1("aa_mask1.yml", cv::FileStorage::WRITE);
                mask1<< "mask1"<<mask;
                mask1.release();
                cv::FileStorage shading1("aa_shading1.yml", cv::FileStorage::WRITE);
                shading1<< "shading1"<<shading;
                shading1.release();
            }
#endif

#if 1
            {
                cv::FileStorage up1("aa_up1.yml", cv::FileStorage::READ);
                up1["up1"]>>up;
                cv::FileStorage vp1("aa_vp1.yml", cv::FileStorage::READ);
                vp1["vp1"]>>vp;
                cv::FileStorage mask1("aa_mask1.yml", cv::FileStorage::READ);
                mask1["mask1"]>>mask;
                cv::FileStorage shading1("aa_shading1.yml", cv::FileStorage::READ);
                shading1["shading1"]>>shading;
            }
#endif

        }
        else if(numCam==2)
        {
#if 0
            {
                cv::FileStorage up2("aa_up2.yml", cv::FileStorage::WRITE);
                up2<<"up2" <<up;
                up2.release();
                cv::FileStorage vp2("aa_vp2.yml", cv::FileStorage::WRITE);
                vp2<< "vp2"<<vp;
                vp2.release();
                cv::FileStorage mask2("aa_mask2.yml", cv::FileStorage::WRITE);
                mask2<< "mask2"<<mask;
                mask2.release();
                cv::FileStorage shading2("aa_shading2.yml", cv::FileStorage::WRITE);
                shading2<< "shading2"<<shading;
                shading2.release();
            }
#endif

#if 1
            {
                cv::FileStorage up2("aa_up2.yml", cv::FileStorage::READ);
                up2["up2"]>>up;
                cv::FileStorage vp2("aa_vp2.yml", cv::FileStorage::READ);
                vp2["vp2"]>>vp;
                cv::FileStorage mask2("aa_mask2.yml", cv::FileStorage::READ);
                mask2["mask2"]>>mask;
                cv::FileStorage shading2("aa_shading2.yml", cv::FileStorage::READ);
                shading2["shading2"]>>shading;
            }
#endif
        }
#endif
    }
}

void DecoderGrayPhase::writeMatToFile(cv::Mat& m, const std::string& filename, unsigned int dir)
{
    ofstream fout(filename.c_str());

    if(!fout)
    {
        cout<<"File Not Opened"<<endl;  return;
    }

    if(dir==0)
    {
        for(int i=0; i<m.rows; i++)
        {
            for(int j=0; j<m.cols; j++)
            {
                fout<<m.at<float>(i,j)<<"\t"; //Note: the type for output
            }
            fout<<endl;
        }
    }
    else
    {
        for(int j=0; j<m.cols; j++)
        {
            for(int i=0; i<m.rows; i++)
            {
                fout<<m.at<float>(i,j)<<"\t";
            }
            fout<<endl;
        }
    }

    fout.close();
}

void DecoderGrayPhase::writeMatToFile(cv::Mat& m, const std::string& filename, unsigned int dir, int value)
{
    ofstream fout(filename.c_str());

    if(!fout)
    {
        cout<<"File Not Opened"<<endl;  return;
    }

    if(dir==0)
    {
        for(int i=0; i<m.rows; i++)
        {
            for(int j=0; j<m.cols; j++)
            {
                unsigned int tmp  = m.at<uchar>(i,j);
                fout<< tmp <<"\t";
            }
            fout<<endl;
        }
    }
    else
    {
        for(int j=0; j<m.cols; j++)
        {
            for(int i=0; i<m.rows; i++)
            {
                unsigned int tmp  = m.at<uchar>(i,j);
                fout<< tmp <<"\t";
            }
            fout<<endl;
        }
    }

    fout.close();
}
