#include "CodecGrayPhase.h"
#include <cmath>
#include <iomanip>

#include "cvtools.h"

static unsigned int Nhorz = 11;
static unsigned int Nvert = 11;
static unsigned int num_fringes = 8;
//int get_num_bits(int direction) const {
//    if (direction)
//        return  ceilf(logf(projector_height) / logf(2));
//    else
//        return  ceilf(logf(projector_width) / logf(2));
//}


using namespace std;

// -----------------------------------------------Encoder-----------------------------------------------------------------
void EncoderGrayPhase::get_gray(int direction, int id, slib::Field<2,float>& image) const {
    int level = m_options.get_num_bits(direction)  - 1 - id / 2;
    GenerateGrayCodeImage(direction, level, image);
    // complementary image
    if (id % 2)
        for (int y = 0; y < image.size(1); y++)
            for (int x = 0; x < image.size(0); x++)
                image.cell(x, y) = 1 - image.cell(x, y);
}

void EncoderGrayPhase::get_phase(int direction, int id, slib::Field<2,float>& image) const {
        GeneratePhaseCodeImage(direction, m_options.num_fringes, id, image);
}

void EncoderGrayPhase::GetImage(int id, slib::Field<2,float>& image) const {
    image.Initialize(m_options.projector_width,m_options.projector_height);
    if ( m_options.horizontal) {
        int num_bits = m_options.get_num_bits(0);
        if (id < num_bits*2) {
            get_gray(0, id, image);
            return;
        }
        id-=m_options.get_num_bits(0)*2;
        if (id < m_options.num_fringes) {
            get_phase(0, id, image);
            return;
        }
        id-=m_options.num_fringes;
    }
    if (m_options.vertical) {
        int num_bits = m_options.get_num_bits(1);
        if (id < m_options.get_num_bits(1)*2) {
            get_gray(1, id, image);
            return;
        }
        id-=m_options.get_num_bits(1)*2;
        if (id < m_options.num_fringes) {
            get_phase(1, id, image);
            return;
        }
        id-=m_options.num_fringes;
    }
    throw std::runtime_error("invalid image id");
}

EncoderGrayPhase::EncoderGrayPhase(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Encoder(_screenCols, _screenRows, _dir){

    N = 2;

    // Set total pattern number
    if(dir & CodecDirHorizontal)
        this->N += Nhorz;

    if(dir & CodecDirVertical)
        this->N += Nvert;

    // Encode every pixel column
    int NbitsHorz = ceilf(log2f((float)screenCols));

    // Number of vertical encoding patterns
    int NbitsVert = ceilf(log2f((float)screenRows));

    cv::Mat patternOn(1, 1, CV_8UC3, cv::Scalar(0));
    patternOn.at<cv::Vec3b>(0,0) = cv::Vec3b(255, 255, 255);
    patterns.push_back(patternOn);

    cv::Mat patternOff(1, 1, CV_8UC3, cv::Scalar(0));
    patterns.push_back(patternOff);

    if(dir & CodecDirHorizontal){
        // Precompute horizontally encoding patterns
        for(unsigned int p=0; p<Nhorz; p++){
            cv::Mat patternP(1, screenCols, CV_8UC3);
            // Loop through columns in first row
            for(unsigned int j=0; j<screenCols; j++){
                unsigned int jGray = binaryToGray(j);
                // Amplitude of channels
                float amp = get_bit(jGray, NbitsHorz-p);
                patternP.at<cv::Vec3b>(0,j) = cv::Vec3b(255.0*amp,255.0*amp,255.0*amp);
            }
            patterns.push_back(patternP);
        }
    }
    if(dir & CodecDirVertical){
        // Precompute vertical encoding patterns
        for(unsigned int p=0; p<Nvert; p++){
            cv::Mat patternP(screenRows, 1, CV_8UC3);

            // Loop through rows in first column
            for(unsigned int i=0; i<screenRows; i++){

                unsigned int iGray = binaryToGray(i);

                // Amplitude of channels
                float amp = get_bit(iGray, NbitsVert-p);
                patternP.at<cv::Vec3b>(i,0) = cv::Vec3b(255.0*amp,255.0*amp,255.0*amp);
            }
            patterns.push_back(patternP);
        }
    }

    #if 0
        for(unsigned int i=0; i<patterns.size(); i++){
            std::stringstream fileNameStream;
            fileNameStream << "pattern_" << std::setw(2) << std::setfill('0') << i << ".bmp";
            cv::imwrite(fileNameStream.str(), patterns[i]);
        }

    #endif
}

cv::Mat EncoderGrayPhase::getEncodingPattern(unsigned int depth){
    return patterns[depth];
}


//------------------------------------------------Decoder-------------------------------------------------------------------------

DecoderGrayCode::DecoderGrayPhase(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Decoder(_screenCols, _screenRows, _dir){

    N = 2;

    if(dir & CodecDirHorizontal)
        this->N += Nhorz;

    if(dir & CodecDirVertical)
        this->N += Nvert;

    frames.resize(N);

}

void DecoderGrayCode::setFrame(unsigned int depth, const cv::Mat frame){
    frames[depth] = frame;
}

void DecoderGrayCode::decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading){

    // Get shading (max) image
    shading = frames[0];

    // Get min image
    cv::Mat minImage = frames[1];

    // Threshold shading image for mask
    mask = (shading > 20) & (shading >  2.0*minImage);

    // Binarize frames. TODO: subpixel interpolation.
    vector<cv::Mat> framesBinary(frames.size());
    for(unsigned int i=0; i<frames.size(); i++){
        // Foreground pixels 1, background 0
        //cv::threshold(frames[i], framesBinary[i], 80, 1, cv::THRESH_BINARY);
        framesBinary[i].create(frames[0].size(), CV_8U);
        framesBinary[i] = cv::abs(shading-frames[i]) < cv::abs(frames[i]-minImage);
        cv::threshold(framesBinary[i], framesBinary[i], 1, 1, cv::THRESH_BINARY);
    }

    // Encode every pixel column
    int NbitsHorz = ceilf(log2f((float)screenCols));

    // Number of vertical encoding patterns
    int NbitsVert = ceilf(log2f((float)screenRows));

    if(dir & CodecDirHorizontal){
        vector<cv::Mat> framesHorz(framesBinary.begin()+2, framesBinary.begin()+Nhorz+2);

        // Construct up image.
        for(int i = 0; i < up.rows; i++){
            for(int j = 0; j < up.cols; j++){
                unsigned int enc = 0;
                for(unsigned int f=0; f<framesHorz.size(); f++){
                    // Gray decimal
                    enc += powi(2, NbitsHorz-f-1)*framesHorz[f].at<unsigned char>(i,j);
                }
                // Standard decimal
                enc = grayToBinary(enc, Nhorz);
                up.at<float>(i,j) = enc;

            }
        }
    }

//    cvtools::writeMat(up, "up.mat", "up");

    if(dir & CodecDirVertical){
        vector<cv::Mat> framesVert(framesBinary.end()-Nvert, framesBinary.end());

        // Construct vp image.
        for(int i = 0; i < vp.rows; i++){
            for(int j = 0; j < vp.cols; j++){
                unsigned int enc = 0;
                for(unsigned int f=0; f<framesVert.size(); f++){
                    // Gray decimal
                    enc += powi(2, NbitsVert-f-1)*framesVert[f].at<unsigned char>(i,j);
                }
                // Standard decimal
                enc = grayToBinary(enc, Nvert);
                vp.at<float>(i,j) = enc;
            }
        }
    }
}
