#include "cvtools.h"

#ifdef _WIN32
#include <cstdint>
#endif

#include <stdio.h>

//Yang
#include <fstream>
//#include <iostream>
//#include <string>

namespace cvtools{

// Phase correlation image registration including scale, rotation and translational shift
void phaseCorrelate(const cv::Mat &im1, const cv::Mat &im2, float &scale, float &angle, cv::Point2f &shift){

    assert(im1.size() == im2.size());
    assert(im1.type() == im2.type());

    scale = 1.0;
    angle = 0.0;

//    cv::Mat im1Float, im2Float;
//    im1.convertTo(im1Float, CV_32F);
//    im2.convertTo(im2Float, CV_32F);

//    cv::Mat im1LogPolar = cvtools::logPolar(im1Float, 100.0);
//    cv::Mat im2LogPolar = cvtools::logPolar(im2Float, 100.0);

    // hanning window
    cv::Mat window;
    cv::createHanningWindow(window, im1.size(), CV_32F);

//    // determine scale and rotation
//    cv::Point2f scaleRotation = phasecorrelation::phaseCorrelate(im1LogPolar, im2LogPolar, window);

//    // convert scale to proper scale
//    scale = cv::exp(scaleRotation.x / 100.0);

//    // convert rotation angle to degrees
//    angle = -scaleRotation.y * 180.0/(im1.cols/2.0);

//    // correct for scale and rotation
//    cv::Mat im1ScaledRotated;

//    cv::Mat scaleRotationMatrix = cv::getRotationMatrix2D(cv::Point2f(im1.cols/2.0, im1.rows/2.0), angle, scale);
//    cv::warpAffine(im1Float, im1ScaledRotated, scaleRotationMatrix, im1Float.size());

    // determine translational shift
    //shift = phasecorrelation::phaseCorrelate(im1, im2, window);
}


// Log polar image transformation with log scaling factor (to bring intensities into proper range)
cv::Mat logPolar(const cv::Mat &image, float scale){

    cv::Mat result(image.size(), image.type());

    IplImage imageIpl(image);
    IplImage resultIpl(result);

    cvLogPolar(&imageIpl, &resultIpl, cv::Point2f(imageIpl.width/2.0, imageIpl.height/2.0), scale);

    return result;
}



// Forward distortion of points. The inverse of the undistortion in cv::initUndistortRectifyMap().
// Inspired by Pascal Thomet, http://code.opencv.org/issues/1387#note-11
// Convention for distortion parameters: http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html
void initDistortMap(const cv::Matx33f cameraMatrix, const cv::Vec<float, 5> distCoeffs, const cv::Size size, cv::Mat &map1, cv::Mat &map2){

    float fx = cameraMatrix(0,0);
    float fy = cameraMatrix(1,1);
    float ux = cameraMatrix(0,2);
    float uy = cameraMatrix(1,2);

    float k1 = distCoeffs[0];
    float k2 = distCoeffs[1];
    float p1 = distCoeffs[2];
    float p2 = distCoeffs[3];
    float k3 = distCoeffs[4];

    map1.create(size, CV_32F);
    map2.create(size, CV_32F);

    for(int col = 0; col < size.width; col++){
        for(int row = 0; row < size.height; row++){

            // move origo to principal point and convert using focal length
            float x = (col-ux)/fx;
            float y = (row-uy)/fy;

            float xCorrected, yCorrected;

            //Step 1 : correct distortion
            float r2 = x*x + y*y;
            //radial
            xCorrected = x * (1. + k1*r2 + k2*r2*r2 + k3*r2*r2*r2);
            yCorrected = y * (1. + k1*r2 + k2*r2*r2 + k3*r2*r2*r2);
            //tangential
            xCorrected = xCorrected + (2.*p1*x*y + p2*(r2+2.*x*x));
            yCorrected = yCorrected + (p1*(r2+2.*y*y) + 2.*p2*x*y);

            //convert back to pixel coordinates
            float col_displaced = xCorrected * fx + ux;
            float row_displaced = yCorrected * fy + uy;

            // correct the vector in the opposite direction
            map1.at<float>(row,col) = col+(col-col_displaced);
            map2.at<float>(row,col) = row +(row-row_displaced);
        }
    }
}

// Downsample a texture which was created in virtual column/row space for a diamond pixel array projector
cv::Mat diamondDownsample(cv::Mat &pattern){

    cv::Mat pattern_diamond(pattern.rows,pattern.cols/2,CV_8UC3);

    for(unsigned int col = 0; col < pattern_diamond.cols; col++){
        for(unsigned int row = 0; row < pattern_diamond.rows; row++){

            pattern_diamond.at<cv::Vec3b>(row,col)=pattern.at<cv::Vec3b>(row,col*2+row%2);
        }
    }

    return pattern_diamond;

}


void mouseCallback(int evt, int x, int y, int flags, void* param){
    cv::Mat *im = (cv::Mat*) param;
    if (evt == cv::EVENT_LBUTTONDOWN) {
        if(im->type() == CV_8UC3){
            printf("%d %d: %d, %d, %d\n",
                   x, y,
                   (int)(*im).at<cv::Vec3b>(y, x)[0],
                    (int)(*im).at<cv::Vec3b>(y, x)[1],
                    (int)(*im).at<cv::Vec3b>(y, x)[2]);
        } else if (im->type() == CV_32F) {
            printf("%d %d: %f\n",
                   x, y,
                   im->at<float>(y, x));
        }
    }
}

void imshow(const char *windowName, cv::Mat im, unsigned int x, unsigned int y){

    // Imshow
    //if(!cv::GetWindowHandle(windowName)){
        int windowFlags = CV_WINDOW_FREERATIO | CV_WINDOW_KEEPRATIO;
        cv::namedWindow(windowName, windowFlags);
        cv::moveWindow(windowName, x, y);
    //}
    cv::imshow(windowName, im);
}

void imagesc(const char *windowName, cv::Mat im){

    // Imshow with scaled image


}

cv::Mat histimage(cv::Mat histogram){

    cv::Mat histImage(512, 640, CV_8UC3, cv::Scalar(0));

    // Normalize the result to [ 2, histImage.rows-2 ]
    cv::normalize(histogram, histogram, 2, histImage.rows-2, cv::NORM_MINMAX, -1, cv::Mat());

    float bin_w = (float)histImage.cols/(float)histogram.rows;

    // Draw main histogram
    for(int i = 1; i < histogram.rows-10; i++){
        cv::line(histImage, cv::Point( bin_w*(i-1), histImage.rows - cvRound(histogram.at<float>(i-1)) ),
                 cv::Point( bin_w*(i), histImage.rows - cvRound(histogram.at<float>(i)) ),
                 cv::Scalar(255, 255, 255), 2, 4);
    }

    // Draw red max
    for(int i = histogram.rows-10; i < histogram.rows; i++){
        cv::line(histImage, cv::Point( bin_w*(i-1), histImage.rows - cvRound(histogram.at<float>(i-1)) ),
                 cv::Point( bin_w*(i), histImage.rows - cvRound(histogram.at<float>(i)) ),
                 cv::Scalar(255, 0, 0), 2, 4);
    }

    return histImage;
}

void hist(const char *windowName, cv::Mat histogram, unsigned int x, unsigned int y){

    // Display
    imshow(windowName, histimage(histogram), x, y);
    cv::Point(1,2);
}


void writeMat(cv::Mat const& mat, const char* filename, const char* varName, bool bgr2rgb){
    /*!
         *  \author Philip G. Lee <rocketman768@gmail.com>
         *  Write \b mat into \b filename
         *  in uncompressed .mat format (Level 5 MATLAB) for Matlab.
         *  The variable name in matlab will be \b varName. If
         *  \b bgr2rgb is true and there are 3 channels, swaps 1st and 3rd
         *  channels in the output. This is needed because OpenCV matrices
         *  are bgr, while Matlab is rgb. This has been tested to work with
         *  3-channel single-precision floating point matrices, and I hope
         *  it works on other types/channels, but not exactly sure.
         *  Documentation at <http://www.mathworks.com/help/pdf_doc/matlab/matfile_format.pdf>
         */
    int textLen = 116;
    char* text;
    int subsysOffsetLen = 8;
    char* subsysOffset;
    int verLen = 2;
    char* ver;
    char flags;
    int bytes;
    int padBytes;
    int bytesPerElement;
    int i,j,k,k2;
    bool doBgrSwap;
    char mxClass;
    int32_t miClass;
    uchar const* rowPtr;
    uint32_t tmp32;
    float tmp;
    FILE* fp;

    // Matlab constants.
    const uint16_t MI = 0x4d49; // Contains "MI" in ascii.
    const int32_t miINT8 = 1;
    const int32_t miUINT8 = 2;
    const int32_t miINT16 = 3;
    const int32_t miUINT16 = 4;
    const int32_t miINT32 = 5;
    const int32_t miUINT32 = 6;
    const int32_t miSINGLE = 7;
    const int32_t miDOUBLE = 9;
    const int32_t miMATRIX = 14;
    const char mxDOUBLE_CLASS = 6;
    const char mxSINGLE_CLASS = 7;
    const char mxINT8_CLASS = 8;
    const char mxUINT8_CLASS = 9;
    const char mxINT16_CLASS = 10;
    const char mxUINT16_CLASS = 11;
    const char mxINT32_CLASS = 12;
    const char mxUINT32_CLASS = 13;
    const uint64_t zero = 0; // Used for padding.

    fp = fopen( filename, "wb" );

    if( fp == 0 )
        return;

    const int rows = mat.rows;
    const int cols = mat.cols;
    const int chans = mat.channels();

    doBgrSwap = (chans==3) && bgr2rgb;

    // I hope this mapping is right :-/
    switch( mat.depth() ){
    case CV_8U:
        mxClass = mxUINT8_CLASS;
        miClass = miUINT8;
        bytesPerElement = 1;
        break;
    case CV_8S:
        mxClass = mxINT8_CLASS;
        miClass = miINT8;
        bytesPerElement = 1;
        break;
    case CV_16U:
        mxClass = mxUINT16_CLASS;
        miClass = miUINT16;
        bytesPerElement = 2;
        break;
    case CV_16S:
        mxClass = mxINT16_CLASS;
        miClass = miINT16;
        bytesPerElement = 2;
        break;
    case CV_32S:
        mxClass = mxINT32_CLASS;
        miClass = miINT32;
        bytesPerElement = 4;
        break;
    case CV_32F:
        mxClass = mxSINGLE_CLASS;
        miClass = miSINGLE;
        bytesPerElement = 4;
        break;
    case CV_64F:
        mxClass = mxDOUBLE_CLASS;
        miClass = miDOUBLE;
        bytesPerElement = 8;
        break;
    default:
        return;
    }

    //==================Mat-file header (128 bytes, page 1-5)==================
    text = new char[textLen]; // Human-readable text.
    memset( text, ' ', textLen );
    text[textLen-1] = '\0';
    const char* t = "MATLAB 5.0 MAT-file, Platform: PCWIN";
    memcpy( text, t, strlen(t) );

    subsysOffset = new char[subsysOffsetLen]; // Zeros for us.
    memset( subsysOffset, 0x00, subsysOffsetLen );
    ver = new char[verLen];
    ver[0] = 0x00;
    ver[1] = 0x01;

    fwrite( text, 1, textLen, fp );
    fwrite( subsysOffset, 1, subsysOffsetLen, fp );
    fwrite( ver, 1, verLen, fp );
    // Endian indicator. MI will show up as "MI" on big-endian
    // systems and "IM" on little-endian systems.
    fwrite( &MI, 2, 1, fp );
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    //===================Data element tag (8 bytes, page 1-8)==================
    bytes = 16 + 24 + (8 + strlen(varName) + (8-(strlen(varName)%8))%8)
            + (8 + rows*cols*chans*bytesPerElement);
    fwrite( &miMATRIX, 4, 1, fp ); // Data type.
    fwrite( &bytes, 4, 1, fp); // Data size in bytes.
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    //====================Array flags (16 bytes, page 1-15)====================
    bytes = 8;
    fwrite( &miUINT32, 4, 1, fp );
    fwrite( &bytes, 4, 1, fp );
    flags = 0x00; // Complex, logical, and global flags all off.

    tmp32 = 0;
    tmp32 = (flags << 8 ) | (mxClass);
    fwrite( &tmp32, 4, 1, fp );

    fwrite( &zero, 4, 1, fp ); // Padding to 64-bit boundary.
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    //===============Dimensions subelement (24 bytes, page 1-17)===============
    bytes = 12;
    fwrite( &miINT32, 4, 1, fp );
    fwrite( &bytes, 4, 1, fp );

    fwrite( &rows, 4, 1, fp );
    fwrite( &cols, 4, 1, fp );
    fwrite( &chans, 4, 1, fp );
    fwrite( &zero, 4, 1, fp ); // Padding to 64-bit boundary.
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    //==Array name (8 + strlen(varName) + (8-(strlen(varName)%8))%8 bytes, page 1-17)==
    bytes = strlen(varName);

    fwrite( &miINT8, 4, 1, fp );
    fwrite( &bytes, 4, 1, fp );
    fwrite( varName, 1, bytes, fp );

    // Pad to nearest 64-bit boundary.
    padBytes = (8-(bytes%8))%8;
    fwrite( &zero, 1, padBytes, fp );
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    //====Matrix data (rows*cols*chans*bytesPerElement+8 bytes, page 1-20)=====
    bytes = rows*cols*chans*bytesPerElement;
    fwrite( &miClass, 4, 1, fp );
    fwrite( &bytes, 4, 1, fp );

    for( k = 0; k < chans; ++k )
    {
        if( doBgrSwap )
        {
            k2 = (k==0)? 2 : ((k==2)? 0 : 1);
        }
        else
            k2 = k;

        for( j = 0; j < cols; ++j )
        {
            for( i = 0; i < rows; ++i )
            {
                rowPtr = mat.data + mat.step*i;
                fwrite( rowPtr + (chans*j + k2)*bytesPerElement, bytesPerElement, 1, fp );
            }
        }
    }

    // Pad to 64-bit boundary.
    padBytes = (8-(bytes%8))%8;
    fwrite( &zero, 1, padBytes, fp );
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    fclose(fp);
    delete[] text;
    delete[] subsysOffset;
    delete[] ver;
}


//Note: Not the true bilinear interpolation here: window: 15-20
//mask: update mask if the pixel is interpolatted.
void bilinearInterpolation(cv::Mat &input, cv::Mat &mask, ushort window)
{
    ushort nRows = input.rows;
    ushort nCols = input.cols;
    for(ushort i= window+1; i < nRows-window-1; i++)
    {
        for(ushort j=window+1; j<nCols-window-1;j++)
        {
            float p0 = input.at<float>(i,j);
            if(p0 != 0) continue;  //only fill the value when it is zero.

            //   * q12  *   r2   *   q22 *
            //   *          *           *
            //y2 *          pt          *
            //   *          *           *
            //y  *      pl  p0   pr     *
            //   *          *           *
            //y1 *          pb          *
            //   *          *           *
            //   * q11  *   r1   *   q21 *
            //   *      x1  x    x2
            //find the top and bottom boundary
            //float q11=0.0, q12=0.0, q21=0.0, q22=0.0;
            ushort shift=0, success=0;
            float pt=0.0, pb=0.0, pl=0.0, pr=0.0;
            ushort x1=0, x2=0, y1=0, y2=0;
            ushort x=j, y=i;

            while(shift<window)
            {
                shift++;
                float pt0= input.at<float>(i-shift , j);
                if(y2==0 && pt0 != 0.0){
                    success++;   y2=i-shift; pt = pt0;
                }
                float pb0= input.at<float>(i+shift , j);
                if(y1==0 && pb0 != 0.0){
                    success++;   y1=i+shift; pb = pb0;
                }
                float pl0 = input.at<float>(i, j-shift);
                if(x1==0 && pl0 != 0.0){
                    success++;   x1=j-shift; pl = pl0;
                }
                float pr0 = input.at<float>(i, j+shift);
                if(x2==0 && pr0 != 0.0){
                    success++;   x2=j+shift; pr = pr0;
                }
                if(success==4) break;
            }
            if(success!=4) continue;

            //If q12, q22, q11,q21 ==0 , bilinearInterpolation does not work!
            //p0 = bilinearInterpolation_onePixel(q11, q12, q21, q22, x1, x2, y1, y2, x, y);

            //So using linear interpolation here in both direction and then average them!
            float p0y = linearInterpolation(y,y2,y1,pt,pb);
            float p0x = linearInterpolation(x,x1,x2,pl,pr);
            p0 = (p0x+p0y)/2.0;
            input.at<float>(i,j) = p0;
            mask.at<uchar>(i,j) = 1;
        }
    }
}

inline float linearInterpolation(float q, float q0, float q1, float v0, float v1)
{
    float d1 = q-q0;
    float d = q1-q0;
//    f(q0)=v0                 f(q)=v          f(q1)=v1
//      q0                       q               q1
//       |---------d1------------|-------d2-------|
//       |--------------------d-------------------|
    //v=(d2/d)*v0 + (d1/d)*v1    or equivalently,
    //v=(1-d1/d)*v0 + (d1/d)*v1  or equivalently,
    float v=(d1/d)*(v1-v0) + v0;
    return v;
}

// https://helloacm.com
inline float bilinearInterpolation_onePixel(float q11, float q12, float q21, float q22, float x1, float x2, float y1, float y2, float x, float y)
{
    float x2x1, y2y1, x2x, y2y, yy1, xx1;
    x2x1 = x2 - x1;
    y2y1 = y2 - y1;
    x2x = x2 - x;
    y2y = y2 - y;
    yy1 = y - y1;
    xx1 = x - x1;
    return 1.0 / (x2x1 * y2y1) * (
        q11 * x2x * y2y +
        q21 * xx1 * y2y +
        q12 * x2x * yy1 +
        q22 * xx1 * yy1
    );
}

////Another triangulation method, instead of cv::triangulatePoints
////http://stackoverflow.com/questions/16295551/how-to-correctly-use-cvtriangulatepoints
////I tried cv::triangulatePoints, but somehow it calculates garbage.
////So to implement a linear triangulation method manually, which returns a 4x1 matrix for the triangulated 3D point.
//cv::Mat Triangulator::triangulate_Linear_LS(cv::Mat mat_P_l, cv::Mat mat_P_r, cv::Mat warped_back_l, cv::Mat warped_back_r)
//{
//    cv::Mat A(4,3,CV_64FC1), b(4,1,CV_64FC1), X(3,1,CV_64FC1), X_homogeneous(4,1,CV_64FC1), W(1,1,CV_64FC1);
//    W.at<double>(0,0) = 1.0;
//    A.at<double>(0,0) = (warped_back_l.at<double>(0,0)/warped_back_l.at<double>(2,0))*mat_P_l.at<double>(2,0) - mat_P_l.at<double>(0,0);
//    A.at<double>(0,1) = (warped_back_l.at<double>(0,0)/warped_back_l.at<double>(2,0))*mat_P_l.at<double>(2,1) - mat_P_l.at<double>(0,1);
//    A.at<double>(0,2) = (warped_back_l.at<double>(0,0)/warped_back_l.at<double>(2,0))*mat_P_l.at<double>(2,2) - mat_P_l.at<double>(0,2);
//    A.at<double>(1,0) = (warped_back_l.at<double>(1,0)/warped_back_l.at<double>(2,0))*mat_P_l.at<double>(2,0) - mat_P_l.at<double>(1,0);
//    A.at<double>(1,1) = (warped_back_l.at<double>(1,0)/warped_back_l.at<double>(2,0))*mat_P_l.at<double>(2,1) - mat_P_l.at<double>(1,1);
//    A.at<double>(1,2) = (warped_back_l.at<double>(1,0)/warped_back_l.at<double>(2,0))*mat_P_l.at<double>(2,2) - mat_P_l.at<double>(1,2);
//    A.at<double>(2,0) = (warped_back_r.at<double>(0,0)/warped_back_r.at<double>(2,0))*mat_P_r.at<double>(2,0) - mat_P_r.at<double>(0,0);
//    A.at<double>(2,1) = (warped_back_r.at<double>(0,0)/warped_back_r.at<double>(2,0))*mat_P_r.at<double>(2,1) - mat_P_r.at<double>(0,1);
//    A.at<double>(2,2) = (warped_back_r.at<double>(0,0)/warped_back_r.at<double>(2,0))*mat_P_r.at<double>(2,2) - mat_P_r.at<double>(0,2);
//    A.at<double>(3,0) = (warped_back_r.at<double>(1,0)/warped_back_r.at<double>(2,0))*mat_P_r.at<double>(2,0) - mat_P_r.at<double>(1,0);
//    A.at<double>(3,1) = (warped_back_r.at<double>(1,0)/warped_back_r.at<double>(2,0))*mat_P_r.at<double>(2,1) - mat_P_r.at<double>(1,1);
//    A.at<double>(3,2) = (warped_back_r.at<double>(1,0)/warped_back_r.at<double>(2,0))*mat_P_r.at<double>(2,2) - mat_P_r.at<double>(1,2);
//    b.at<double>(0,0) = -((warped_back_l.at<double>(0,0)/warped_back_l.at<double>(2,0))*mat_P_l.at<double>(2,3) - mat_P_l.at<double>(0,3));
//    b.at<double>(1,0) = -((warped_back_l.at<double>(1,0)/warped_back_l.at<double>(2,0))*mat_P_l.at<double>(2,3) - mat_P_l.at<double>(1,3));
//    b.at<double>(2,0) = -((warped_back_r.at<double>(0,0)/warped_back_r.at<double>(2,0))*mat_P_r.at<double>(2,3) - mat_P_r.at<double>(0,3));
//    b.at<double>(3,0) = -((warped_back_r.at<double>(1,0)/warped_back_r.at<double>(2,0))*mat_P_r.at<double>(2,3) - mat_P_r.at<double>(1,3));
//    cv::solve(A,b,X,DECOMP_SVD);
//    cv::vconcat(X,W,X_homogeneous);
//    return X_homogeneous;
//}
////------------------------------------------------------------
//// for debug
////------------------------------------------------------------

//// dump a spatial pattern in color code
////template <typename T> void WriteCorrespondenceMap(const Field<2, T>&code, const Field<2,float> &mask, const std::string & filename, float scale);
//template <typename T> void WriteCorrespondenceMap(const Mat& code, const Mat &mask, const std::string & filename, float scale)
//{
//    // export a color images
//#ifdef USE_8BIT_DUMP
//    Field<2,float> img(code.size());
//    for (int y = 0; y < code.size(1); y++) {
//        for (int x = 0; x < code.size(0); x++) {
//            if (mask.cell(x, y))
//                img.cell(x, y) = scale * code.cell(x, y);
//            else
//                img.cell(x, y) = 0;
//        }
//    }
//#else
//    Field<2,CVector<3,float>> img(code.size());
//    img.Clear(make_vector(0, 0, 0));
//    for (int y = 0; y < code.size(1); y++) {
//        for (int x = 0; x < code.size(0); x++) {
//            if (mask.cell(x, y)) {
//                float r, g, b;
//                image::GetHueColor(scale * code.cell(x, y), r, g, b);
//                img.cell(x, y) = make_vector(r, g, b);
//            }
//        }
//    }
//#endif
//    image::Write(img, filename);
//}

////template <typename T> void ExportCorrespondencePlot(const Field<2, T>&code, const Field<2,float> &mask, const std::string & filename);
//template <typename T> void ExportCorrespondencePlot(const Mat& code, const Mat& mask, const std::string & filename)
//{
//    // export data for gnuplot
//    FILE *fw = fopen(filename.c_str(), "wb");
//    for (int i = 0; i < code.size(0) && i < code.size(1); i++) {
//        if (mask.cell(i,i))
//            fprintf(fw, "%d %f\n", i, (float)code.cell(i,i));
//    }
//    fclose(fw);
//}

//plot float number between [0 1]: CV_32F;
void writeMatToFile(cv::Mat& m, const std::string& filename, unsigned int dir)
{
    std::ofstream fout(filename.c_str());

    if(!fout)
    {
        std::cout<<"File Not Opened"<<std::endl;  return;
    }

    if(dir==0)
    {
        for(int i=0; i<m.rows; i++)
        {
            for(int j=0; j<m.cols; j++)
            {
                fout<<m.at<float>(i,j)<<"\t"; //Note: the type for output
            }
            fout<<std::endl;
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
            fout<<std::endl;
        }
    }

    fout.close();
}

//type=0: uchar; type=1: ushort; type=2: float
// dir=0: horizontal;  dir==1: vertical
void writeMatToFile(cv::Mat& m, const std::string& filename, unsigned int dir, int type)
{
    std::ofstream fout(filename.c_str());

    if(!fout)
    {
        std::cout<<"File Not Opened"<<std::endl;  return;
    }

    if(dir==0)
    {
        for(int i=0; i<m.rows; i++)
        {
            for(int j=0; j<m.cols; j++)
            {
                if(type==0){
                    uchar tmp= m.at<uchar>(i,j);
                    fout<< tmp <<"\t";
                }
                if(type==1){
                    ushort tmp= m.at<ushort>(i,j);
                    fout<< tmp <<"\t";
                }
                if(type==2){
                    float tmp= m.at<float>(i,j);
                    fout<< tmp <<"\t";
                }
            }
            fout<<std::endl;
        }
    }
    else
    {
        for(int j=0; j<m.cols; j++)
        {
            for(int i=0; i<m.rows; i++)
            {
                if(type==0){
                    uchar tmp= m.at<uchar>(i,j);
                    fout<< tmp <<"\t";
                }
                if(type==1){
                    ushort tmp= m.at<ushort>(i,j);
                    fout<< tmp <<"\t";
                }
                if(type==2){
                    float tmp= m.at<float>(i,j);
                    fout<< tmp <<"\t";
                }
            }
            fout<<std::endl;
        }
    }

    fout.close();
}




}
