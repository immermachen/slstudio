//Yang Zhang: The combination of Gray code and phase shift from:
//Shuntaro Yamazaki, Masaaki Mochimaru, and Takeo Kanade, "Simultaneous
//self-calibration of a projector and a camera using structured light",
//In Proc. Projector Camera Systems 2011, pp. 67-74, June 2011



#include "GrayPhase.h"


#define USE_8BIT_DUMP

namespace slib
{

//------------------------------------------------------------
// gray code
//------------------------------------------------------------
int ConvertGrayToBinary(const unsigned long graycode)
{
    int bincode = 0;
    int mask = 1;
    while (mask < graycode)
        mask *= 2;
    for (; mask; mask /= 2)
        bincode += (mask & graycode) ^ (mask & (bincode / 2));
    return bincode;
}

// generate a single bit plane of gray-code pattern
void GenerateGrayCodeImage(const int direction, const int level, Mat& bmp) //Field<2,float> &bmp
{
//    for (int y = 0; y < bmp.size(1); y++)
//    {
//        for (int x = 0; x < bmp.size(0); x++)
//        {
//            int binary = direction ? y : x;
//            int gray = binary ^ (binary >> 1);
//            bmp.cell(x, y) = (gray & (1 << level)) ? 1 : 0;
//        }
//    }

    for (int x = 0; x < bmp.rows; x++)
    {
        for (int y = 0; y < bmp.cols; y++)
        {
            int binary = direction ? x : y;
            int gray = binary ^ (binary >> 1);
            bmp.at<uchar>(x,y) = (gray & (1 << level)) ? 1 : 0;
        }
    }
}

void DecodeGrayCodeImages(const std::vector<Mat>& diffs, Mat& result)
{
    int nlevels = diffs.size();
    const cv::Size& size = diffs[0].size();

    // load binary codes
    Mat binary = Mat::zeros(size, CV_16UC1);

    for (int l=0; l<nlevels; l++)
        for (int x = 0; x < size.height; x++)
            for (int y = 0; y < size.width; y++)
                if (diffs[l].at<float>(x, y) > 0)
                    binary.at<ushort>(x, y) += (1 << l);

    // decode
    result = Mat::zeros(size,CV_16UC1);

    for (int x = 0; x < size.height; x++)
    for (int y = 0; y < size.width; y++)
    {
        result.at<ushort>(x, y) = ConvertGrayToBinary(binary.at<ushort>(x, y));;
    }
}

// update valid region by thresholding
//void CountGraycodeUncertainty(const Field<2,float> &diff, const float threshold, Field<2,int> &uncertainty);
void CountGraycodeUncertainty(const Mat &diff, const float threshold, Mat &uncertainty)
{
    for (int y = 0; y < diff.cols; y++)
        for (int x = 0; x < diff.rows; x++)
            if (abs(diff.at<float>(x, y)) < threshold)
                uncertainty.at<uchar>(x, y)++;
}

//------------------------------------------------------------
// phase-shifting code
//------------------------------------------------------------

// generate moire pattern images.
// 'period' is the phase period of sinusoidal curve in pixel
//void GeneratePhaseCodeImage(const int direction, const int period, const int phase, Field<2,float> &bmp);
void GeneratePhaseCodeImage(const int direction, const int period, const int phase, Mat &bmp)
{
    std::vector<float> table(period);
    for (int i = 0; i < period; i++)
        table[i] = sin(2.0 * M_PI * (i + phase) / period) / 2.0 + 0.5;

    for (int x = 0; x < bmp.rows; x++)
        for (int y = 0; y < bmp.cols; y++)
        bmp.at<float>(x, y) = table[(direction ? x : y) % period];
}

// generate moire pattern with only one row or one column.
// 'period' is the phase period of sinusoidal curve in pixel
Mat GeneratePhaseCode(unsigned int length,const int period, const int phase)
{
    cv::Mat phaseVector(length,1,CV_8UC3);
    std::vector<float> table(period);
    for (int i = 0; i < period; i++)
        table[i] = sin(2.0 * M_PI * (i + phase) / period) / 2.0 + 0.5;

    for (int x = 0; x < length; x++)
    {
        float amp = table[ x % period ] * 255.0;
        phaseVector.at<cv::Vec3b>(x, 0) = cv::Vec3b(amp,amp,amp);
    }

    return phaseVector;
}

// generate phase image from moire pattern images.
//void DecodePhaseCodeImages(const std::vector<Field<2,float>> &images, Field<2,float>& result);
void DecodePhaseCodeImages(const std::vector<Mat> &images, Mat& result)
{
    const cv::Size& size = images[0].size();
    const int nphases = images.size();

    Mat mat = Mat(nphases,3,CV_32F);
    Mat matinv = Mat(3, nphases,CV_32F);

    for (int r = 0; r < nphases; r++)
    {
        mat.at<float>(r, 0) = cos(2 * M_PI * r / nphases);
        mat.at<float>(r, 1) = sin(2 * M_PI * r / nphases);
        mat.at<float>(r, 2) = 1;
    }
    //mat = GetPseudoInverse(mat);
    cv::invert(mat, matinv,cv::DECOMP_SVD); //TODO

    result = Mat::zeros(size,CV_32F);

    for (int x = 0; x < size.height; x++)
    {
        for (int y = 0; y < size.width; y++)
        {
            Mat vec = Mat(nphases,1,CV_32F);

            for (int r = 0; r < nphases; r++)
                vec.at<float>(r,0) = images[r].at<float>(x, y);
            vec = matinv * vec;

            float A = sqrt(vec.at<float>(0, 0) *vec.at<float>(0, 0) + vec.at<float>(1, 0) * vec.at<float>(1, 0));
            float phi = atan2(vec.at<float>(0, 0) / A, vec.at<float>(1, 0) / A);
            float m = vec.at<float>(2, 0);
            while (phi < 0)
                phi += 2 * M_PI;
            result.at<float>(x, y) = phi / (2 * M_PI);
            //a += result.at<float>(x, y);
        }
    }
}

//------------------------------------------------------------
// phase unwrapping
//------------------------------------------------------------

// unwrap phase
// 'period' is phase period of sinusoidal curve in pixel
// 'reference' is reference integer code
// 'tolerance' is max correctable error in reference global code (must be less than half of period)
//void UnwrapPhase(const Field<2,float> &phase, const int period, const Field<2,float> &reference, Field<2,float>& result, Field<2,float>& unwrap_error);
void UnwrapPhase(const Mat &phase, const int period, const Mat &reference, Mat& result, Mat& unwrap_error)
{
    //TODO: max correctable phase error: How
    std::cout<< "Warning: UnwrapPhase: How to set the window is better?" << std::endl;
    float window = 2.0/period;
    //float window = 1.0;

    unwrap_error = Mat::zeros(phase.rows, phase.cols, CV_32F);

    for (int x = 0; x < phase.rows; x++) {
        for (int y = 0; y < phase.cols; y++) {
            int graycode = reference.at<ushort>(x, y);//ushort=[0-65535], 16bit. so for [0 1024], e.g. =11
            float moire_phase = phase.at<float>(x, y); //correct:[0 1],  false case:[-2 2] maybe? so need to correct it. e.g. = 1.3
            float gray_phase = (float)(graycode % period) / period; //[0 1], e.g. =(11%4)/4=0.75

            //if (_isnan(moire_phase)) {
            if (!moire_phase) {
                result.at<float>(x, y) = graycode;
                unwrap_error.at<float>(x,y) = 0.5;
                continue;
            }

            // normalized:  moire in [gray-0.5, gray+0.5)
            //  0      -w        +w 1
            // -|-------o====g====o-|----
            // ----x=========m=========o-
            if (moire_phase >= gray_phase + 0.5)    //Now correct the phase value if it not in [0 1], e.g. = 1.3 > 0.75+0.5
                moire_phase -= 1;                   // e.g.=0.3
            else if (moire_phase < gray_phase - 0.5)
                moire_phase += 1;

            float diff = fabs(gray_phase - moire_phase);
            if (diff < window) {
                //TODO: Yang: check this:::: how to deal with negative value???
                result.at<float>(x, y) = graycode - (graycode % period) + period * moire_phase;//e.g.: 11-(11%4)+4*0.3 = 9.2
//TODO:Yang:   Maybe solution is:
//                float newval = graycode - (graycode % period) + period * moire_phase;
//                if (newval < 0)
//                {
//                    result.at<float>(x, y) = graycode;
//                }
//                else
//                {
//                    result.at<float>(x, y) = newval;
//                }
            } else {
                result.at<float>(x, y) = graycode;  //e.g. = 11
            }
            unwrap_error.at<float>(x, y) = diff;   //[0 1]
        }
    }
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

} // namespace slib
