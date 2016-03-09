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
// 'window' is max correctable error in reference global code (must be less than half of period)
//void UnwrapPhase(const Field<2,float> &phase, const int period, const Field<2,float> &reference, Field<2,float>& result, Field<2,float>& unwrap_error);
void UnwrapPhase(const Mat &phase, const int period, const Mat &reference, Mat& result, Mat& unwrap_error, Mat& improved)
{
    //TODO: max correctable phase error: How
    std::cout<< "Warning: UnwrapPhase: How to set the window is better?" << std::endl;
    //float window = 2.0/period;
    float window = period/2.0;
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
                improved.at<float>(x,y) = diff;
            } else {
                result.at<float>(x, y) = graycode;  //e.g. = 11
            }
            unwrap_error.at<float>(x, y) = diff;   //[0 1]
        }
    }

}

} // namespace slib
