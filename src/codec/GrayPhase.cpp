#include "GrayPhase.h"


#define USE_8BIT_DUMP

namespace slib
{

//------------------------------------------------------------
// gray code
//------------------------------------------------------------
int GrayPhase::ConvertGrayToBinary(const unsigned long graycode)
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
void GrayPhase::GenerateGrayCodeImage(const int direction, const int level, Mat& bmp) //Field<2,float> &bmp
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

void GrayPhase::DecodeGrayCodeImages(const std::vector<Mat>& bmps, Mat& result)//Field<2,float>& result
{
//    int nlevels = bmps.size();
//    const CVector<2,int>& size = bmps[0].size();

//    // load binary codes
//    Field<2, unsigned long> binary(size, 0);
//    for (int l=0; l<nlevels; l++)
//        for (int y = 0; y < size[1]; y++)
//            for (int x = 0; x < size[0]; x++)
//                if (bmps[l].cell(x, y) > 0)
//                    binary.cell(x, y) += (1 << l);

//    // decode
//    result.Initialize(size);
//    for (int x = 0; x < size[0]; x++)
//    for (int y = 0; y < size[1]; y++)
//    {
//        int b = binary.cell(x, y);
//        int a = ConvertGrayToBinary(binary.cell(x, y));
//            result.cell(x, y) = a;
//    }

    int nlevels = bmps.size();
    const cv::Size& size = bmps[0].size();

    // load binary codes
    Mat binary = Mat::zeros(size, CV_32S);

    for (int l=0; l<nlevels; l++)
        for (int y = 0; y < size.width; y++)
            for (int x = 0; x < size.height; x++)
                if (bmps[l].at<float>(x, y) > 0)
                    binary.at<CV_32S>(x, y) += (1 << l);

    // decode
    result = Mat::zeros(size,CV_32S);

    for (int x = 0; x < size.height; x++)
    for (int y = 0; y < size.width; y++)
    {
        int b = binary.at<CV_32S>(x, y);
        int a = ConvertGrayToBinary(binary.at<CV_32S>(x, y));
            result.at<CV_32S>(x, y) = a;
    }
}

// update valid region by thresholding
//void CountGraycodeUncertainty(const Field<2,float> &diff, const float threshold, Field<2,int> &uncertainty);
void GrayPhase::CountGraycodeUncertainty(const Mat &diff, const float threshold, Mat &uncertainty)
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
void GrayPhase::GeneratePhaseCodeImage(const int direction, const int period, const int phase, Mat &bmp)
{
    std::vector<float> table(period);
    for (int i = 0; i < period; i++)
        table[i] = sin(2.0 * M_PI * (i + phase) / period) / 2.0 + 0.5;


    for (int x = 0; x < bmp.rows; x++)
        for (int y = 0; y < bmp.cols; y++)
        bmp.at<float>(x, y) = table[(direction ? x : y) % period];
}

// generate phase image from moire pattern images.
//void DecodePhaseCodeImages(const std::vector<Field<2,float>> &images, Field<2,float>& result);
void GrayPhase::DecodePhaseCodeImages(const std::vector<Mat> &images, Mat& result)
{
    const cv::Size& size = images[0].size();
    const int nphases = images.size();

    Mat mat = Mat(nphases,3,CV_32F);
    Mat matinv = Mat(3, nphases,CV_32F);

    for (int r = 0; r < nphases; r++)
    {
        mat(r, 0) = cos(2 * M_PI * r / nphases);
        mat(r, 1) = sin(2 * M_PI * r / nphases);
        mat(r, 2) = 1;
    }
    //mat = GetPseudoInverse(mat);
    cv::invert(mat, matinv,cv::DECOMP_SVD); //TODO

    result = Mat::zeros(size,CV_32F);

    for (int x = 0; x < size.height; x++)
    {
        for (int y = 0; y < size.width; y++)
        {
            Mat vec = Mat(nPhases,1,CV_32F);

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
void GrayPhase::UnwrapPhase(const Mat &phase, const int period, const Mat &reference, Mat& result, Mat& unwrap_error)
{
    // max correctable phase error
    float window = 2.0/period;
    result = Mat(phase.size(), CV_32F);
    unwrap_error = Mat(phase.size(), CV_32F);

    for (int x = 0; x < phase.rows; x++) {
        for (int y = 0; y < phase.cols; y++) {
            int graycode = reference.at<ushort>(x, y);//ushort=[0-65535], 16bit. so for [0 1024], e.g. =11
            float moire_phase = phase.at<float>(x, y); //correct:[0 1],  false case:[-2 2] maybe? so need to correct it. e.g. = 1.3
            float gray_phase = (float)(graycode % period) / period; //[0 1], e.g. =(11%4)/4=0.75

            if (_isnan(moire_phase)) {
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

            float diff = abs(gray_phase - moire_phase);
            if (diff < window) {
                //TODO: Yang: check this:::: how to deal with negative value???
                result.cell(x, y) = graycode - (graycode % period) + period * moire_phase;//e.g.: 11-(11%4)+4*0.3 = 9.2
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

////------------------------------------------------------------
//// for debug
////------------------------------------------------------------

//// dump a spatial pattern in color code
////template <typename T> void WriteCorrespondenceMap(const Field<2, T>&code, const Field<2,float> &mask, const std::string & filename, float scale);
//template <typename T> void GrayPhase::WriteCorrespondenceMap(const Mat& code, const Mat &mask, const std::string & filename, float scale)
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
//template <typename T> void GrayPhase::ExportCorrespondencePlot(const Mat& code, const Mat& mask, const std::string & filename)
//{
//    // export data for gnuplot
//    FILE *fw = fopen(filename.c_str(), "wb");
//    for (int i = 0; i < code.size(0) && i < code.size(1); i++) {
//        if (mask.cell(i,i))
//            fprintf(fw, "%d %f\n", i, (float)code.cell(i,i));
//    }
//    fclose(fw);
//}

} // namespace slib
