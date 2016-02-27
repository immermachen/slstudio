#include "Triangulator.h"
#include <math.h>
#include <iostream>


#ifdef WIN32
    #ifndef NAN
        static const unsigned long __nan[2] = {0xffffffff, 0x7fffffff};
    #define NAN (*(const float *) __nan)
    #endif
#endif

Triangulator::Triangulator(CalibrationData _calibration) : calibration(_calibration){

    // Precompute uc, vc maps
    uc.create(calibration.frameHeight, calibration.frameWidth, CV_32F);
    vc.create(calibration.frameHeight, calibration.frameWidth, CV_32F);

    for(unsigned int row=0; row<calibration.frameHeight; row++){
        for(unsigned int col=0; col<calibration.frameWidth; col++){
            uc.at<float>(row, col) = col;
            vc.at<float>(row, col) = row;
        }
    }

    // Precompute determinant tensor
    cv::Mat Pc(3,4,CV_32F,cv::Scalar(0.0));
    cv::Mat(calibration.Kc).copyTo(Pc(cv::Range(0,3), cv::Range(0,3)));

    cv::Mat Pp(3,4,CV_32F), temp(3,4,CV_32F);
    cv::Mat(calibration.Rp).copyTo(temp(cv::Range(0,3), cv::Range(0,3)));
    cv::Mat(calibration.Tp).copyTo(temp(cv::Range(0,3), cv::Range(3,4)));
    Pp = cv::Mat(calibration.Kp) * temp;

    cv::Mat e = cv::Mat::eye(4, 4, CV_32F);

    int sz[] = {4, 3, 3, 3};
    cv::Mat C(4, sz, CV_32F, cv::Scalar::all(0));
    for(int k=0; k<4; k++){
        for(int i=0; i<3; i++){
            for(int j=0; j<3; j++){
                for(int l=0; l<3; l++){
                    cv::Mat op(4, 4, CV_32F);
                    Pc.row(i).copyTo(op.row(0));
                    Pc.row(j).copyTo(op.row(1));
                    Pp.row(l).copyTo(op.row(2));
                    e.row(k).copyTo(op.row(3));
                    C.at<float>(cv::Vec4i(k,i,j,l)) = cv::determinant(op.t());
                }
            }
        }
    }
    determinantTensor = C;

    // Precompute lens correction maps
    cv::Mat eye = cv::Mat::eye(3, 3, CV_32F);
    cv::initUndistortRectifyMap(calibration.Kc, calibration.kc, eye, calibration.Kc, cv::Size(calibration.frameWidth, calibration.frameHeight), CV_32FC1, lensMap1, lensMap2); //camera0
    cv::initUndistortRectifyMap(calibration.Kp, calibration.kp, eye, calibration.Kp, cv::Size(calibration.frameWidth, calibration.frameHeight), CV_32FC1, lensMap1_P, lensMap2_P); //camera1

    //cv::Mat map1, map2;
    //cv::normalize(lensMap1, map1, 0, 255, cv::NORM_MINMAX, CV_8U);
    //cv::normalize(lensMap2, map2, 0, 255, cv::NORM_MINMAX, CV_8U);
    //cv::imwrite("map1.png", map1);
    //cv::imwrite("map2.png", map2);
}


void Triangulator::triangulate(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading, cv::Mat &pointCloud){

    if(1)
    {
        double minVal,maxVal;
        cv::Mat tmp = up.clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "m_phase_map: Max-Min = " << maxVal << "-" << minVal << std::endl;
        tmp.convertTo(tmp,CV_16U, 65535/(maxVal-minVal),-65535*minVal/(maxVal-minVal));
        //tmp.convertTo(tmp,CV_16U, 65535/(maxVal),0);
        cv::imwrite("aup_distorted.png", tmp);   // gray_map is CV_16U using PNG

        tmp = vp.clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "m_phase_map: Max-Min = " << maxVal << "-" << minVal << std::endl;
        tmp.convertTo(tmp,CV_16U, 65535/(maxVal-minVal),-65535*minVal/(maxVal-minVal));
        //tmp.convertTo(tmp,CV_16U, 65535/(maxVal),0);
        cv::imwrite("avp_distorted.png", tmp);   // gray_map is CV_16U using PNG


        tmp = lensMap1.clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "m_phase_map: Max-Min = " << maxVal << "-" << minVal << std::endl;
        tmp.convertTo(tmp,CV_16U, 65535/(maxVal-minVal),-65535*minVal/(maxVal-minVal));
        //tmp.convertTo(tmp,CV_16U, 65535/(maxVal),0);
        cv::imwrite("alensMap1.png", tmp);   // gray_map is CV_16U using PNG
        tmp = lensMap2.clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "m_phase_map: Max-Min = " << maxVal << "-" << minVal << std::endl;
        tmp.convertTo(tmp,CV_16U, 65535/(maxVal-minVal),-65535*minVal/(maxVal-minVal));
        //tmp.convertTo(tmp,CV_16U, 65535/(maxVal),0);
        cv::imwrite("alensMap2.png", tmp);   // gray_map is CV_16U using PNG
    }

    // Undistort up, mask and shading
    if(!up.empty()){
        cv::Mat upUndistort;
        cv::remap(up, upUndistort, lensMap1, lensMap2, cv::INTER_LINEAR);
        up = upUndistort;
    }
    if(!vp.empty()){
        cv::Mat vpUndistort;
        cv::remap(vp, vpUndistort, lensMap1, lensMap2, cv::INTER_LINEAR);
        vp = vpUndistort;
    }

    cv::Mat maskUndistort, shadingUndistort;
    cv::remap(mask, maskUndistort, lensMap1, lensMap2, cv::INTER_LINEAR);
    cv::remap(shading, shadingUndistort, lensMap1, lensMap2, cv::INTER_LINEAR);
    mask = maskUndistort;
    shading = shadingUndistort;

    if(1)
    {
        double minVal,maxVal;
        cv::Mat tmp = up.clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "m_phase_map: Max-Min = " << maxVal << "-" << minVal << std::endl;
        tmp.convertTo(tmp,CV_16U, 65535/(maxVal-minVal),-65535*minVal/(maxVal-minVal));
        //tmp.convertTo(tmp,CV_16U, 65535/(maxVal),0);
        cv::imwrite("aup_undistorted.png", tmp);   // gray_map is CV_16U using PNG

        tmp = vp.clone();
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        //std::cout<< "m_phase_map: Max-Min = " << maxVal << "-" << minVal << std::endl;
        tmp.convertTo(tmp,CV_16U, 65535/(maxVal-minVal),-65535*minVal/(maxVal-minVal));
        //tmp.convertTo(tmp,CV_16U, 65535/(maxVal),0);
        cv::imwrite("avp_undistorted.png", tmp);   // gray_map is CV_16U using PNG
    }

    // Triangulate
    cv::Mat xyz;
    if(!up.empty() && vp.empty())
        triangulateFromUp(up, xyz);
    else if(!vp.empty() && up.empty())
        triangulateFromVp(vp, xyz);
    else if(!up.empty() && !vp.empty())
        triangulateFromUpVp(up, vp, xyz);

    // Mask
    pointCloud = cv::Mat(up.size(), CV_32FC3, cv::Scalar(NAN, NAN, NAN));
    xyz.copyTo(pointCloud, mask);

}

void Triangulator::triangulate(cv::Mat &up0, cv::Mat &vp0, cv::Mat &mask0, cv::Mat &shading0, cv::Mat &up1, cv::Mat &vp1, cv::Mat &mask1, cv::Mat &shading1, cv::Mat &pointCloud)
{
    // Undistort up, mask and shading
    if(!up0.empty()){
        cv::Mat upUndistort;
        cv::remap(up0, upUndistort, lensMap1, lensMap2, cv::INTER_LINEAR);
        up0 = upUndistort;
    }
    if(!up1.empty()){
        cv::Mat upUndistort;
        cv::remap(up1, upUndistort, lensMap1_P, lensMap2_P, cv::INTER_LINEAR);
        up1 = upUndistort;
    }
    if(!vp0.empty()){
        cv::Mat vpUndistort;
        cv::remap(vp0, vpUndistort, lensMap1, lensMap2, cv::INTER_LINEAR);
        vp0 = vpUndistort;
    }
    if(!vp1.empty()){
        cv::Mat vpUndistort;
        cv::remap(vp1, vpUndistort, lensMap1_P, , cv::INTER_LINEAR);
        vp1 = vpUndistort;
    }

    cv::Mat maskUndistort0, shadingUndistort0, maskUndistort1, shadingUndistort1;
    cv::remap(mask0, maskUndistort0, lensMap1, lensMap2, cv::INTER_LINEAR);
    cv::remap(shading0, shadingUndistort0, lensMap1, lensMap2, cv::INTER_LINEAR);
    mask0 = maskUndistort0;
    shading0 = shadingUndistort0;

    cv::remap(mask1, maskUndistort1, lensMap1_P, lensMap2_P, cv::INTER_LINEAR);
    cv::remap(shading1, shadingUndistort1, lensMap1_P, lensMap2_P, cv::INTER_LINEAR);
    mask1 = maskUndistort1;
    shading1 = shadingUndistort1;

    // Triangulate
    cv::Mat xyz;
    triangulateFromUpVp(up0, vp0, up1, vp1, xyz);

    // Aplly Mask
    pointCloud = cv::Mat(up0.size(), CV_32FC3, cv::Scalar(NAN, NAN, NAN));
    xyz.copyTo(pointCloud, mask0);
    pointCloud.copyTo(pointCloud, mask1);

}

void Triangulator::triangulateFromUp(cv::Mat &up, cv::Mat &xyz){

    // Solve for xyzw using determinant tensor
    cv::Mat C = determinantTensor;
    std::vector<cv::Mat> xyzw(4);
    for(unsigned int i=0; i<4; i++){
//        xyzw[i].create(up.size(), CV_32F);
        xyzw[i] = C.at<float>(cv::Vec4i(i,0,1,0)) - C.at<float>(cv::Vec4i(i,2,1,0))*uc - C.at<float>(cv::Vec4i(i,0,2,0))*vc -
                C.at<float>(cv::Vec4i(i,0,1,2))*up + C.at<float>(cv::Vec4i(i,2,1,2))*up.mul(uc) + C.at<float>(cv::Vec4i(i,0,2,2))*up.mul(vc);
    }

    // Convert to non homogenous coordinates
    for(unsigned int i=0; i<3; i++)
        xyzw[i] /= xyzw[3];

    // Merge
    cv::merge(std::vector<cv::Mat>(xyzw.begin(), xyzw.begin()+3), xyz);

}

void Triangulator::triangulateFromVp(cv::Mat &vp, cv::Mat &xyz){

    // Solve for xyzw using determinant tensor
    cv::Mat C = determinantTensor;
    std::vector<cv::Mat> xyzw(4);
    for(unsigned int i=0; i<4; i++){
//        xyzw[i].create(vp.size(), CV_32F);
        xyzw[i] = C.at<float>(cv::Vec4i(i,0,1,1)) - C.at<float>(cv::Vec4i(i,2,1,1))*uc - C.at<float>(cv::Vec4i(i,0,2,1))*vc -
                C.at<float>(cv::Vec4i(i,0,1,2))*vp + C.at<float>(cv::Vec4i(i,2,1,2))*vp.mul(uc) + C.at<float>(cv::Vec4i(i,0,2,2))*vp.mul(vc);
    }

    // Convert to non homogenous coordinates
    for(unsigned int i=0; i<3; i++)
        xyzw[i] /= xyzw[3];

    // Merge
    cv::merge(std::vector<cv::Mat>(xyzw.begin(), xyzw.begin()+3), xyz);

}

void Triangulator::triangulateFromUpVp(cv::Mat &up, cv::Mat &vp, cv::Mat &xyz)
{

    std::cerr << "WARNING! NOT FULLY IMPLEMENTED!" << std::endl;
    int N = up.rows * up.cols;

    cv::Mat projPointsCam(2, N, CV_32F);
    uc.reshape(0,1).copyTo(projPointsCam.row(0));
    vc.reshape(0,1).copyTo(projPointsCam.row(1));

    cv::Mat projPointsProj(2, N, CV_32F);
    up.reshape(0,1).copyTo(projPointsProj.row(0));
    vp.reshape(0,1).copyTo(projPointsProj.row(1));

    cv::Mat Pc(3,4,CV_32F,cv::Scalar(0.0));
    cv::Mat(calibration.Kc).copyTo(Pc(cv::Range(0,3), cv::Range(0,3)));

    cv::Mat Pp(3,4,CV_32F), temp(3,4,CV_32F);
    cv::Mat(calibration.Rp).copyTo(temp(cv::Range(0,3), cv::Range(0,3)));
    cv::Mat(calibration.Tp).copyTo(temp(cv::Range(0,3), cv::Range(3,4)));
    Pp = cv::Mat(calibration.Kp) * temp;

    cv::Mat xyzw;
    cv::triangulatePoints(Pc, Pp, projPointsCam, projPointsProj, xyzw);

    xyz.create(3, N, CV_32F);
    for(int i=0; i<N; i++){
        xyz.at<float>(0,i) = xyzw.at<float>(0,i)/xyzw.at<float>(3,i);
        xyz.at<float>(1,i) = xyzw.at<float>(1,i)/xyzw.at<float>(3,i);
        xyz.at<float>(2,i) = xyzw.at<float>(2,i)/xyzw.at<float>(3,i);
    }

    xyz = xyz.t();
    xyz = xyz.reshape(3, up.rows);
}

void Triangulator::triangulateFromUpVp(cv::Mat &up0, cv::Mat &vp0, cv::Mat &up1, cv::Mat &vp1, cv::Mat &xyz)
{
    //TODO: find the corresponding points depending on the phase value in up and vp.


    int N = up0.rows * up0.cols;

    cv::Mat projPointsCam(2, N, CV_32F);
    up0.reshape(0,1).copyTo(projPointsCam.row(0));
    vp0.reshape(0,1).copyTo(projPointsCam.row(1));

    cv::Mat projPointsProj(2, N, CV_32F);
    up1.reshape(0,1).copyTo(projPointsProj.row(0));
    vp1.reshape(0,1).copyTo(projPointsProj.row(1));

    cv::Mat Pc(3,4,CV_32F,cv::Scalar(0.0));
    cv::Mat(calibration.Kc).copyTo(Pc(cv::Range(0,3), cv::Range(0,3)));

    cv::Mat Pp(3,4,CV_32F), temp(3,4,CV_32F);
    cv::Mat(calibration.Rp).copyTo(temp(cv::Range(0,3), cv::Range(0,3)));
    cv::Mat(calibration.Tp).copyTo(temp(cv::Range(0,3), cv::Range(3,4)));
    Pp = cv::Mat(calibration.Kp) * temp;

    cv::Mat xyzw;
    cv::triangulatePoints(Pc, Pp, projPointsCam, projPointsProj, xyzw);

    xyz.create(3, N, CV_32F);
    for(int i=0; i<N; i++){
        xyz.at<float>(0,i) = xyzw.at<float>(0,i)/xyzw.at<float>(3,i);
        xyz.at<float>(1,i) = xyzw.at<float>(1,i)/xyzw.at<float>(3,i);
        xyz.at<float>(2,i) = xyzw.at<float>(2,i)/xyzw.at<float>(3,i);
    }

    xyz = xyz.t();
    xyz = xyz.reshape(3, up0.rows);
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

