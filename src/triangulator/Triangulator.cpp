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

    // Triangulate  // Vp better than Up, why?
    cv::Mat xyz;
    if(!up.empty() && vp.empty())
    {
        std::cout << "Using triangulateFromUp!" << std::endl;
        triangulateFromUp(up, xyz);
    }
    else if(!vp.empty() && up.empty())
    {
        std::cout << "Using triangulateFromVp!" << std::endl;
        triangulateFromVp(vp, xyz);
    }
    else if(!up.empty() && !vp.empty())
    {
        std::cout << "Triangulator::triangulate: TODO: Using triangulateFromVp instead of triangulateFromUpVp!" << std::endl;
        //triangulateFromUp(up, xyz); //bad
        triangulateFromVp(vp, xyz); //good
        //triangulateFromUpVp(up, vp, xyz); //not so good as Vp, it has skewed.
    }

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
        cv::remap(vp1, vpUndistort, lensMap1_P, lensMap2_P, cv::INTER_LINEAR);
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

//    //combine Mask
//    cv::Mat mask = cv::Mat::zeros(mask0.size(), CV_8U);
//    mask1.copyTo(mask, mask0);

    //apply mask
    cv::Mat up0_m, vp0_m, up1_m, vp1_m;
    up0.copyTo(up0_m, mask0);
    vp0.copyTo(vp0_m, mask0);
    up1.copyTo(up1_m, mask1);
    vp1.copyTo(vp1_m, mask1);

    //TODO: option: check the identity property of phase value: delete duplicate value;
    //validateIdentity()

    std::vector<intersection> matches0, matches1;
    cv::Mat mask = mask0; //as final mask
    phasecorrelate_Epipolar(up0_m, vp0_m, mask, up1_m, vp1_m, matches0, matches1);

    //debug
    {
#if 1
    double minVal,maxVal;
    cv::Mat tmp = mask.clone();
    cv::minMaxIdx(tmp,&minVal,&maxVal);
    tmp.convertTo(tmp,CV_8U,255.0/(maxVal-minVal),-255.0*minVal/(maxVal-minVal));
    cv::imwrite("am_mask_final.BMP", tmp);
#endif
    }

    std::cout << "Triangulator::phasecorrelate_Epipolar:  finished! size_matches0=" << matches0.size() << std::endl;

    // Triangulate
    cv::Mat xyz;
    triangulateFromPhaseCorrelate(matches0,matches1, xyz);

    std::cout << "Triangulator::triangulateFromPhaseCorrelate:  finished!" << std::endl;

    // Aplly Mask    
    pointCloud = cv::Mat(up0.size(), CV_32FC3, cv::Scalar(NAN, NAN, NAN));
    xyz.copyTo(pointCloud, mask);
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

    //Yang: TODO: should find the corresponding pixel position (x, y) of phase value (up, vp)???
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

void Triangulator::triangulateFromPhaseCorrelate(std::vector<intersection> &matches0, std::vector<intersection> &matches1, cv::Mat &xyz)
{
    assert(matches0.size()>0);
    assert(matches0.size() == matches1.size());
    int N = matches0.size();
    assert(N == calibration.frameWidth*calibration.frameHeight);

    cv::Mat projPointsCam(2, N, CV_32F);
    cv::Mat projPointsProj(2, N, CV_32F);

    for(unsigned int i=0; i<N; i++)
    {
        intersection p0 = matches0[i];
        intersection p1 = matches1[i];
        projPointsCam.at<float>(0, i) = p0.y;  // uc or up, col
        projPointsCam.at<float>(1, i) = p0.x;  // vc or vp, row
        projPointsProj.at<float>(0,i) = p1.y;
        projPointsProj.at<float>(1,i) = p1.x;
    }


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
    xyz = xyz.reshape(3, calibration.frameHeight); //convert back to up0.rows;
}

static void getIntersectionLabels(const cv::Mat& up, const cv::Mat& vp, std::vector<intersection>& intersections)
{
    int nRows = up.rows;
    int nCols = up.cols;
    unsigned int id=0;

    // collect intersections
    for(int i=0; i<nRows-1; i++){
        for(int j=0; j<nCols-1; j++){
            float _up = up.at<float>(i, j);
            float _vp = vp.at<float>(i, j);
            if(_up==0  || _vp==0) continue;

            intersections.push_back(intersection(i, j, _up, _vp, id++));
        }
    }
//    // Option: sort
//    std::sort(intersections.begin(), intersections.end(), sortingLarger);

//    // Option: remove duplicates
//    std::vector<intersection>::iterator it;
//    it = std::unique(intersections.begin(), intersections.end(), sortingEqual);
//    intersections.resize(std::distance(intersections.begin(),it));
}

void Triangulator::phasecorrelate(cv::Mat &up0, cv::Mat &vp0, cv::Mat &up1, cv::Mat &vp1, std::vector<intersection> matches0, std::vector<intersection> matches1)
{
    // get intersections
    std::vector<intersection> intersections0, intersections1;
    getIntersectionLabels(up0, vp0, intersections0);
    getIntersectionLabels(up1, vp1, intersections1);

    // match intersections
    for(int i=0; i < intersections0.size(); i++)
    {
        intersection p0 = intersections0[i];
        std::vector<intersection> windowsmatched; //collect some roughly matched in one Window;
        float windows = 2.0;
        for(int j=0; j<intersections1.size();j++)
        {
            intersection p1=intersections1[j];
            if( std::abs(p0.up-p1.up) < windows && std::abs(p0.vp - p1.vp) < windows)
            {
                p1.distance = std::abs(p0.up-p1.up) + std::abs(p0.vp - p1.vp);
                windowsmatched.push_back(p1);
            }
        }

        //Method1:find the closest one from the windows, currenty use this;
        //Method2: advance: do interpolation;
        if(windowsmatched.size()>0)
        {
            std::sort(windowsmatched.begin(), windowsmatched.end(), sortingLargerDistance); //ascending order
            intersection matched = windowsmatched[0];

            matches0.push_back(p0);
            matches1.push_back(matched);
        }
    }
    //option: subpixel refinement
}

//find phase correspondence using epipolar constraint.
void Triangulator::phasecorrelate_Epipolar(cv::Mat &up0, cv::Mat &vp0, cv::Mat &mask, cv::Mat &up1, cv::Mat &vp1, std::vector<intersection> &matches0, std::vector<intersection> &matches1)
{
    for(unsigned int row=0; row<calibration.frameHeight; row++){
        for(unsigned int col=0; col<calibration.frameWidth; col++){
            uc.at<float>(row, col) = col;
            vc.at<float>(row, col) = row;
        }
    }

    //loop up0 to find epipolar line
    ushort nRows = up0.rows;
    ushort nCols = up0.cols;
    std::vector<cv::Vec3f> epilines;
    std::vector<cv::Point2f> inputs;// Mat inputs = up0_row.reshape(0, 1).clone(); // 0: the same channel; 1: one row;
    for(ushort y=0; y < nRows; y++) // match intersections
    {
        for(ushort x=0; x<nCols;x++)
        {
            cv::Point2f point;
            point.x = x; //TODO: x is col??? In opencv x: col; y:row; same with matlab?
            point.y = y;
            inputs.push_back(point);
        }
    }
//    points – Input points. N*1 or 1*N matrix of type CV_32FC2 or vector<Point2f> .
//    whichImage – Index of the image (1 or 2) that contains the points .
//    F – Fundamental matrix that can be estimated using findFundamentalMat() or stereoRectify() .
//    lines – Output vector of the epipolar lines corresponding to the points in the other image. Each line ax + by + c=0 is encoded by 3 numbers (a, b, c) .
    cv::computeCorrespondEpilines(inputs, 1, calibration.F, epilines);
    cv::Mat epilineMat = cv::Mat(epilines).reshape(0, up0.rows);//.t(); //3-channel

    //debug epipolar
    {
#if 1
        double minVal,maxVal;
        cv::Mat tmp = up0.clone();        
        cv::minMaxIdx(tmp,&minVal,&maxVal);
        tmp.convertTo(tmp,CV_16U, 65535/(maxVal-minVal),-65535*minVal/(maxVal-minVal));
        cv::Mat tmpcolor;
        cv::cvtColor(tmp, tmpcolor, cv::COLOR_GRAY2RGB);

        cv::Mat tmp1 = up1.clone();
        cv::minMaxIdx(tmp1,&minVal,&maxVal);
        tmp1.convertTo(tmp1,CV_16U, 65535/(maxVal-minVal),-65535*minVal/(maxVal-minVal));
        cv::Mat tmpcolor1;
        cv::cvtColor(tmp1, tmpcolor1, cv::COLOR_GRAY2RGB);

        //get all points on the right eipolar line, give a point in left
        std::vector<cv::Point > lefts;
        std::vector<cv::Scalar> colors;
        lefts.push_back(cv::Point(900, 1000));
        colors.push_back(cv::Scalar(255,255,0));

        lefts.push_back(cv::Point(950, 1050));
        colors.push_back(cv::Scalar(255,0,255));

        lefts.push_back(cv::Point(1100, 800));
        colors.push_back(cv::Scalar(0,255,255));

        lefts.push_back(cv::Point(1200, 900));
        colors.push_back(cv::Scalar(255,0,0));

        lefts.push_back(cv::Point(500, 500));
        colors.push_back(cv::Scalar(0,255,0));

        lefts.push_back(cv::Point(1500, 1040));
        colors.push_back(cv::Scalar(0,0,255));

        lefts.push_back(cv::Point(1300, 1300));
        colors.push_back(cv::Scalar(125,0,0));

        lefts.push_back(cv::Point(1300, 950));
        colors.push_back(cv::Scalar(0,125,0));

        for(int i=0;i<lefts.size();i++)
        {
            cv::Vec3f epiline = epilineMat.at<cv::Vec3f>(lefts[i]); //TODO
            float a,b,c;
            a = epiline[0]; b = epiline[1]; c = epiline[2];
            int x1=1, x2=nCols; //x:cols; y:rows
            int y1 = (int) ( (-c-a*x1)/b + 0.5f );// Casting to an int truncates the value. Adding 0.5 causes it to do proper rounding.
            int y2 = (int) ( round( (-c-a*x2)/b ) );
            cv::Point p1(x1,y1), p2(x2,y2);
            cv::line(tmpcolor1, p1, p2, colors[i],2 ); //cv::Scalar(b,g,r)
            cv::circle(tmpcolor,lefts[i],10,colors[i],2);
        }
        cv::imwrite("am_up0_color.bmp", tmpcolor);   // gray_map is CV_16U using PNG
        cv::imwrite("am_up1_color.bmp", tmpcolor1);   // gray_map is CV_16U using PNG

#endif
    }

    float threshold = 2.0;

    for(ushort i=0; i < nRows; i++) // match intersections
    {
        for(ushort j=0; j<nCols;j++)
        {
            std::vector<intersection> windowsmatched; //collect some roughly matched in one right epiplar Line;
            intersection p0,p1;
            p0.up = up0.at<float>(i,j);
            p0.vp = vp0.at<float>(i,j);
            p0.x = i;
            p0.y = j;

            if(p0.up!=0.0 && p0.vp!=0.0) //masked ignore;
            {
                //get all points on the left eipolar line
                cv::Vec3f epiline = epilineMat.at<cv::Vec3f>(i,j);
                float a,b,c;
                a = epiline[0]; b = epiline[1]; c = epiline[2];
                //y = (-c-a*x) / b;  ax+by+c=0
                ushort boundary = 3; //cut the boundary
                for(ushort x=boundary; x<nCols-boundary;x++) //Note: (x,y) is matlab coordiante=(col,row)
                {
                    float yy = (-c-a*x)/b;
                    ushort y = round(yy); //x:cols; y:rows

                    if(y<boundary || y>nRows-boundary) continue;

                    p1.up = up1.at<float>(y,x);
                    p1.vp = vp1.at<float>(y,x);
                    p1.x = y;
                    p1.y = x;

                    if(p1.up==0.0 || p1.vp==0.0) continue; //masked ignore;

                    float diff_up = fabs(p0.up - p1.up);
                    float diff_vp = fabs(p0.vp - p1.vp);

                    if( diff_up < threshold && diff_vp < threshold)
                    {
                        p1.distance = diff_up + diff_vp;
                        windowsmatched.push_back(p1);

                        // continue search in a 3*3 window, center in w0=p0;
                        //%  7 8 9
                        //%  4 0 6    0: is the center
                        //%  1 2 3
                        intersection w0 = p0;
                        intersection w7(y-1,x-1, up1.at<float>(y-1,x-1), vp1.at<float>(y-1,x-1));
                        intersection w8(y-1,x, up1.at<float>(y-1,x), vp1.at<float>(y-1,x));
                        intersection w9(y-1,x+1, up1.at<float>(y-1,x+1), vp1.at<float>(y-1,x+1));
                        intersection w4(y,x-1, up1.at<float>(y,x-1), vp1.at<float>(y,x-1));
                        intersection w6(y-1,x+1, up1.at<float>(y-1,x+1), vp1.at<float>(y-1,x+1));
                        intersection w1(y+1,x-1, up1.at<float>(y+1,x-1), vp1.at<float>(y+1,x-1));
                        intersection w2(y+1,x, up1.at<float>(y+1,x), vp1.at<float>(y+1,x));
                        intersection w3(y+1,x+1, up1.at<float>(y+1,x+1), vp1.at<float>(y+1,x+1));

                        diff_up = fabs(w7.up - w0.up);
                        diff_vp = fabs(w7.vp - w0.vp);
                        if( diff_up < threshold && diff_vp < threshold)
                        {
                            w7.distance = diff_up + diff_vp;
                            windowsmatched.push_back(w7);
                        }

                        diff_up = fabs(w8.up - w0.up);
                        diff_vp = fabs(w8.vp - w0.vp);
                        if( diff_up < threshold && diff_vp < threshold)
                        {
                            w8.distance = diff_up + diff_vp;
                            windowsmatched.push_back(w8);
                        }

                        diff_up = fabs(w9.up - w0.up);
                        diff_vp = fabs(w9.vp - w0.vp);
                        if( diff_up < threshold && diff_vp < threshold)
                        {
                            w9.distance = diff_up + diff_vp;
                            windowsmatched.push_back(w9);
                        }

                        diff_up = fabs(w4.up - w0.up);
                        diff_vp = fabs(w4.vp - w0.vp);
                        if( diff_up < threshold && diff_vp < threshold)
                        {
                            w4.distance = diff_up + diff_vp;
                            windowsmatched.push_back(w4);
                        }

                        diff_up = fabs(w6.up - w0.up);
                        diff_vp = fabs(w6.vp - w0.vp);
                        if( diff_up < threshold && diff_vp < threshold)
                        {
                            w6.distance = diff_up + diff_vp;
                            windowsmatched.push_back(w6);
                        }

                        diff_up = fabs(w1.up - w0.up);
                        diff_vp = fabs(w1.vp - w0.vp);
                        if( diff_up < threshold && diff_vp < threshold)
                        {
                            w1.distance = diff_up + diff_vp;
                            windowsmatched.push_back(w1);
                        }

                        diff_up = fabs(w2.up - w0.up);
                        diff_vp = fabs(w2.vp - w0.vp);
                        if( diff_up < threshold && diff_vp < threshold)
                        {
                            w2.distance = diff_up + diff_vp;
                            windowsmatched.push_back(w2);
                        }

                        diff_up = fabs(w3.up - w0.up);
                        diff_vp = fabs(w3.vp - w0.vp);
                        if( diff_up < threshold && diff_vp < threshold)
                        {
                            w3.distance = diff_up + diff_vp;
                            windowsmatched.push_back(w3);
                        }
                    }
                }
            }
            //Method1:find the closest one from the windows, currenty use this;
            //Method2: advance: do interpolation;
            if(windowsmatched.size()>0)
            {
                std::sort(windowsmatched.begin(), windowsmatched.end(), sortingLargerDistance); //ascending order
                intersection matched = windowsmatched[0];
                matches1.push_back(matched);
            }
            else
            {
                mask.at<uchar>(i,j) = 0;
                matches1.push_back(p0); //redundant
            }
            matches0.push_back(p0);
        }
    }
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

