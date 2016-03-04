#include "SLTriangulatorWorker.h"

#include <QCoreApplication>
#include <QSettings>

#include <iostream>
#include <opencv2/opencv.hpp>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

void SLTriangulatorWorker::setup(){
    QSettings settings("SLStudio");
    writeToDisk = settings.value("writeToDisk/pointclouds",false).toBool();

    //iNum = settings.value("camera/interfaceNumber", 0).toInt();
    //cNum = settings.value("camera/cameraNumber", 0).toInt();

    // Initialize triangulator with calibration
    calibration = new CalibrationData;
    if(cNum==1)
    {
        std::cout << "SLTriangulatorWorker::setup:: Using Calibration_1.xml" << std::endl;
        calibration->load("calibration_1.xml");
    }
    else if(cNum==0)
    {
        std::cout << "SLTriangulatorWorker::setup:: Using Calibration_0.xml" << std::endl;
        calibration->load("calibration_0.xml");
    }
    else if(cNum==3)
    {
        std::cout << "SLTriangulatorWorker::setup:: Using Calibration_3.xml" << std::endl;
        calibration->load("calibration_3.xml");
    }

    triangulator = new Triangulator(*calibration);
}

void SLTriangulatorWorker::triangulatePointCloud(cv::Mat up, cv::Mat vp, cv::Mat mask, cv::Mat shading)
{
    std::cout<< "SLTriangulatorWorker::triangulatePointCloud(cv::Mat up, cv::Mat vp, cv::Mat mask, cv::Mat shading)" << std::endl;

    // Recursively call self until latest event is hit
    busy = true;
    QCoreApplication::sendPostedEvents(this, QEvent::MetaCall);
    bool result = busy;
    busy = false;
    if(!result){
        std::cerr << "SLTriangulatorWorker: dropped phase image!" << std::endl;
        return;
    }

    time.restart();

    // Reconstruct point cloud
    cv::Mat pointCloud;
    triangulator->triangulate(up, vp, mask, shading, pointCloud);

    // Convert point cloud to PCL format
    PointCloudPtr pointCloudPCL(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Interprete as organized point cloud
    pointCloudPCL->width = pointCloud.cols;
    pointCloudPCL->height = pointCloud.rows;
    pointCloudPCL->is_dense = false;

    pointCloudPCL->points.resize(pointCloud.rows*pointCloud.cols);

    for(int row=0; row<pointCloud.rows; row++){
        int offset = row * pointCloudPCL->width;
        for(int col=0; col<pointCloud.cols; col++){
            const cv::Vec3f pnt = pointCloud.at<cv::Vec3f>(row,col);
            unsigned char shade = shading.at<unsigned char>(row,col);
            pcl::PointXYZRGB point;
            point.x = pnt[0]; point.y = pnt[1]; point.z = pnt[2];
            point.r = shade; point.g = shade; point.b = shade;
            pointCloudPCL->points[offset + col] = point;
        }
    }

//    std::vector<cv::Mat> xyz;
//    cv::split(pointCloud, xyz);

//    // stack xyz data
//    std::vector<cv::Mat> pointCloudChannels;
//    pointCloudChannels.push_back(xyz[0]);
//    pointCloudChannels.push_back(xyz[1]);
//    pointCloudChannels.push_back(xyz[2]);

//    // 4 byte padding
//    pointCloudChannels.push_back(cv::Mat::zeros(pointCloud.size(), CV_32F));

//    // triple uchar color information
//    std::vector<cv::Mat> rgb;
//    rgb.push_back(shading);
//    rgb.push_back(shading);
//    rgb.push_back(shading);
//    rgb.push_back(cv::Mat::zeros(shading.size(), CV_8U));

//    cv::Mat rgb8UC4;
//    cv::merge(rgb, rgb8UC4);

//    cv::Mat rgb32F(rgb8UC4.size(), CV_32F, rgb8UC4.data);

//    pointCloudChannels.push_back(rgb32F);

//    // 12 bytes padding
//    pointCloudChannels.push_back(cv::Mat::zeros(pointCloud.size(), CV_32F));
//    pointCloudChannels.push_back(cv::Mat::zeros(pointCloud.size(), CV_32F));
//    pointCloudChannels.push_back(cv::Mat::zeros(pointCloud.size(), CV_32F));

//    // merge channels
//    cv::Mat pointCloudPadded;
//    cv::merge(pointCloudChannels, pointCloudPadded);

//    // memcpy everything
//    memcpy(&pointCloudPCL->points[0], pointCloudPadded.data, pointCloudPadded.rows*pointCloudPadded.cols*sizeof(pcl::PointXYZRGB));

//    // filtering
//    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> filter;
//    filter.setMeanK(5);
//    filter.setStddevMulThresh(1.0);
//    filter.setInputCloud(pointCloudPCL);
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudFiltered(new pcl::PointCloud<pcl::PointXYZRGB>);
//    filter.filter(*pointCloudFiltered);

    // Emit result
    emit newPointCloud(pointCloudPCL, cNum);

    std::cout << "Triangulator: " << time.elapsed() << "ms" << std::endl;

    if(writeToDisk){
        QString fileName = QString("acam_%1").arg(cNum,1);// = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmsszzz");
        fileName.append(".ply");
        //pcl::io::savePCDFileBinary(fileName.toStdString(), *pointCloudPCL);
        pcl::PLYWriter w;
        // Write to ply in binary without camera
        w.write<pcl::PointXYZRGB> (fileName.toStdString(), *pointCloudPCL, true, false);
        std::cout << "Save PLY: " << fileName.toStdString() << std::endl;
    }

    emit finished();
}


void SLTriangulatorWorker::triangulatePointCloud(cv::Mat up0, cv::Mat vp0, cv::Mat mask0, cv::Mat shading0, cv::Mat up1, cv::Mat vp1, cv::Mat mask1, cv::Mat shading1)
{
    std::cout<< "SLTriangulatorWorker::triangulatePointCloud(cv::Mat up0, cv::Mat vp0, cv::Mat mask0, cv::Mat shading0, cv::Mat up1, cv::Mat vp1, cv::Mat mask1, cv::Mat shading1)" << std::endl;

    // Recursively call self until latest event is hit
    busy = true;
    QCoreApplication::sendPostedEvents(this, QEvent::MetaCall);
    bool result = busy;
    busy = false;
    if(!result){
        std::cerr << "SLTriangulatorWorker: dropped phase image!" << std::endl;
        return;
    }

    time.restart();

    // Reconstruct point cloud
    cv::Mat pointCloud;
    triangulator->triangulate(up0, vp0, mask0, shading0, up1, vp1, mask1, shading1, pointCloud);

    // Convert point cloud to PCL format
    PointCloudPtr pointCloudPCL(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Interprete as organized point cloud
    pointCloudPCL->width = pointCloud.cols;
    pointCloudPCL->height = pointCloud.rows;
    pointCloudPCL->is_dense = false;

    pointCloudPCL->points.resize(pointCloud.rows*pointCloud.cols);

    for(int row=0; row<pointCloud.rows; row++){
        int offset = row * pointCloudPCL->width;
        for(int col=0; col<pointCloud.cols; col++){
            const cv::Vec3f pnt = pointCloud.at<cv::Vec3f>(row,col);

            //unsigned char shade = shading0.at<unsigned char>(row,col);  //[Yang]: TODO, To use which texture?

            pcl::PointXYZRGB point;
            point.x = pnt[0]; point.y = pnt[1]; point.z = pnt[2];
            //point.r = shade; point.g = shade; point.b = shade; //Yang: TODO
            point.r = 255; point.g = 1; point.b = 1;
            pointCloudPCL->points[offset + col] = point;
        }
    }


    // Emit result
    emit newPointCloud(pointCloudPCL, cNum);

    std::cout << "Triangulator: " << time.elapsed() << "ms" << std::endl;

    if(writeToDisk){
        QString fileName = QString("acam_%1").arg(cNum,1);// = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmsszzz");
        fileName.append(".ply");
        //pcl::io::savePCDFileBinary(fileName.toStdString(), *pointCloudPCL);
        pcl::PLYWriter w;
        // Write to ply in binary without camera
        w.write<pcl::PointXYZRGB> (fileName.toStdString(), *pointCloudPCL, true, false);
        std::cout << "Save PLY: " << fileName.toStdString() << std::endl;
    }

    emit finished();
}

SLTriangulatorWorker::~SLTriangulatorWorker(){
    delete calibration;
    delete triangulator;

    std::cout<<"triangulatorWorker deleted\n"<<std::flush;
}
