#include "SLStudio.h"
#include "ui_SLStudio.h"

#include <stdio.h>
#include <string.h>
#include <QThread>
#include <QFileDialog>

#include "SLCalibrationDialog.h"
#include "SLPreferenceDialog.h"
#include "SLAboutDialog.h"

#include "SLVideoWidget.h"

#include <QtGui>

#include "cvtools.h"

using namespace std;

SLStudio::SLStudio(QWidget *parent) : QMainWindow(parent), ui(new Ui::SLStudio),
        scanWorkerThread(NULL), settings(NULL),
        histogramDialog(NULL), shadingDialog(NULL), decoderUpDialog(NULL), decoderVpDialog(NULL), trackerDialog(NULL){

    ui->setupUi(this);

    time = new QTime;

    // Restore main window geometry
    settings = new QSettings("SLStudio");
    restoreGeometry(settings->value("geometry/mainwindow").toByteArray());
    restoreState(settings->value("state/mainwindow").toByteArray());

    // Ui connections
    connect(ui->pointCloudWidget, SIGNAL(newPointCloudDisplayed()), this, SLOT(updateDisplayRate()));

    // Create video dialogs
    histogramDialog = new SLVideoDialog("Histogram", this);
    shadingDialog  = new SLVideoDialog("Shading", this);
    decoderUpDialog = new SLVideoDialog("Decoder Up", this);
    decoderVpDialog = new SLVideoDialog("Decoder Vp", this);

    // Add view menu actions
    ui->menuView->addAction(histogramDialog->toggleViewAction());
    ui->menuView->addAction(shadingDialog->toggleViewAction());
    ui->menuView->addAction(decoderUpDialog->toggleViewAction());
    ui->menuView->addAction(decoderVpDialog->toggleViewAction());

    // Restore Geometry
    histogramDialog->restoreGeometry(settings->value("geometry/histogram").toByteArray());
    shadingDialog->restoreGeometry(settings->value("geometry/shading").toByteArray());
    decoderUpDialog->restoreGeometry(settings->value("geometry/decoderUp").toByteArray());
    decoderVpDialog->restoreGeometry(settings->value("geometry/decoderVp").toByteArray());

    // Restore Visibility
    histogramDialog->setVisible(settings->value("visible/histogram", false).toBool());
    shadingDialog->setVisible(settings->value("visible/shading", false).toBool());
    decoderUpDialog->setVisible(settings->value("visible/decoderUp", false).toBool());
    decoderVpDialog->setVisible(settings->value("visible/decoderVp", false).toBool());

    // Tracker Dialog
    trackerDialog = new SLTrackerDialog(this);
    ui->menuView->addAction(trackerDialog->toggleViewAction());
    trackerDialog->setVisible(settings->value("visible/trackerDialog", false).toBool());

}

void SLStudio::onShowHistogram(cv::Mat im){
    if(histogramDialog->isVisible())
        histogramDialog->showImageCV(im);
}

void SLStudio::onShowShading(cv::Mat im){
    if(shadingDialog->isVisible())
        shadingDialog->showImageCV(im);
}

void SLStudio::onShowDecoderUp(cv::Mat im){
    if(decoderUpDialog->isVisible())
        decoderUpDialog->showImageCV(im);
}

void SLStudio::onShowDecoderVp(cv::Mat im){
    if(decoderVpDialog->isVisible()){
        decoderVpDialog->showImageCV(im);
        //std::cout << "Showing now!" << std::endl;
    }
}

void SLStudio::onActionStart(){

    unsigned int iNum = settings->value("camera/interfaceNumber", 0).toInt();
    unsigned int cNum = settings->value("camera/cameraNumber", 0).toInt();

//    std::cout<< "onActionStart ---->  cNum = " << cNum << std::endl;

    // Register metatypes
    qRegisterMetaType<cv::Mat>("cv::Mat");
    qRegisterMetaType< std::vector<cv::Mat> >("std::vector<cv::Mat>");
    //qRegisterMetaType< PointCloudConstPtr >("PointCloudConstPtr");

    //QObject::connect: Cannot queue arguments of type 'PointCloudPtr'
    //(Make sure 'PointCloudPtr' is registered using qRegisterMetaType().)
    qRegisterMetaType< PointCloudPtr >("PointCloudPtr");

    // Prepare scanWorker on separate thread
    scanWorker = new SLScanWorker(this);
    scanWorkerThread = new QThread(this);
    scanWorkerThread->setObjectName("scanWorkerThread");
    scanWorker->moveToThread(scanWorkerThread);    
    connect(scanWorker, SIGNAL(showHistogram(cv::Mat)), this, SLOT(onShowHistogram(cv::Mat)));

    //https://mayaposch.wordpress.com/2011/11/01/how-to-really-truly-use-qthreads-the-full-explanation/
    connect(scanWorker, SIGNAL(finished()), this, SLOT(onScanWorkerFinished()));
    connect(scanWorker, SIGNAL(finished()), scanWorkerThread, SLOT(quit()));
    connect(scanWorker, SIGNAL(finished()), scanWorker, SLOT(deleteLater()));
    connect(scanWorkerThread, SIGNAL(finished()), scanWorkerThread, SLOT(deleteLater()));

    if(cNum == 0 || cNum == 2)
    {
        decoderWorker = new SLDecoderWorker(0);
        decoderThread = new QThread(this);
        decoderThread->setObjectName("decoderThread");
        decoderWorker->moveToThread(decoderThread);
        connect(decoderThread, SIGNAL(started()), decoderWorker, SLOT(setup()));
        connect(decoderWorker, SIGNAL(finished()), decoderThread, SLOT(quit()));
        connect(decoderWorker, SIGNAL(finished()), decoderWorker, SLOT(deleteLater()));
        connect(decoderThread, SIGNAL(finished()), decoderThread, SLOT(deleteLater()));

        triangulatorWorker = new SLTriangulatorWorker(0);
        triangulatorThread = new QThread(this);
        triangulatorThread->setObjectName("triangulatorThread");
        triangulatorWorker->moveToThread(triangulatorThread);
        connect(triangulatorThread, SIGNAL(started()), triangulatorWorker, SLOT(setup()));
        connect(triangulatorWorker, SIGNAL(finished()), triangulatorThread, SLOT(quit()));
        connect(triangulatorWorker, SIGNAL(finished()), triangulatorWorker, SLOT(deleteLater()));
        connect(triangulatorThread, SIGNAL(finished()), triangulatorThread, SLOT(deleteLater()));

        connect(scanWorker, SIGNAL(newFrameSeq(std::vector<cv::Mat>)), decoderWorker, SLOT(decodeSequence(std::vector<cv::Mat>)));
        connect(decoderWorker, SIGNAL(showShading(cv::Mat)), this, SLOT(onShowShading(cv::Mat)));
        connect(decoderWorker, SIGNAL(showDecoderUp(cv::Mat)), this, SLOT(onShowDecoderUp(cv::Mat)));
        connect(decoderWorker, SIGNAL(showDecoderVp(cv::Mat)), this, SLOT(onShowDecoderVp(cv::Mat)));
        connect(decoderWorker, SIGNAL(newUpVp(cv::Mat,cv::Mat,cv::Mat,cv::Mat)), triangulatorWorker, SLOT(triangulatePointCloud(cv::Mat,cv::Mat,cv::Mat,cv::Mat)));
        connect(triangulatorWorker, SIGNAL(newPointCloud(PointCloudPtr, unsigned int)), this, SLOT( receiveNewPointCloud(PointCloudPtr, unsigned int) ) );
        connect(triangulatorWorker, SIGNAL(imshow(const char*,cv::Mat,uint,uint)), this, SLOT(imshow(const char*,cv::Mat,uint,uint)));

        decoderThread->start(QThread::LowPriority);
        triangulatorThread->start(QThread::LowPriority);
    }

    if(cNum == 1 | cNum == 2)
    {
        decoderWorker2 = new SLDecoderWorker(1);
        decoderThread2 = new QThread(this);
        decoderThread2->setObjectName("decoderThread2");
        decoderWorker2->moveToThread(decoderThread2);
        connect(decoderThread2, SIGNAL(started()), decoderWorker2, SLOT(setup()));
        connect(decoderWorker2, SIGNAL(finished()), decoderThread2, SLOT(quit()));
        connect(decoderWorker2, SIGNAL(finished()), decoderWorker2, SLOT(deleteLater()));
        connect(decoderThread2, SIGNAL(finished()), decoderThread2, SLOT(deleteLater()));

        triangulatorWorker2 = new SLTriangulatorWorker(1);
        triangulatorThread2 = new QThread(this);
        triangulatorThread2->setObjectName("triangulatorThread2");
        triangulatorWorker2->moveToThread(triangulatorThread2);
        connect(triangulatorThread2, SIGNAL(started()), triangulatorWorker2, SLOT(setup()));
        connect(triangulatorWorker2, SIGNAL(finished()), triangulatorThread2, SLOT(quit()));
        connect(triangulatorWorker2, SIGNAL(finished()), triangulatorWorker2, SLOT(deleteLater()));
        connect(triangulatorThread2, SIGNAL(finished()), triangulatorThread2, SLOT(deleteLater()));

        connect(scanWorker, SIGNAL(newFrameSeq2(std::vector<cv::Mat>)), decoderWorker2, SLOT(decodeSequence(std::vector<cv::Mat>)));
        connect(decoderWorker2, SIGNAL(showShading(cv::Mat)), this, SLOT(onShowShading(cv::Mat)));
        connect(decoderWorker2, SIGNAL(showDecoderUp(cv::Mat)), this, SLOT(onShowDecoderUp(cv::Mat)));
        connect(decoderWorker2, SIGNAL(showDecoderVp(cv::Mat)), this, SLOT(onShowDecoderVp(cv::Mat)));
        connect(decoderWorker2, SIGNAL(newUpVp(cv::Mat,cv::Mat,cv::Mat,cv::Mat)), triangulatorWorker2, SLOT(triangulatePointCloud(cv::Mat,cv::Mat,cv::Mat,cv::Mat)));
        connect(triangulatorWorker2, SIGNAL(newPointCloud(PointCloudPtr, unsigned int)), this, SLOT(receiveNewPointCloud(PointCloudPtr, unsigned int)));
        connect(triangulatorWorker2, SIGNAL(imshow(const char*,cv::Mat,uint,uint)), this, SLOT(imshow(const char*,cv::Mat,uint,uint)));

        decoderThread2->start(QThread::LowPriority);
        triangulatorThread2->start(QThread::LowPriority);
    }

    if(cNum == 3)
    {
        decoderWorker = new SLDecoderWorker(3); //3: using calibration xml of CC
        decoderThread = new QThread(this);
        decoderThread->setObjectName("decoderThread");
        decoderWorker->moveToThread(decoderThread);
        connect(decoderThread, SIGNAL(started()), decoderWorker, SLOT(setup()));
        connect(decoderWorker, SIGNAL(finished()), decoderThread, SLOT(quit()));
        connect(decoderWorker, SIGNAL(finished()), decoderWorker, SLOT(deleteLater()));
        connect(decoderThread, SIGNAL(finished()), decoderThread, SLOT(deleteLater()));

        triangulatorWorker = new SLTriangulatorWorker(3);
        triangulatorThread = new QThread(this);
        triangulatorThread->setObjectName("triangulatorThread");
        triangulatorWorker->moveToThread(triangulatorThread);
        connect(triangulatorThread, SIGNAL(started()), triangulatorWorker, SLOT(setup()));
        connect(triangulatorWorker, SIGNAL(finished()), triangulatorThread, SLOT(quit()));
        connect(triangulatorWorker, SIGNAL(finished()), triangulatorWorker, SLOT(deleteLater()));
        connect(triangulatorThread, SIGNAL(finished()), triangulatorThread, SLOT(deleteLater()));

        connect(scanWorker, SIGNAL(newFrameSeq(std::vector<cv::Mat>, std::vector<cv::Mat>)), decoderWorker, SLOT(decodeSequence(std::vector<cv::Mat>, std::vector<cv::Mat>)));
        connect(decoderWorker, SIGNAL(showShading(cv::Mat)), this, SLOT(onShowShading(cv::Mat)));
        connect(decoderWorker, SIGNAL(showDecoderUp(cv::Mat)), this, SLOT(onShowDecoderUp(cv::Mat)));
        connect(decoderWorker, SIGNAL(showDecoderVp(cv::Mat)), this, SLOT(onShowDecoderVp(cv::Mat)));
        connect(decoderWorker, SIGNAL(newUpVp(cv::Mat,cv::Mat,cv::Mat,cv::Mat, cv::Mat,cv::Mat,cv::Mat,cv::Mat)), triangulatorWorker, SLOT(triangulatePointCloud(cv::Mat,cv::Mat,cv::Mat,cv::Mat, cv::Mat,cv::Mat,cv::Mat,cv::Mat)));
        connect(triangulatorWorker, SIGNAL(newPointCloud(PointCloudPtr, unsigned int)), this, SLOT( receiveNewPointCloud(PointCloudPtr, unsigned int) ) );
        connect(triangulatorWorker, SIGNAL(imshow(const char*,cv::Mat,uint,uint)), this, SLOT(imshow(const char*,cv::Mat,uint,uint)));

        decoderThread->start(QThread::LowPriority);
        triangulatorThread->start(QThread::LowPriority);
    }

    scanWorkerThread->start(QThread::TimeCriticalPriority);

    // Setup and start processing
    //QMetaObject::invokeMethod(decoderWorker, "setup");//, Q_ARG(int,0));
    //QMetaObject::invokeMethod(triangulatorWorker, "setup");//, Q_ARG(int,0));
    //QMetaObject::invokeMethod(decoderWorker2, "setup");//, Q_ARG(int,1));
    //QMetaObject::invokeMethod(triangulatorWorker2, "setup");//, Q_ARG(int,1));

    QMetaObject::invokeMethod(scanWorker, "setup");
    if(cNum == 3)
        QMetaObject::invokeMethod(scanWorker, "doWork_CC");
    else
        QMetaObject::invokeMethod(scanWorker, "doWork");
    time->start();

    // Change ui elements
    ui->actionStart->setEnabled(false);
    ui->actionStop->setEnabled(true);
    ui->actionTracking->setEnabled(true);
    ui->actionSavePointCloud->setEnabled(true);
    ui->actionSaveScreenshot->setEnabled(true);
    ui->actionCalibration->setEnabled(false);

}

void SLStudio::onActionStop(){
    // Stop processing on scan worker thread
    QMetaObject::invokeMethod(scanWorker, "stopWorking");

    //cv::destroyAllWindows();

//    decoderThread->quit();
//    decoderThread->wait();
//    std::cout<<"decoderThread deleted\n"<<std::flush;

//    triangulatorThread->quit();
//    triangulatorThread->wait();
//    std::cout<<"triangulatorThread deleted\n"<<std::flush;

//    decoderThread2->quit();
//    decoderThread2->wait();
//    std::cout<<"decoderThread2 deleted\n"<<std::flush;

//    triangulatorThread2->quit();
//    triangulatorThread2->wait();
//    std::cout<<"triangulatorThread2 deleted\n"<<std::flush;
}

void SLStudio::onScanWorkerFinished(){
//    QMetaObject::invokeMethod(scanWorker, "deleteLater");
//    // Terminate scan worker thread
//    scanWorkerThread->quit();
//    scanWorkerThread->wait();

    //scanWorkerThread->deleteLater();
    //delete scanWorkerThread;

    // Change ui elements
    ui->actionStart->setEnabled(true);
    ui->actionStop->setEnabled(false);
    ui->actionTracking->setEnabled(false);
    ui->actionCalibration->setEnabled(true);
}

void SLStudio::onActionCalibration(){
    SLCalibrationDialog *calibrationDialog = new SLCalibrationDialog(this);
    calibrationDialog->exec();
}


void SLStudio::onActionPreferences(){
    SLPreferenceDialog *preferenceDialog = new SLPreferenceDialog(this);
    preferenceDialog->exec();
}

void SLStudio::updateDisplayRate(){

    int mSecElapsed = time->restart();
    displayIntervals.push_back(mSecElapsed);

    if(displayIntervals.size() > 10)
        displayIntervals.erase(displayIntervals.begin(), displayIntervals.end()-10);

    float meanMSecElapsed = 0;
    for(unsigned int i=0; i<displayIntervals.size(); i++)
        meanMSecElapsed += displayIntervals[i];

    meanMSecElapsed /= displayIntervals.size();

    QString fpsString = QString("PCPS: %1").arg(1000.0/meanMSecElapsed, 0, 'f', 2);
    ui->statusBar->showMessage(fpsString);

}

void SLStudio::receiveNewPointCloud(PointCloudPtr pointCloud, unsigned int _cNum){
    std::cout<< "Receiving PCL .........!" << std::endl;

    unsigned int iNum = settings->value("camera/interfaceNumber", 0).toInt();
    unsigned int cNum = settings->value("camera/cameraNumber", 0).toInt();

    if(cNum == 2)
    {
//        //add point clouds
//        if(_cNum == 0)
//        {
//            ui->pointCloudWidget->pointCloudPCL_Left = pointCloud;
//            std::cout<< "Add pointCloudPCL_Left!" << std::endl;
//            nPCL++;
//        }
//        else
//        {
//            ui->pointCloudWidget->pointCloudPCL_Right = pointCloud;
//            std::cout<< "Add pointCloudPCL_Right!" << std::endl;
//            nPCL++;
//        }

//        if(nPCL == 2)
//        {
//            std::cout<< "Beginning PCL Registration ....!" << std::endl;
//            if(ui->pointCloudWidget->registerPointCloud())
//            {
//                std::cout<< "PCL Registration OK!" << std::endl;
//                // Display point cloud in widget
//                if(ui->actionUpdatePointClouds->isChecked())
//                    ui->pointCloudWidget->updatePointCloud(pointCloud);
//                std::cout<< "Received PCL Finished From Registration !" << std::endl;
//            }
//            nPCL = 0;
//        }
    }

    if(cNum<2 || cNum==3)
    {
        // Display point cloud in widget
        if(ui->actionUpdatePointClouds->isChecked())
            ui->pointCloudWidget->updatePointCloud(pointCloud);

        if(trackerDialog->isVisible())
            trackerDialog->receiveNewPointCloud(pointCloud);

        std::cout<< "Received PCL Finished From Camera=!"<< cNum << std::endl;
    }    
}

void SLStudio::closeEvent(QCloseEvent *event){

    // Save main window geometry
    settings->setValue("geometry/mainwindow", saveGeometry());
    settings->setValue("state/mainwindow", saveState());

    // Store Geometry
    settings->setValue("geometry/histogram", histogramDialog->saveGeometry());
    settings->setValue("geometry/shading", shadingDialog->saveGeometry());
    settings->setValue("geometry/decoderUp", decoderUpDialog->saveGeometry());
    settings->setValue("geometry/decoderVp", decoderVpDialog->saveGeometry());

    // Store Visibility
    settings->setValue("visible/histogram", histogramDialog->isVisible());
    settings->setValue("visible/shading", shadingDialog->isVisible());
    settings->setValue("visible/decoderUp", decoderUpDialog->isVisible());
    settings->setValue("visible/decoderVp", decoderVpDialog->isVisible());

    // Store data for trackerDialog (temp)
    settings->setValue("geometry/trackerDialog", trackerDialog->saveGeometry());
    settings->setValue("visible/trackerDialog", trackerDialog->isVisible());

    event->accept();
}

SLStudio::~SLStudio(){
    delete ui;
    delete settings;
}

void SLStudio::onActionLoadCalibration(){
    QString fileName = QFileDialog::getOpenFileName(this, "Choose calibration file", QString(), "*.xml");
    if(!(fileName.length() == 0)){
        CalibrationData calibration;
        calibration.load(fileName);
        calibration.save("calibration.xml");
    }
}

void SLStudio::onActionExportCalibration(){
    CalibrationData calibration;
    calibration.load("calibration.xml");
//  Non native file dialog
//    QFileDialog saveFileDialog(this, "Export Calibration", QString(), "*.xml;;*.slcalib;;*.m");
//    saveFileDialog.setDefaultSuffix("xml");
//    saveFileDialog.exec();
//    QString fileName = saveFileDialog.selectedFiles().first();
//  Native file dialog
    QString selectedFilter;
    QString fileName = QFileDialog::getSaveFileName(this, "Export Calibration", QString(), "*.xml;;*.slcalib;;*.m", &selectedFilter);

    if(!(fileName.length() == 0)){
        QFileInfo info(fileName);
        QString type = info.suffix();
        if(type == "")
            fileName.append(selectedFilter.remove(0,1));
        calibration.save(fileName);
    }
}

void SLStudio::onActionAbout(){
    SLAboutDialog *aboutDialog = new SLAboutDialog(this);
    aboutDialog->exec();
}

// Debuggings slots for plotting on the main thread
void SLStudio::hist(const char* windowName, cv::Mat im, unsigned int x, unsigned int y){
    cvtools::hist(windowName, im, x, y);
}
void SLStudio::imshow(const char* windowName, cv::Mat im, unsigned int x, unsigned int y){
    cvtools::imshow(windowName, im, x, y);
}


