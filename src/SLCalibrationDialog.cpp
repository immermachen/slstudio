#include "SLCalibrationDialog.h"
#include "ui_SLCalibrationDialog.h"

#include <QtTest/QTest>
#include <QSettings>

#include <opencv2/opencv.hpp>

#include "Camera.h"
#include "ProjectorOpenGL.h"
#include "ProjectorLC3000.h"
#include "ProjectorLC4500.h"
#include "SLProjectorVirtual.h"

#include "CalibratorLocHom.h"
#include "CalibratorCC.h"
#include "CalibratorRBF.h"

#include "cvtools.h"
#include "SLCameraVirtual.h"

SLCalibrationDialog::SLCalibrationDialog(SLStudio *parent) : QDialog(parent), ui(new Ui::SLCalibrationDialog), reviewMode(false), timerInterval(100), delay(100)
{
    ui->setupUi(this);
    setSizeGripEnabled(false);
    // Release this dialog on close
    this->setAttribute(Qt::WA_DeleteOnClose);
    QSettings settings("SLStudio");
    writeToDisk = settings.value("writeToDisk/frames", false).toBool();
    flip = settings.value("flip",1).toInt();
    //Checkerboard parameters
    float checkerSize = settings.value("calibration/checkerSize",10).toFloat();
    //ui->checkerSizeBox->setValue(checkerSize);
    ui->txtCheckSize->setPlainText(settings.value("calibration/checkerSize",10).toString());
    unsigned int checkerRows = settings.value("calibration/checkerRows",6).toInt();
    ui->checkerRowsBox->setValue(checkerRows);
    unsigned int checkerCols = settings.value("calibration/checkerCols",9).toInt();
    ui->checkerColsBox->setValue(checkerCols);
    // Read trigger configuration
    CameraTriggerMode triggerMode;
    QString sTriggerMode = settings.value("trigger/mode", "Hardware").toString();
    if(sTriggerMode == "hardware")
        triggerMode = triggerModeHardware;
    else if(sTriggerMode == "software")
        triggerMode = triggerModeSoftware;
    else
        std::cerr << "SLCalibrationDialog: invalid trigger mode " << sTriggerMode.toStdString() << std::endl;
    // Create camera
    iNum = settings.value("camera/interfaceNumber", 0).toInt();
    cNum = settings.value("camera/cameraNumber", 0).toInt();
    std::cout<<"InterfaceNumber and cameraNumber: "<< iNum << ", " << cNum<<std::endl;
    if(iNum == 0)
    {
        if(cNum<2)
        {
            camera.push_back(Camera::NewCamera(iNum,cNum,triggerMode));  //only note:cNum
        }
        else
        {
            for(int c=0;c<2;c++)
            {
                camera.push_back(Camera::NewCamera(iNum,c,triggerMode));
            }
        }
    }
    else if(iNum == -1)
    {
        if(cNum<2)
        {
            camera.push_back(new SLCameraVirtual(cNum,triggerMode));
        }
        else
        {
            for(int c=0;c<2;c++)
            {
                camera.push_back(new SLCameraVirtual(c,triggerMode));
            }
        }
    }
    delay = settings.value("trigger/delay", "100").toInt();
    // Set camera settings
    CameraSettings camSettings;
    camSettings.shutter = settings.value("camera/shutter", 16.666).toFloat();
    camSettings.gain = 0.0;
    for(int c=0;c<camera.size();c++)
    {
//        Camera * curCam = camera[c];
        camera[c]->setCameraSettings(camSettings);
        camera[c]->startCapture();
    }

    //------------------------------------- Initialize projector---------------------------------
    int screenNum = settings.value("projector/screenNumber", -1).toInt();
    if(screenNum >= 0)
        //projector = new ProjectorOpenGL(screenNum);
        projector = cv::makePtr<ProjectorOpenGL>(screenNum);
    else if(screenNum == -1 || screenNum == -4)  //  SLStudio Virtual Screen 1024*768" -1,  "SLStudio Virtual Screen 1920*1080", -4);
        projector = cv::makePtr<SLProjectorVirtual>(screenNum);
    else if(screenNum == -2)
        projector = cv::makePtr<ProjectorLC3000>(0);
    else if(screenNum == -3)
        projector = cv::makePtr<ProjectorLC4500>(0);
    else
        std::cerr << "SLCalibrationDialog: invalid projector id " << screenNum << std::endl;
    unsigned int screenResX, screenResY;
    projector->getScreenRes(&screenResX, &screenResY);
    std::cout<< screenResX << "-" << screenResY << std::endl;
    diamondPattern = settings.value("projector/diamondPattern", false).toBool();
    // Unique number of rows and columns
    if(diamondPattern){
        screenCols = 2*screenResX;
        screenRows = screenResY;
    } else {
        screenCols = screenResX;
        screenRows = screenResY;
    }

    if(cNum==3) //for CC
    {
        calibrator[0] = cv::makePtr<CalibratorCC>(screenCols, screenRows);
        connect(calibrator[0], SIGNAL(newSequenceResult(vector< cv::Mat >, unsigned int, vector< bool >)), this, SLOT(onNewSequenceResult(vector< cv::Mat >,uint, vector< bool >)));
        numPatterns = calibrator[0]->getNPatterns();
    }
    else   //for CP
    {
        //Register Metatypes
        qRegisterMetaType<cv::Mat>("cv::Mat");
        qRegisterMetaType<CalibrationData>("CalibrationData");
        if(cNum==0 || cNum == 2)
        {
            calibrator[0] = cv::makePtr<CalibratorLocHom>(screenCols, screenRows);
    //        calThread1 = cv::makePtr<QThread>(this);
    //        calThread1 = new QThread(this);
            calThread1.setObjectName("calThread1");
            calibrator[0]->moveToThread(&calThread1);
            //connect(calThread1, SIGNAL(started()), calibrator[0], SLOT(calibrate()));
            connect(calibrator[0], SIGNAL(signal_calFinished(uint, CalibrationData)), this, SLOT(slot_ReceiveCalData(uint,CalibrationData)));
            connect(this, SIGNAL(signal_Calibrate(uint)), calibrator[0], SLOT(slot_calibrateWrap(uint)));
            connect(calibrator[0], SIGNAL(finished()), &calThread1, SLOT(quit()));
            connect(calibrator[0], SIGNAL(finished()), calibrator[0], SLOT(deleteLater()));
            connect(&calThread1, SIGNAL(finished()), &calThread1, SLOT(deleteLater()));
            connect(calibrator[0], SIGNAL(newSequenceResult(cv::Mat, unsigned int, bool)), this, SLOT(onNewSequenceResult(cv::Mat,uint,bool)));
            calThread1.start();

            numPatterns = calibrator[0]->getNPatterns();
        }
        if(cNum==1  || cNum == 2)
        {
            calibrator[1] = cv::makePtr<CalibratorLocHom>(screenCols, screenRows);
    //        calThread2 = cv::makePtr<QThread>(this);
    //        calThread2 = new QThread(this);
            calThread2.setObjectName("calThread2");
            calibrator[1]->moveToThread(&calThread2);
            connect(calibrator[1], SIGNAL(signal_calFinished(uint, CalibrationData)), this, SLOT(slot_ReceiveCalData(uint,CalibrationData)));
            connect(this, SIGNAL(signal_Calibrate(uint)), calibrator[1], SLOT(slot_calibrateWrap(uint)));
            connect(calibrator[1], SIGNAL(finished()), &calThread2, SLOT(quit()));
            connect(calibrator[1], SIGNAL(finished()), calibrator[1], SLOT(deleteLater()));
            connect(&calThread2, SIGNAL(finished()), &calThread2, SLOT(deleteLater()));
            connect(calibrator[1], SIGNAL(newSequenceResult(cv::Mat, unsigned int, bool)), this, SLOT(onNewSequenceResult2(cv::Mat,uint,bool)));
            calThread2.start();

            numPatterns = calibrator[1]->getNPatterns();
        }
    }

    // Upload patterns to projector/GPU
    for(unsigned int i=0; i < numPatterns; i++)
    {
        cv::Mat pattern;
        if(cNum==0 || cNum==2 || cNum==3)
            pattern = calibrator[0]->getCalibrationPattern(i);
        if(cNum==1)
            pattern = calibrator[1]->getCalibrationPattern(i);

        // general repmat
        pattern = cv::repeat(pattern, screenRows/pattern.rows + 1, screenCols/pattern.cols + 1);
        pattern = pattern(cv::Range(0, screenRows), cv::Range(0, screenCols));

        if(diamondPattern)
            pattern = cvtools::diamondDownsample(pattern);

        projector->setPattern(i, pattern.ptr(), pattern.cols, pattern.rows);

#if 0
        QString filename = QString("PatternSeq_cal_%1_%2.bmp").arg(8, 2, 10, QChar('0')).arg(i, 2, 10, QChar('0'));
        cv::imwrite(filename.toStdString(), pattern);
#endif

    }

    // Start live view
    timerInterval = delay + camSettings.shutter;
    liveViewTimer = startTimer(timerInterval);

}

void SLCalibrationDialog::timerEvent(QTimerEvent *event)
{

    if(event->timerId() != liveViewTimer){
        std::cerr << "Something fishy..." << std::endl << std::flush;
        return;
    }

    QApplication::processEvents();

    CameraFrame frame, frame2;
    if(iNum==0)
    {
        if(cNum<2)
        {
            frame = camera[0]->getFrame();
            cv::Mat frameCV(frame.height, frame.width, CV_8UC1, frame.memory);
//            frameCV = frameCV.clone();
            if(cNum==0)
                ui->videoWidget->showFrameCV(frameCV);
            else
            {
                if(flip==1)
                    cv::flip(frameCV,frameCV,-1);

                ui->videoWidget2->showFrameCV(frameCV);
            }
        }
        else
        {
            frame = camera[0]->getFrame();
            cv::Mat frameCV(frame.height, frame.width, CV_8UC1, frame.memory);
//            frameCV = frameCV.clone();

            frame2 = camera[1]->getFrame();
            cv::Mat frameCV2(frame2.height, frame2.width, CV_8UC1, frame2.memory);
//            frameCV2 = frameCV2.clone();
            //0: flip aroud x-axis;1: flip around Y-axis; -1:flip both directions;
            if(flip==1)
                cv::flip(frameCV2,frameCV2,-1);

            ui->videoWidget->showFrameCV(frameCV);
            ui->videoWidget2->showFrameCV(frameCV2);
        }
    }
    else if(iNum == -1)
    {}

    delete frame.memory;
    delete frame2.memory;
    QApplication::processEvents();
}


SLCalibrationDialog::~SLCalibrationDialog()
{
    if(cNum==0  || cNum == 2)
    {
        calThread1.quit();
        calThread1.wait();
        calibrator[0]->deleteLater();
        calThread1.deleteLater();
    }
    if(cNum==1  || cNum == 2)
    {
        calThread2.quit();
        calThread2.wait();
        calibrator[1]->deleteLater();;
        calThread2.deleteLater();
    }

    for(int c=0; c<camera.size();c++)
        delete camera[c];  //TODO this can be used by cv::Ptr.
//    delete projector;    //for cv::Ptr do not need anymore!!!


//    delete calThread1;
//    delete calThread2;

    this->deleteLater();


//    delete ui;
}


void SLCalibrationDialog::on_snapButton_clicked()
{
    // If in review mode
    if(reviewMode){
        reviewMode = false;
        ui->listWidget->clearSelection();
        ui->listWidget2->clearSelection();
        liveViewTimer = startTimer(timerInterval);
        ui->snapButton->setText("Snap");
        return;
    }
    ui->snapButton->setEnabled(false);    
    killTimer(liveViewTimer);// Stop live view

    vector<std::string> frameSeq[2];

    int numPat = numPatterns;
    if(cNum == 3)
        numPat = 1;

    for(unsigned int i=0; i<numPat; i++)
    {
        // Project pattern
        if(cNum == 3)
        {
            //projector->displayPattern(numPatterns-1); //only use full white pattern
        }
        else
            projector->displayPattern(i);

        QTest::qSleep(delay);

        // Effectuate sleep (necessary with some camera implementations)
        QApplication::processEvents();

        CameraFrame frame, frame2;

        if(cNum == 0 || cNum == 2 || cNum == 3)
        {
            int numCam = 0;
            int numSeqs = frameSeqs[0].size();
            QString filename = QString("dataForCalTest/%1_%2_%3.bmp").arg(numSeqs,2, 10, QChar('0')).arg(numCam, 1).arg(i, 2, 10, QChar('0'));
            //QString filename = QString("dataForCalTest/%1_%2_60.bmp").arg(numSeqs,2, 10, QChar('0')).arg(numCam, 1);
            cv::Mat curframeCV;

            if(iNum==0)
            {
                frame = camera[0]->getFrame();
                cv::Mat curframe(frame.height, frame.width, CV_8U, frame.memory);
                curframeCV = curframe;
//                curframeCV = curframeCV.clone();
            }
            else if(iNum == -1)
            {
                curframeCV = cv::imread(filename.toStdString().c_str(), CV_LOAD_IMAGE_GRAYSCALE);
//                curframeCV = curframeCV.clone();
            }


            QString datacal = filename;
            if(writeToDisk)
            {
                datacal = QString("dataCal/%1_%2_%3.bmp").arg(numSeqs,2, 10, QChar('0')).arg(numCam, 1).arg(i, 2, 10, QChar('0'));
                cv::imwrite(datacal.toStdString(), curframeCV);
            }

            frameSeq[0].push_back(datacal.toStdString());
            ui->videoWidget->showFrameCV(curframeCV);
        }

        if(cNum == 1 || cNum == 2 || cNum == 3)
        {
            int numCam = 1;
            int numSeqs = frameSeqs[1].size();
            QString filename = QString("dataForCalTest/%1_%2_%3.bmp").arg(numSeqs,2, 10, QChar('0')).arg(numCam, 1).arg(i, 2, 10, QChar('0'));
            //QString filename = QString("dataForCalTest/%1_%2_60.bmp").arg(numSeqs,2, 10, QChar('0')).arg(numCam, 1);
            cv::Mat curframeCV;

            if(iNum==0)
            {
                unsigned int index = cNum < 2 ? 0 : 1;
                frame2 = camera[index]->getFrame();
                cv::Mat curframe(frame2.height, frame2.width, CV_8U, frame2.memory);
                curframeCV = curframe;
//                curframeCV = curframeCV.clone();
                if(flip)
                    cv::flip(curframeCV,curframeCV,-1);
            }
            else if(iNum == -1)
            {
                curframeCV = cv::imread(filename.toStdString().c_str(), CV_LOAD_IMAGE_GRAYSCALE);
//                curframeCV = curframeCV.clone();

                //0: flip aroud x-axis;1: flip around Y-axis; -1:flip both directions;
//                cv::flip(curframeCV,curframeCV,-1);
            }


            QString datacal = filename;
            if(writeToDisk)
            {
                datacal = QString("dataCal/%1_%2_%3.bmp").arg(numSeqs,2, 10, QChar('0')).arg(numCam, 1).arg(i, 2, 10, QChar('0'));
                cv::imwrite(datacal.toStdString(), curframeCV);
            }

            frameSeq[1].push_back(datacal.toStdString());
            ui->videoWidget2->showFrameCV(curframeCV);
        }

        delete frame.memory;
        delete frame2.memory;
    }

    if(cNum == 0 || cNum == 2 || cNum == 3)
    {
        // Store frame sequence
        frameSeqs[0].push_back(frameSeq[0]);

        // Add identifier to list
        QListWidgetItem* item = new QListWidgetItem(QString("Sequence %1").arg(frameSeqs[0].size()), ui->listWidget);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable); // set checkable flag
        item->setCheckState(Qt::Checked); // AND initialize check state
    }
    if(cNum == 1 || cNum == 2 || cNum == 3)
    {
        frameSeqs[1].push_back(frameSeq[1]);

        // Add identifier to list
        QListWidgetItem* item2 = new QListWidgetItem(QString("Sequence %1").arg(frameSeqs[1].size()), ui->listWidget2);
        item2->setFlags(item2->flags() | Qt::ItemIsUserCheckable); // set checkable flag
        item2->setCheckState(Qt::Checked); // AND initialize check state
    }

    //    // Allow calibration if enough frame pairs
    //    if(ui->listWidget->count() >= 3)
    ui->calibrateButton->setEnabled(true);

    // Display white
    projector->displayWhite();

    // Restart live view
    liveViewTimer = startTimer(timerInterval);

    ui->snapButton->setEnabled(true);

}


void SLCalibrationDialog::on_calibrateButton_clicked()
{

    // Disable interface elements
    ui->calibrateButton->setEnabled(false);
    ui->listWidget->setEnabled(false);
    ui->listWidget2->setEnabled(false);
    numCalCount=0;

    // Stop live view
    killTimer(liveViewTimer);
    reviewMode = true;
    ui->snapButton->setText("Live View");

    if(cNum == 0 || cNum == 2)
    {
        std::cout<< "----------------Calibrate Camear1-----------------------"<<std::endl;

        calibrator[0]->reset();
        // Note which frame sequences are used
        activeFrameSeqs[0].clear();
        for(int i=0; i<ui->listWidget->count(); i++){
            if(ui->listWidget->item(i)->checkState() == Qt::Checked){
                //vector<cv::Mat> frameSeq(frameSeqs[0][i].begin(), frameSeqs[0][i].begin() + calibrator[0]->getNPatterns());
                vector<std::string> frameSeq(frameSeqs[0][i].begin(), frameSeqs[0][i].begin() + numPatterns);
                calibrator[0]->addFrameSequence(frameSeq);
                activeFrameSeqs[0].push_back(i);
            }
        }
        // Perform calibration
        //calib[0] = calibrator[0]->calibrate();
//        emit(signal_Calibrate(0));
        QMetaObject::invokeMethod(calibrator[0], "slot_calibrateWrap", Q_ARG(uint,0));//, Q_ARG(int,1));

    }

    if(cNum==1  || cNum == 2)
    {
        std::cout<< "----------------Calibrate Camear2-----------------------"<<std::endl;
        calibrator[1]->reset();
        // Note which frame sequences are used
        activeFrameSeqs[1].clear();
        for(int i=0; i<ui->listWidget2->count(); i++){
            if(ui->listWidget2->item(i)->checkState() == Qt::Checked){
                //vector<cv::Mat> frameSeq(frameSeqs[1][i].begin(), frameSeqs[1][i].begin() + calibrator[0]->getNPatterns());
                vector<std::string> frameSeq(frameSeqs[1][i].begin(), frameSeqs[1][i].begin() + numPatterns);
                calibrator[1]->addFrameSequence(frameSeq);
                activeFrameSeqs[1].push_back(i);
            }
        }
        // Perform calibration
        //calib[1] = calibrator[1]->calibrate();
//        emit(signal_Calibrate(1));
        QMetaObject::invokeMethod(calibrator[1], "slot_calibrateWrap", Q_ARG(uint,1));//, Q_ARG(int,1));
    }

    if(cNum == 3)
    {
        std::cout<< "----------------Calibrate CC -----------------------"<<std::endl;

        calibrator[0]->reset();
        // Note which frame sequences are used
        activeFrameSeqs[0].clear();
        for(int i=0; i<ui->listWidget->count(); i++)
        {
            if(ui->listWidget->item(i)->checkState() == Qt::Checked)
            {
                //vector<cv::Mat> frameSeq(frameSeqs[0][i].begin(), frameSeqs[0][i].begin() + calibrator[0]->getNPatterns());
                vector<std::string> frameSeq;
                frameSeq.push_back(frameSeqs[0][i][0]);
                frameSeq.push_back(frameSeqs[1][i][0]);
                calibrator[0]->addFrameSequence(frameSeq);
                activeFrameSeqs[0].push_back(i);
            }
        }
        // Perform calibration
        calib[0] = calibrator[0]->calibrate();

        // Re-enable interface elements
        ui->calibrateButton->setEnabled(true);
        ui->listWidget->setEnabled(true);
        ui->listWidget2->setEnabled(true);
        ui->saveButton->setEnabled(true);

        std::cout<< "----------------Calibrate CC Finished! -----------------------"<<std::endl;
    }
    // Re-enable interface elements
    //TODO
}

void SLCalibrationDialog::slot_ReceiveCalData(uint numCam, CalibrationData calData)
{
    numCalCount++;
    std::cout << "slot_ReceiveCalData: numCam = " << numCalCount << std::endl;
    calib[numCam] = calData;
    if(numCalCount == camera.size())
    {
        // Re-enable interface elements
        ui->calibrateButton->setEnabled(true);
        ui->listWidget->setEnabled(true);
        ui->listWidget2->setEnabled(true);
        ui->saveButton->setEnabled(true);
    }
}


void SLCalibrationDialog::on_listWidget_itemSelectionChanged()
{
    // If selection was cleared
    if(ui->listWidget->selectedItems().isEmpty())
        return;

    // Stop live view
    killTimer(liveViewTimer);
    reviewMode = true;
    ui->snapButton->setText("Live View");

    int currentRow = ui->listWidget->currentRow();

    cv::Mat curFrame = cv::imread(frameSeqs[0][currentRow].back(), CV_LOAD_IMAGE_GRAYSCALE);
    ui->videoWidget->showFrameCV(curFrame);
}

void SLCalibrationDialog::on_listWidget2_itemSelectionChanged()
{
    // If selection was cleared
    if(ui->listWidget2->selectedItems().isEmpty())
        return;

    // Stop live view
    killTimer(liveViewTimer);
    reviewMode = true;
    ui->snapButton->setText("Live View2");

    int currentRow = ui->listWidget2->currentRow();

    cv::Mat curFrame = cv::imread(frameSeqs[1][currentRow].back(), CV_LOAD_IMAGE_GRAYSCALE);
    ui->videoWidget2->showFrameCV(curFrame);
}

void SLCalibrationDialog::on_saveButton_clicked()
{

    unsigned int screenResX, screenResY;
    projector->getScreenRes(&screenResX, &screenResY);

    if(cNum<2)
    {
        calib[cNum].frameWidth = camera[0]->getFrameWidth();
        calib[cNum].frameHeight = camera[0]->getFrameHeight();
        calib[cNum].screenResX = screenResX;
        calib[cNum].screenResY = screenResY;
        calib[cNum].calibrationDateTime = QDateTime::currentDateTime().toString("DD.MM.YYYY HH:MM:SS").toStdString();

        //QString filename = QString("frameSeq_cal_%1_%2.bmp").arg(seqNum, 2, 10, QChar('0')).arg(i, 2, 10, QChar('0'));
        //arg(int a, int fieldWidth = 0, int base = 10, QChar fillChar = QLatin1Char( ' ' )) const
        QString calFilename = QString("calibration_%1.xml").arg(cNum,1);
        calib[cNum].save(calFilename);
    }
    else if(cNum==2)
    {
        for(int c=0; c<2; c++)
        {
            calib[c].frameWidth = camera[c]->getFrameWidth();
            calib[c].frameHeight = camera[c]->getFrameHeight();
            calib[c].screenResX = screenResX;
            calib[c].screenResY = screenResY;
            calib[c].calibrationDateTime = QDateTime::currentDateTime().toString("DD.MM.YYYY HH:MM:SS").toStdString();

            //QString filename = QString("frameSeq_cal_%1_%2.bmp").arg(seqNum, 2, 10, QChar('0')).arg(i, 2, 10, QChar('0'));
            //arg(int a, int fieldWidth = 0, int base = 10, QChar fillChar = QLatin1Char( ' ' )) const
            QString calFilename = QString("calibration_%1.xml").arg(c,1);
            calib[c].save(calFilename);
        }
    }
    else if(cNum==3)
    {
        calib[0].frameWidth = camera[0]->getFrameWidth();
        calib[0].frameHeight = camera[0]->getFrameHeight();
        calib[0].screenResX = screenResX;
        calib[0].screenResY = screenResY;
        calib[0].calibrationDateTime = QDateTime::currentDateTime().toString("DD.MM.YYYY HH:MM:SS").toStdString();

        //QString filename = QString("frameSeq_cal_%1_%2.bmp").arg(seqNum, 2, 10, QChar('0')).arg(i, 2, 10, QChar('0'));
        //arg(int a, int fieldWidth = 0, int base = 10, QChar fillChar = QLatin1Char( ' ' )) const        
        QString calFilename = QString("calibration_%1.xml").arg(cNum,1);
        calib[0].save(calFilename);
        QString calFile = QString("calibration_%1.txt").arg(cNum,1); //
        calib[0].save(calFile);
    }

    this->close();
}

void SLCalibrationDialog::onNewSequenceResult(cv::Mat img, unsigned int idx, bool success)
{

    // Skip non-active frame sequences
    int idxListView = activeFrameSeqs[0][idx];

    // Append calibration result to frame sequence
    unsigned int N = calibrator[0]->getNPatterns();

    int numCam = 0;
    QString filename = QString("dataCal/%1_%2_%3.bmp").arg(idxListView,2, 10, QChar('0')).arg(numCam, 1).arg(N, 2, 10, QChar('0'));
    cv::imwrite(filename.toStdString(), img);

    if(frameSeqs[0][idxListView].size() == N)
        frameSeqs[0][idxListView].push_back(filename.toStdString());
    else
        frameSeqs[0][idxListView][N] = filename.toStdString();

    if(!success) // uncheck
        ui->listWidget->item(idxListView)->setCheckState(Qt::Unchecked);

    // Highlight
    ui->listWidget->setCurrentRow(idxListView);
    ui->listWidget->setFocus();

    QApplication::processEvents();
}


void SLCalibrationDialog::onNewSequenceResult2(cv::Mat img, unsigned int idx, bool success)
{
    // Skip non-active frame sequences
    int idxListView = activeFrameSeqs[1][idx];

    // Append calibration result to frame sequence
    unsigned int N = calibrator[1]->getNPatterns();

    int numCam = 1;
    QString filename = QString("dataCal/%1_%2_%3.bmp").arg(idxListView,2, 10, QChar('0')).arg(numCam, 1).arg(N, 2, 10, QChar('0'));
    cv::imwrite(filename.toStdString(), img);

    if(frameSeqs[1][idxListView].size() == N)
        frameSeqs[1][idxListView].push_back(filename.toStdString());
    else
        frameSeqs[1][idxListView][N] = filename.toStdString();

    if(!success) // uncheck
        ui->listWidget2->item(idxListView)->setCheckState(Qt::Unchecked);

    // Highlight
    ui->listWidget2->setCurrentRow(idxListView);
    ui->listWidget2->setFocus();

    QApplication::processEvents();
}

void SLCalibrationDialog::onNewSequenceResult(std::vector< cv::Mat > img, unsigned int idx, std::vector< bool > success)
{
    // Skip non-active frame sequences
    int idxListView = activeFrameSeqs[0][idx];

    int numCam = 0, numCam1 = 1;
    QString filename = QString("dataCal/%1_%2.bmp").arg(idxListView,2, 10, QChar('0')).arg(numCam, 1);
    cv::imwrite(filename.toStdString(), img[0]);

    QString filename1 = QString("dataCal/%1_%2.bmp").arg(idxListView,2, 10, QChar('0')).arg(numCam1, 1);
    cv::imwrite(filename1.toStdString(), img[1]);

    frameSeqs[0][idxListView].push_back(filename.toStdString());
    frameSeqs[1][idxListView].push_back(filename.toStdString());

    if(!success[0]) // uncheck
        ui->listWidget->item(idxListView)->setCheckState(Qt::Unchecked);
    if(!success[1]) // uncheck
        ui->listWidget2->item(idxListView)->setCheckState(Qt::Unchecked);

    // Highlight
    ui->listWidget->setCurrentRow(idxListView);
    ui->listWidget->setFocus();
    // Highlight
    ui->listWidget2->setCurrentRow(idxListView);
    ui->listWidget2->setFocus();

    QApplication::processEvents();
}


void SLCalibrationDialog::closeEvent(QCloseEvent *)
{
    // Stop live view
    killTimer(liveViewTimer);

    // Save calibration settings
    QSettings settings("SLStudio");
    float checkerSize = ui->txtCheckSize->toPlainText().toFloat();//  ui->checkerSizeBox->value();
    settings.setValue("calibration/checkerSize", checkerSize);
    unsigned int checkerRows = ui->checkerRowsBox->value();
    settings.setValue("calibration/checkerRows", checkerRows);
    unsigned int checkerCols = ui->checkerColsBox->value();
    settings.setValue("calibration/checkerCols", checkerCols);

}
