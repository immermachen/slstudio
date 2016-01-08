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
#include "CalibratorRBF.h"

#include "cvtools.h"
#include "SLCameraVirtual.h"

SLCalibrationDialog::SLCalibrationDialog(SLStudio *parent) : QDialog(parent), ui(new Ui::SLCalibrationDialog), reviewMode(false), timerInterval(100), delay(100) {
    ui->setupUi(this);
    setSizeGripEnabled(false);

    // Release this dialog on close
    this->setAttribute(Qt::WA_DeleteOnClose);

    QSettings settings("SLStudio");

    writeToDisk = settings.value("writeToDisk/frames", false).toBool();

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
            for(int c=0;c<cNum;c++)
            {
                camera.push_back(Camera::NewCamera(iNum,c,triggerMode));  //only note:cNum
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
            for(int c=0;c<cNum;c++)
            {
                camera.push_back(new SLCameraVirtual(c,triggerMode));  //only note:cNum
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
        Camera * curCam = camera[c];
        curCam->setCameraSettings(camSettings);
        curCam->startCapture();
    }

    // Initialize projector
    int screenNum = settings.value("projector/screenNumber", -1).toInt();
    if(screenNum >= 0)
        projector = new ProjectorOpenGL(screenNum);
    else if(screenNum == -1)
        projector = new SLProjectorVirtual(screenNum);
    else if(screenNum == -2)
        projector = new ProjectorLC3000(0);
    else if(screenNum == -3)
        projector = new ProjectorLC4500(0);
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

    // Create calibrator

    calibrator.push_back(new CalibratorLocHom(screenCols, screenRows));
    calibrator.push_back(new CalibratorLocHom(screenCols, screenRows));

    connect(calibrator[0], SIGNAL(newSequenceResult(cv::Mat, unsigned int, bool)), this, SLOT(onNewSequenceResult(cv::Mat,uint,bool)));
    connect(calibrator[1], SIGNAL(newSequenceResult(cv::Mat, unsigned int, bool)), this, SLOT(onNewSequenceResult2(cv::Mat,uint,bool)));

    // Upload patterns to projector/GPU
    for(unsigned int i=0; i<calibrator[0]->getNPatterns(); i++){
        cv::Mat pattern = calibrator[0]->getCalibrationPattern(i);

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

void SLCalibrationDialog::timerEvent(QTimerEvent *event){

    if(event->timerId() != liveViewTimer){
        std::cerr << "Something fishy..." << std::endl << std::flush;
        return;
    }

    QApplication::processEvents();

    if(iNum==0)
    {
        if(cNum<2)
        {
            CameraFrame frame = camera[0]->getFrame();
            cv::Mat frameCV(frame.height, frame.width, CV_8UC1, frame.memory);
            frameCV = frameCV.clone();
            if(cNum==0)
                ui->videoWidget->showFrameCV(frameCV);
            else
                ui->videoWidget2->showFrameCV(frameCV);
        }
        else
        {
            //            CameraFrame frame2 = camera[1]->getFrame();
            //            cv::Mat frameCV2(frame2.height, frame2.width, CV_8UC1, frame2.memory);
            //            frameCV2 = frameCV2.clone();

        }
    }
    else if(iNum == -1)
    {}

    QApplication::processEvents();
}

SLCalibrationDialog::~SLCalibrationDialog(){
    delete ui;
}

void SLCalibrationDialog::on_snapButton_clicked(){

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

    // Stop live view
    killTimer(liveViewTimer);


    vector<cv::Mat> frameSeq[2];

    for(unsigned int i=0; i<calibrator[0]->getNPatterns(); i++)
    {
        // Project pattern
        projector->displayPattern(i);
        QTest::qSleep(delay);

        // Effectuate sleep (necessary with some camera implementations)
        QApplication::processEvents();

        vector< cv::Mat > frameCV;
        for(int c=0;c<camera.size();c++)
        {
            CameraFrame frame = camera[c]->getFrame();
            cv::Mat curframeCV(frame.height, frame.width, CV_8U, frame.memory);
//            std::stringstream oss;
//            oss << "data/aCam"<<c+1<<"-1/Capture-" << i <<".bmp";
//            cv::Mat curframeCV = cv::imread(oss.str(), CV_LOAD_IMAGE_GRAYSCALE);
            curframeCV = curframeCV.clone();

            frameCV.push_back(curframeCV);

            if(writeToDisk)
            {
                int numSeqs = frameSeqs[0].size();
                QString filename = QString("../../caldata/%1_%2_%3.bmp").arg(numSeqs,2, 10, QChar('0')).arg(cNum, 1).arg(i, 2, 10, QChar('0'));
                cv::imwrite(filename.toStdString(), curframeCV);
            }
        }

        // Show frame
        if(iNum==0 && cNum<2)
        {
            if(cNum == 0)
                ui->videoWidget->showFrameCV(frameCV[0]);
            else
                ui->videoWidget2->showFrameCV(frameCV[0]);
        }
        // Save frame
        frameSeq[cNum].push_back(frameCV[0]);
    }

    // Store frame sequence
    frameSeqs[cNum].push_back(frameSeq[cNum]);

    if(cNum == 0)
    {
        // Add identifier to list
        QListWidgetItem* item = new QListWidgetItem(QString("Sequence %1").arg(frameSeqs[0].size()), ui->listWidget);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable); // set checkable flag
        item->setCheckState(Qt::Checked); // AND initialize check state
    }
    else
    {
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

void SLCalibrationDialog::on_calibrateButton_clicked(){

    // Disable interface elements
    ui->calibrateButton->setEnabled(false);
    ui->listWidget->setEnabled(false);
    ui->listWidget2->setEnabled(false);

    // Stop live view
    killTimer(liveViewTimer);
    reviewMode = true;
    ui->snapButton->setText("Live View");

    std::cout<< "----------------Calibrate Camear1-----------------------"<<std::endl;
    if(cNum == 0)
    {
        calibrator[0]->reset();
        // Note which frame sequences are used
        activeFrameSeqs[0].clear();
        for(int i=0; i<ui->listWidget->count(); i++){
            if(ui->listWidget->item(i)->checkState() == Qt::Checked){
                vector<cv::Mat> frameSeq(frameSeqs[0][i].begin(), frameSeqs[0][i].begin() + calibrator[0]->getNPatterns());
                calibrator[0]->addFrameSequence(frameSeq);
                activeFrameSeqs[0].push_back(i);
            }
        }
        // Perform calibration
        calib[0] = calibrator[0]->calibrate();
    }
    else if(cNum==1)
    {
        std::cout<< "----------------Calibrate Camear2-----------------------"<<std::endl;
        calibrator[1]->reset();
        // Note which frame sequences are used
        activeFrameSeqs[1].clear();
        for(int i=0; i<ui->listWidget2->count(); i++){
            if(ui->listWidget2->item(i)->checkState() == Qt::Checked){
                vector<cv::Mat> frameSeq(frameSeqs[1][i].begin(), frameSeqs[1][i].begin() + calibrator[0]->getNPatterns());
                calibrator[1]->addFrameSequence(frameSeq);
                activeFrameSeqs[1].push_back(i);
            }
        }
        // Perform calibration
        calib[1] = calibrator[1]->calibrate();
    }

    // Re-enable interface elements
    ui->calibrateButton->setEnabled(true);
    ui->listWidget->setEnabled(true);
    ui->listWidget2->setEnabled(true);
    ui->saveButton->setEnabled(true);
}

void SLCalibrationDialog::on_listWidget_itemSelectionChanged(){

    // If selection was cleared
    if(ui->listWidget->selectedItems().isEmpty())
        return;

    // Stop live view
    killTimer(liveViewTimer);
    reviewMode = true;
    ui->snapButton->setText("Live View");

    int currentRow = ui->listWidget->currentRow();
    ui->videoWidget->showFrameCV(frameSeqs[0][currentRow].back());
}

void SLCalibrationDialog::on_listWidget2_itemSelectionChanged(){

    // If selection was cleared
    if(ui->listWidget2->selectedItems().isEmpty())
        return;

    // Stop live view
    killTimer(liveViewTimer);
    reviewMode = true;
    ui->snapButton->setText("Live View2");

    int currentRow = ui->listWidget2->currentRow();
    ui->videoWidget2->showFrameCV(frameSeqs[1][currentRow].back());
}

void SLCalibrationDialog::on_saveButton_clicked(){

    unsigned int screenResX, screenResY;
    projector->getScreenRes(&screenResX, &screenResY);

    calib[cNum].frameWidth = camera[0]->getFrameWidth();
    calib[cNum].frameHeight = camera[0]->getFrameHeight();
    calib[cNum].screenResX = screenResX;
    calib[cNum].screenResY = screenResY;
    calib[cNum].calibrationDateTime = QDateTime::currentDateTime().toString("DD.MM.YYYY HH:MM:SS").toStdString();

    //QString filename = QString("frameSeq_cal_%1_%2.bmp").arg(seqNum, 2, 10, QChar('0')).arg(i, 2, 10, QChar('0'));
    //arg(int a, int fieldWidth = 0, int base = 10, QChar fillChar = QLatin1Char( ' ' )) const
    QString calFilename = QString("calibration_%1.xml").arg(cNum,1);
            calib[cNum].save(calFilename);

    this->close();
}


void SLCalibrationDialog::onNewSequenceResult(cv::Mat img, unsigned int idx, bool success){

    // Skip non-active frame sequences
    int idxListView = activeFrameSeqs[0][idx];

    // Append calibration result to frame sequence
    unsigned int N = calibrator[0]->getNPatterns();
    if(frameSeqs[0][idxListView].size() == N)
        frameSeqs[0][idxListView].push_back(img);
    else
        frameSeqs[0][idxListView][N] = img;

    if(!success) // uncheck
        ui->listWidget->item(idxListView)->setCheckState(Qt::Unchecked);

    // Highlight
    ui->listWidget->setCurrentRow(idxListView);
    ui->listWidget->setFocus();

    QApplication::processEvents();
}
void SLCalibrationDialog::onNewSequenceResult2(cv::Mat img, unsigned int idx, bool success){

    // Skip non-active frame sequences
    int idxListView = activeFrameSeqs[1][idx];

    // Append calibration result to frame sequence
    unsigned int N = calibrator[1]->getNPatterns();
    if(frameSeqs[1][idxListView].size() == N)
        frameSeqs[1][idxListView].push_back(img);
    else
        frameSeqs[1][idxListView][N] = img;

    if(!success) // uncheck
        ui->listWidget2->item(idxListView)->setCheckState(Qt::Unchecked);

    // Highlight
    ui->listWidget2->setCurrentRow(idxListView);
    ui->listWidget2->setFocus();

    QApplication::processEvents();
}

void SLCalibrationDialog::closeEvent(QCloseEvent *){
    // Stop live view
    killTimer(liveViewTimer);

    for(int c=0; c<camera.size();c++)
        delete camera[c];

    delete projector;
    delete calibrator[0];
    delete calibrator[1];
    this->deleteLater();

    // Save calibration settings
    QSettings settings("SLStudio");
    float checkerSize = ui->txtCheckSize->toPlainText().toFloat();//  ui->checkerSizeBox->value();
    settings.setValue("calibration/checkerSize", checkerSize);
    unsigned int checkerRows = ui->checkerRowsBox->value();
    settings.setValue("calibration/checkerRows", checkerRows);
    unsigned int checkerCols = ui->checkerColsBox->value();
    settings.setValue("calibration/checkerCols", checkerCols);

}
