#include "SLPreferenceDialog.h"
#include "ui_SLPreferenceDialog.h"

#include "OpenGLContext.h"
#include "Camera.h"
#include "Codec.h"

#include <QSettings>

using namespace std;

SLPreferenceDialog::SLPreferenceDialog(QWidget *parent) : QDialog(parent), ui(new Ui::SLPreferenceDialog) {
    ui->setupUi(this);

    // Query projectors
    vector<ScreenInfo> screenList = OpenGLContext::GetScreenInfo();
    for(unsigned int i=0; i<screenList.size(); i++){
        QString screenString = QString("Screen %1: %2x%3").arg(i).arg(screenList[i].resX).arg(screenList[i].resY);
        ui->projectorComboBox->addItem(screenString, i);
    }
    // Add virtual projector option
    ui->projectorComboBox->addItem("SLStudio Virtual Screen 1024*768", -1);
    // Add virtual projector option
    ui->projectorComboBox->addItem("SLStudio Virtual Screen 1920*1080", -4);
    // Add LC3000 option
    #ifdef WITH_LC3000API
        ui->projectorComboBox->addItem("LC3000 API", -2);
    #endif
    // Add LC4500 option
    #ifdef WITH_LC4500API
        ui->projectorComboBox->addItem("LC4500 API", -3);
    #endif

    // Query cameras
    vector< vector<CameraInfo> > interfaceCameraList = Camera::GetInterfaceCameraList();
    for(unsigned int i=0; i<interfaceCameraList.size(); i++){
        vector<CameraInfo> cameraList = interfaceCameraList[i];
        for(unsigned int j=0; j<cameraList.size(); j++){
            string cam = "Left";
            if(j==1) cam = "Right";
            QString cameraString = QString("%4Cam%1:%2: %3").arg(j,1).arg(cameraList[j].vendor.c_str()).arg(cameraList[j].model.c_str()).arg(cam.c_str());
            ui->cameraComboBox->addItem(cameraString, QPoint(i, j)); //[0 0] and [0 1]
        }
    }
    // Add virtual camera option
    ui->cameraComboBox->addItem("SLStudio Virtual LeftCamera0", QPoint(-1, 0));
    ui->cameraComboBox->addItem("SLStudio Virtual RightCamera1", QPoint(-1, 1));
    ui->cameraComboBox->addItem("CP: Two Virtual Cameras", QPoint(-1, 2)); //2: use two cameras;
    ui->cameraComboBox->addItem("CC: Two Virtual Cameras", QPoint(-1, 3)); //2: use two cameras;
    // Add two cameras options
    ui->cameraComboBox->addItem("CP: Two Vimba Cameras", QPoint(0, 2)); //2: use two cameras; 0: vimba camera interface
    ui->cameraComboBox->addItem("CC: Two Vimba Cameras", QPoint(0, 3)); //3: use two cameras; 0: vimba camera interface

    // List pattern modes
    ui->patternModeComboBox->addItem("Gray and Phase Shift 4", "CodecGrayPhase4");
    ui->patternModeComboBox->addItem("3 Pattern Phase Shift", "CodecPhaseShift3");
    ui->patternModeComboBox->addItem("4 Pattern Phase Shift", "CodecPhaseShift4");
    ui->patternModeComboBox->addItem("2x3 Pattern Phase Shift", "CodecPhaseShift2x3");
    ui->patternModeComboBox->addItem("3 Pattern Phase Shift Unwrap", "CodecPhaseShift3Unwrap");
    ui->patternModeComboBox->addItem("N Step Pattern Phase Shift", "CodecPhaseShiftNStep");
    ui->patternModeComboBox->addItem("3 Pattern Phase Shift Fast Wrap", "CodecPhaseShift3FastWrap");
    ui->patternModeComboBox->addItem("2+1 Pattern Phase Shift", "CodecPhaseShift2p1");
    ui->patternModeComboBox->addItem("Descattering Phase Shift", "CodecPhaseShiftDescatter");
    ui->patternModeComboBox->addItem("Modulated Phase Shift", "CodecPhaseShiftModulated");
    ui->patternModeComboBox->addItem("Micro Phase Shift", "CodecPhaseShiftMicro");
    ui->patternModeComboBox->addItem("Fast Ratio", "CodecFastRatio");
    ui->patternModeComboBox->addItem("Gray Coding", "CodecGrayCode");

    // Set all elements to current application settings
    QSettings settings("SLStudio");

    QString aquistion = settings.value("aquisition","continuous").toString();
    if(aquistion == "continuous")
        ui->aquisitioncontinuousRadioButton->setChecked(true);
    else
        ui->aquisitionSingleRadioButton->setChecked(true);

    unsigned int flip = settings.value("flip","1").toInt();
    if(flip == 0)
        ui->flipcamera0->setChecked(true);
    else
        ui->flipcamera1->setChecked(true);

    unsigned int patternModeIndex = ui->patternModeComboBox->findData(settings.value("pattern/mode"));
    ui->patternModeComboBox->setCurrentIndex(patternModeIndex);

    CodecDir codecDir = (CodecDir)settings.value("pattern/direction", CodecDirHorizontal).toInt();
    ui->patternHorizontalCheckBox->setChecked(codecDir & CodecDirHorizontal);
    ui->patternVerticalCheckBox->setChecked(codecDir & CodecDirVertical);

    int projectorIndex = ui->projectorComboBox->findData(settings.value("projector/screenNumber"));
    ui->projectorComboBox->setCurrentIndex(projectorIndex);
    ui->diamondPatternCheckBox->setChecked(settings.value("projector/diamondPattern").toBool());
    //ui->verticalBaselineCheckbox->setChecked(settings.value("projector/verticalBaseline").toBool());

    QPoint cameraInterfaceSetting = QPoint(settings.value("camera/interfaceNumber").toInt(), settings.value("camera/cameraNumber").toInt());
    unsigned int cameraIndex = ui->cameraComboBox->findData(cameraInterfaceSetting);
    ui->cameraComboBox->setCurrentIndex(cameraIndex);

    float shutter = settings.value("camera/shutter", 16.666).toFloat();
    ui->shutterDoubleSpinBox->setValue(shutter);


    QString triggerMode = settings.value("trigger/mode","hardware").toString();
    if(triggerMode == "hardware"){
        ui->triggerHardwareRadioButton->setChecked(true);
        on_triggerHardwareRadioButton_clicked();
    } else {
        ui->triggerSoftwareRadioButton->setChecked(true);
        on_triggerSoftwareRadioButton_clicked();
    }
    unsigned int shift = settings.value("trigger/shift",0).toInt();
    ui->shiftSpinBox->setValue(shift);
    unsigned int delay = settings.value("trigger/delay",50).toInt();
    ui->delaySpinBox->setValue(delay);

    bool frames = settings.value("writeToDisk/frames",false).toBool();
    ui->framesCheckBox->setChecked(frames);

    bool pointclouds = settings.value("writeToDisk/pointclouds",false).toBool();
    ui->pointCloudsCheckBox->setChecked(pointclouds);

    bool tracking = settings.value("writeToDisk/tracking",false).toBool();
    ui->trackingCheckBox->setChecked(tracking);

    float threshold_complementary = settings.value("Decoding/threshold_complementary", 0.1).toFloat();
    ui->Threshold_Complentary->setValue(threshold_complementary);
    float threshold_epipolarLineWidth = settings.value("Triangulator/threshold_epipolarLineWidth", 1.0).toFloat();
    ui->Threshold_Epipolarline->setValue(threshold_epipolarLineWidth);
    bool debug_decoding = settings.value("debug/decoding",false).toBool();
    ui->CB_Decoding->setChecked(debug_decoding);
    bool debug_triangulation = settings.value("debug/triangulation",false).toBool();
    ui->CB_Decoding->setChecked(debug_triangulation);
}

SLPreferenceDialog::~SLPreferenceDialog(){
    delete ui;
}

void SLPreferenceDialog::on_buttonBox_accepted(){

    // Save settings
    QSettings settings("SLStudio");

    // Aquisition
    if(ui->aquisitioncontinuousRadioButton->isChecked())
        settings.setValue("aquisition", "continuous");
    else
        settings.setValue("aquisition", "single");

    if(ui->flipcamera0->isChecked())
        settings.setValue("flip","0");
    else
        settings.setValue("flip","1");

    // Pattern mode
    QString patternMode = ui->patternModeComboBox->itemData(ui->patternModeComboBox->currentIndex()).toString();
    settings.setValue("pattern/mode", patternMode);

    // Pattern direction
    bool patternHorizontal = ui->patternHorizontalCheckBox->isChecked();
    bool patternVertical = ui->patternVerticalCheckBox->isChecked();
    CodecDir dir = CodecDirNone;
    if(patternHorizontal && patternVertical)
        dir = CodecDirBoth;
    else if(patternHorizontal)
        dir = CodecDirHorizontal;
    else if(patternVertical)
        dir = CodecDirVertical;
    settings.setValue("pattern/direction", dir);

    // Projector
    int proj = ui->projectorComboBox->itemData(ui->projectorComboBox->currentIndex()).toInt();
    settings.setValue("projector/screenNumber", proj);
    bool diamondPattern = ui->diamondPatternCheckBox->isChecked();
    settings.setValue("projector/diamondPattern", diamondPattern);
    //bool verticalBaseline = ui->verticalBaselineCheckbox->isChecked();
    //settings.setValue("projector/verticalBaseline", verticalBaseline);
    if(proj==-1)
    {
        settings.setValue("projectorVirtual/screenResX", 1024);
        settings.setValue("projectorVirtual/screenResY", 768);
    }
    if(proj==-4)
    {
        settings.setValue("projectorVirtual/screenResX", 1920);
        settings.setValue("projectorVirtual/screenResY", 1080);
    }

    // Camera
    QPoint cam = ui->cameraComboBox->itemData(ui->cameraComboBox->currentIndex()).toPoint();
    settings.setValue("camera/interfaceNumber", cam.x());
    settings.setValue("camera/cameraNumber", cam.y());

    float shutter = ui->shutterDoubleSpinBox->value();
    settings.setValue("camera/shutter", shutter);

    // Trigger mode
    if(ui->triggerHardwareRadioButton->isChecked())
        settings.setValue("trigger/mode", "hardware");
    else
        settings.setValue("trigger/mode", "software");
    unsigned int shift = ui->shiftSpinBox->value();
    settings.setValue("trigger/shift", shift);
    unsigned int delay = ui->delaySpinBox->value();
    settings.setValue("trigger/delay", delay);

    // Write to disk
    bool frames =  ui->framesCheckBox->isChecked();
    settings.setValue("writeToDisk/frames", frames);
    bool pointclouds =  ui->pointCloudsCheckBox->isChecked();
    settings.setValue("writeToDisk/pointclouds", pointclouds);
    bool tracking =  ui->trackingCheckBox->isChecked();
    settings.setValue("writeToDisk/tracking", tracking);

    float threshold_complementary = ui->Threshold_Complentary->value();
    settings.setValue("Decoding/threshold_complementary", threshold_complementary);
    float threshold_epipolarLineWidth = ui->Threshold_Epipolarline->value();
    settings.setValue("Triangulator/threshold_epipolarLineWidth", threshold_epipolarLineWidth);
    bool debug_decoding =  ui->CB_Decoding->isChecked();
    settings.setValue("debug/decoding", debug_decoding);
    bool debug_triangulation =  ui->CB_Triangulation->isChecked();
    settings.setValue("debug/triangulation", debug_triangulation);
}



void SLPreferenceDialog::on_triggerHardwareRadioButton_clicked(){
//    ui->shiftLayout->setEnabled(true);
//    ui->delayLayout->setEnabled(false);
    ui->shiftLabel->setEnabled(true);
    ui->shiftSpinBox->setEnabled(true);
    ui->delayLabel->setEnabled(false);
    ui->delaySpinBox->setEnabled(false);
    ui->delayMsLabel->setEnabled(false);
}

void SLPreferenceDialog::on_triggerSoftwareRadioButton_clicked(){
//    ui->delayLayout->setEnabled(true);
//    ui->shiftLayout->setEnabled(false);
    ui->shiftLabel->setEnabled(false);
    ui->shiftSpinBox->setEnabled(false);
    ui->delayLabel->setEnabled(true);
    ui->delaySpinBox->setEnabled(true);
    ui->delayMsLabel->setEnabled(true);
}

void SLPreferenceDialog::on_cameraComboBox_currentIndexChanged(const QString &arg1){
    if(arg1 == "SLStudio Virtual Camera"){
        ui->shutterDoubleSpinBox->setEnabled(false);
    } else {
        ui->shutterDoubleSpinBox->setEnabled(true);
    }
}

void SLPreferenceDialog::on_patternHorizontalCheckBox_clicked(){
    if(!ui->patternHorizontalCheckBox->isChecked())
        ui->patternVerticalCheckBox->setChecked(true);
}

void SLPreferenceDialog::on_patternVerticalCheckBox_clicked(){
    if(!ui->patternVerticalCheckBox->isChecked())
        ui->patternHorizontalCheckBox->setChecked(true);
}
