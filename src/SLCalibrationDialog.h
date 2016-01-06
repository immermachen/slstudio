/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) 2013 -- 2014 Jakob Wilm, DTU, Kgs.Lyngby, Denmark
 *
*/

#ifndef SLCALIBRATAIONDIALOG_H
#define SLCALIBRATAIONDIALOG_H

#include <QDialog>
#include <QModelIndex>
#include <QListWidgetItem>

#include "Camera.h"
#include "Projector.h"
#include "Calibrator.h"
#include "SLStudio.h"

namespace Ui {
    class SLCalibrationDialog;
}

class SLCalibrationDialog : public QDialog {
    Q_OBJECT
    
    public:
        explicit SLCalibrationDialog(SLStudio *parent = 0);
        ~SLCalibrationDialog();
        void timerEvent(QTimerEvent *event);
        void closeEvent(QCloseEvent *);
    private slots:
        void on_snapButton_clicked();
        void on_calibrateButton_clicked();
        void on_listWidget_itemSelectionChanged();
        void on_listWidget2_itemSelectionChanged();
        void on_saveButton_clicked();
        void onNewSequenceResult(cv::Mat img, unsigned int idx, bool success);
        void onNewSequenceResult2(cv::Mat img, unsigned int idx, bool success);
    signals:
        void newCalibrationSaved(CalibrationData _calib);
    private:
        Ui::SLCalibrationDialog *ui;
        vector< Camera * > camera; //add two camera support
        Projector *projector;
        vector< Calibrator * > calibrator; //add two camera support
        CalibrationData calib[2]; //add two camera support
        int liveViewTimer;
        vector< vector<cv::Mat> > frameSeqs[2]; //n vectors of 62 vectors of Mat
        vector<unsigned int> activeFrameSeqs[2]; //add two camera support
        bool reviewMode;
        unsigned int timerInterval; //ms
        unsigned int delay; //ms
        bool diamondPattern;
        unsigned int screenCols;
        unsigned int screenRows;
};

#endif // SLCALIBRATAIONDIALOG_H
