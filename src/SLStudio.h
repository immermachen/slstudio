/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) 2013 -- 2014 Jakob Wilm, DTU, Kgs.Lyngby, Denmark
 *
*/

#ifndef SLSTUDIO_H
#define SLSTUDIO_H

#include <QMainWindow>
#include <QTimer>
#include <QPointer>
#include <QSettings>

#include "SLScanWorker.h"
#include "SLTrackerWorker.h"
#include "SLPointCloudWidget.h"
#include "SLVideoDialog.h"
#include "SLTrackerDialog.h"

namespace Ui {
    class SLStudio;
}

class SLStudio : public QMainWindow {
    Q_OBJECT
    
    public:
        explicit SLStudio(QWidget *parent = 0);
        void closeEvent(QCloseEvent *event);
        ~SLStudio();

    private slots:
        void onActionStart();
        void onActionStop();
        void onScanWorkerFinished();

        void onActionCalibration();
        void onActionLoadCalibration();
        void onActionPreferences();
        void onActionExportCalibration();

        void updateDisplayRate();
        void receiveNewPointCloud(PointCloudPtr pointCloud, unsigned int _cNum);

        void imshow(const char* windowName, cv::Mat im, unsigned int x, unsigned int y);
        void hist(const char* windowName, cv::Mat im, unsigned int x, unsigned int y);

        void onShowHistogram(cv::Mat im);
        void onShowShading(cv::Mat im);
        void onShowDecoderUp(cv::Mat im);
        void onShowDecoderVp(cv::Mat im);

        void onActionAbout();

signals:
        void newPointCloud(PointCloudConstPtr pointCloud);

    private:
        Ui::SLStudio *ui;
        std::vector<unsigned int> displayIntervals;

        SLScanWorker *scanWorker;
        QThread *scanWorkerThread;

        SLDecoderWorker *decoderWorker;
        QThread *decoderThread;
        SLDecoderWorker *decoderWorker2;
        QThread *decoderThread2;

        SLTriangulatorWorker *triangulatorWorker;
        QThread *triangulatorThread;
        SLTriangulatorWorker *triangulatorWorker2;
        QThread *triangulatorThread2;

        QTime *time;
        QSettings *settings;

        SLVideoDialog *histogramDialog, *shadingDialog, *decoderUpDialog, *decoderVpDialog;
        SLTrackerDialog *trackerDialog;

        unsigned int nPCL;
    public:

};

#endif
