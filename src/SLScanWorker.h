/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) 2013 -- 2014 Jakob Wilm, DTU, Kgs.Lyngby, Denmark
 *
*/

#ifndef SLWORKER_H
#define SLWORKER_H

#include <QObject>
#include <QThread>
#include <QMainWindow>

#include "Camera.h"
#include "Projector.h"
#include "Codec.h"

#include "SLDecoderWorker.h"
#include "SLTriangulatorWorker.h"

enum ScanAquisitionMode{ aquisitionContinuous, aquisitionSingle };

class SLScanWorker : public QObject {
    Q_OBJECT

    public:
        SLScanWorker(QObject *parent): isWorking(false){}
        ~SLScanWorker();
    public slots:
        void setup();
        void doWork();
        void doWork_CC();
        void stopWorking();
    signals:
        //void imshow(const char* windowName, cv::Mat mat, unsigned int x, unsigned int y);
        //void hist(const char* windowName, cv::Mat mat, unsigned int x, unsigned int y);
        void showHistogram(cv::Mat im);
        void newFrame(cv::Mat frame);
        void newFrameSeq(std::vector<cv::Mat> frameSeq);
        void newFrameSeq(std::vector<cv::Mat> frameSeq0, std::vector<cv::Mat> frameSeq1);
        void newFrameSeq2(std::vector<cv::Mat> frameSeq);
        void error(QString err);
        void finished();
    private:
        int iNum;
        int cNum;
        unsigned int flip;
        std::vector<cv::Mat> frameSeq[2];
        bool isWorking;
        std::vector<Camera*> camera;
        Projector * projector;
        Encoder *encoder;

        CameraTriggerMode triggerMode;
        ScanAquisitionMode aquisition;
        bool writeToDisk;
        void setupProjector(int c);
};

#endif
