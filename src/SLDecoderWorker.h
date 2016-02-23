/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) 2013 -- 2014 Jakob Wilm, DTU, Kgs.Lyngby, Denmark
 *
*/

#ifndef SLDECODERWORKER_H
#define SLDECODERWORKER_H

#include <QObject>
#include <QTime>

#include "Codec.h"

class SLDecoderWorker : public QObject {
    Q_OBJECT

    public:
        SLDecoderWorker(unsigned int _cNum) : cNum(_cNum), screenCols(0), screenRows(0){}
        ~SLDecoderWorker();
    public slots:
        void setup();
        void decodeSequence(std::vector<cv::Mat> frameSeq);
        void decodeSequence(std::vector<cv::Mat> frameSeq0, std::vector<cv::Mat> frameSeq1);
    signals:
        void imshow(const char* windowName, cv::Mat mat, unsigned int x, unsigned int y);
        void showShading(cv::Mat mat);
        void showDecoderUp(cv::Mat mat);
        void showDecoderVp(cv::Mat mat);
        void newUpVp(cv::Mat up, cv::Mat vp, cv::Mat mask, cv::Mat shading);
        void newUpVp(cv::Mat up0, cv::Mat vp0, cv::Mat mask0, cv::Mat shading0, cv::Mat up1, cv::Mat vp1, cv::Mat mask1, cv::Mat shading1);
        void error(QString err);
        void finished();
    private:
        Decoder *decoder;
        unsigned int screenCols, screenRows;
        unsigned int cNum;
        QTime time;
        bool busy;
};

#endif
