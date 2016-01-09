#include "SLProjectorVirtual.h"

#include <QTime>
#include <QTest>
#include <QSettings>

SLProjectorVirtual::SLProjectorVirtual(unsigned int){
    QSettings settings("SLStudio");
    screenResX = settings.value("projectorVirtual/screenResX", 1024).toInt();
    screenResY = settings.value("projectorVirtual/screenResY", 768).toInt();

    time = new QTime();
    time->start();
}

void SLProjectorVirtual::waitForProjection(){
    // Wait till 17 msec have elapsed on time
    unsigned int elapsed = time->elapsed();
    if(elapsed < 33)
        QTest::qSleep(33 - elapsed);

    // Reset time
    time->restart();
}

void SLProjectorVirtual::displayPattern(unsigned int){
    this->waitForProjection();
}

void SLProjectorVirtual::displayTexture(const unsigned char*, unsigned int, unsigned int){
    this->waitForProjection();
}

void SLProjectorVirtual::displayBlack(){
    this->waitForProjection();
}

void SLProjectorVirtual::displayWhite(){
    this->waitForProjection();
}

void SLProjectorVirtual::getScreenRes(unsigned int *nx, unsigned int *ny){
    *nx = screenResX;
    *ny = screenResY;
}

SLProjectorVirtual::~SLProjectorVirtual(){
    delete time;
}
