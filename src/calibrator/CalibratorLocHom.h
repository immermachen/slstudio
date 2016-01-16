/*
 * CalibratorLocHome -- Calibrate using "local homographies" as proposed by Moreno, Taubin.
*/

#ifndef CALIBRATORLOCHOM_H
#define CALIBRATORLOCHOM_H

#include "Calibrator.h"

#include "Codec.h"

#include <QThread>

using namespace std;

class CalibratorLocHom : public Calibrator {
    Q_OBJECT
    public:
        CalibratorLocHom(unsigned int _screenCols, unsigned int _screenRows);

        void calibrateWrap(uint numCam);
        ~CalibratorLocHom(){delete encoder; delete decoder;}
public slots:
        CalibrationData calibrate();
        void slot_calibrateWrap(uint numCam);
signals:
        void signal_calFinished(uint numCam, CalibrationData calData);
        void finished();


    private:
        Encoder *encoder;
        Decoder *decoder;
};

#endif // CALIBRATORLOCHOM_H
