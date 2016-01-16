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
        CalibrationData calibrate();
        ~CalibratorLocHom(){delete encoder; delete decoder;}
public slots:
        void slot_calibrateWrap(unsigned int numCam);
signals:
        void signal_calFinished(unsigned int numCam, CalibrationData calData);
        void finished();


    private:
        Encoder *encoder;
        Decoder *decoder;
};

#endif // CALIBRATORLOCHOM_H
