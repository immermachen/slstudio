/*
 * CalibratorLocHome -- Calibrate using "local homographies" as proposed by Moreno, Taubin.
*/

#ifndef CALIBRATORCC_H
#define CALIBRATORCC_H

#include "Calibrator.h"

#include "Codec.h"

#include <QThread>

using namespace std;

class CalibratorCC : public Calibrator {
    Q_OBJECT
    public:
        CalibratorCC(unsigned int _screenCols, unsigned int _screenRows);
        ~CalibratorCC(){delete encoder; delete decoder;}

        CalibrationData calibrate();

    private:
        Encoder *encoder;
        Decoder *decoder;
};

#endif // CALIBRATORCC_H
