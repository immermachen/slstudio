#ifndef CAMERAVIMBA_H
#define CAMERAVIMBA_H

#include "Camera.h"

#include <VimbaCPP.h>
//#include <VimbaC.h>
using namespace AVT;

//#include <VimbaCPPCommon.h>
//#include <Camera.h>
//#include <Interface.h>
//#include <VimbaSystem.h>
//#include <FeatureContainer.h>
//#include <ICameraFactory.h>
//#include <ICameraListObserver.h>
//#include <IInterfaceListObserver.h>
//#include <IFeatureObserver.h>
//#include <IFrameObserver.h>
//#include <Frame.h>


class CameraVimba : public Camera{
    public:
        //Static methods
        static std::vector<CameraInfo> getCameraList();
        //Interface function
        CameraVimba(unsigned int camNum, CameraTriggerMode triggerMode);
        CameraSettings getCameraSettings();
        void setCameraSettings(CameraSettings);
        void startCapture();
        void stopCapture();
        CameraFrame getFrame();
        size_t getFrameSizeBytes();
        size_t getFrameWidth();
        size_t getFrameHeight();
        ~CameraVimba();
    private:

};

#endif // CAMERAVIMBA_H
