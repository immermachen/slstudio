#ifndef CAMERAVIMBA_H
#define CAMERAVIMBA_H

#include "Camera.h"

#include "VimbaCPP.h"
#include "Common/ErrorCodeToMessage.h"
#include "Common/StreamSystemInfo.h"

class CameraVimba : public Camera {
    public:
        // Static methods
        static std::vector<CameraInfo> getCameraList();
        // Interface function
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
