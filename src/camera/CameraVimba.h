#ifndef CAMERAVIMBA_H
#define CAMERAVIMBA_H

#include "Camera.h"
#include <VimbaCPP.h>
#include <VimbaCPP/Examples/Common/ErrorCodeToMessage.h>
#include <VimbaCPP/Examples/Common/StreamSystemInfo.h>


class CameraVimba : public Camera{
public:
    AVT::VmbAPI::VimbaSystem& vimbaSystem = AVT::VmbAPI::VimbaSystem::GetInstance();  // Get a reference to the VimbaSystem singleton
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
    AVT::VmbAPI::CameraPtr camera;

};

#endif // CAMERAVIMBA_H
