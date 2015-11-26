#ifndef CAMERAVIMBA_H
#define CAMERAVIMBA_H

#include "Camera.h"
//#include "vimbaframeobserver.h"
#include <VimbaCPP.h>
#include <VimbaCPP/Examples/Common/ErrorCodeToMessage.h>
#include <VimbaCPP/Examples/Common/StreamSystemInfo.h>
#include <stdio.h>
#include <Bitmap.h>

class CameraVimba : public Camera{
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
    std::string     ErrorCodeToMessage( VmbErrorType eErr ) const ;
    ~CameraVimba();

private: 
    bool Init(unsigned int camNum);

    std::vector<std::string> listOptions(AVT::VmbAPI::FeaturePtr pFeature);
    void setFormat(std::string formatstring);

    std::vector<std::string> listPixelFormats();

    AVT::VmbAPI::VimbaSystem & m_system;
    AVT::VmbAPI::CameraPtr     m_pCamera;
//    VimbaFrameObserver* frameWatcher;
    int bufCount;
    VmbInt64_t height,width;
    VmbInt64_t maxHeight,maxWidth;
    double frameRate,exposure;
    VmbInt64_t camFreq,initialStamp;
    int timeOffset;
    std::string format;
};

#endif // CAMERAVIMBA_H
