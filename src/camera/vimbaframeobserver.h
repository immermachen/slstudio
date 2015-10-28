#ifndef VIMBAFRAMEOBSERVER
#define VIMBAFRAMEOBSERVER

#include <VimbaCPP/Include/VimbaCPP.h>

#include <queue>

#include <QObject>
#include <QMutex>
#include <QDebug>

class VimbaFrameObserver : virtual public AVT::VmbAPI::IFrameObserver
{
    //Q_OBJECT //only for signal/slot mechanism

  public:
    // We pass the camera that will deliver the frames to the constructor
    VimbaFrameObserver( AVT::VmbAPI::CameraPtr pCamera);
    
    // This is our callback routine that will be executed on every received frame
    virtual void FrameReceived( const AVT::VmbAPI::FramePtr pFrame );

    // After the camBackend has been notified about a new frame it can pick it up
    AVT::VmbAPI::FramePtr GetFrame();

    // Clears the double buffer frame queue
    void ClearFrameQueue();

  private:
    // Since a Qt signal cannot contain a whole frame
    // the frame observer stores all FramePtr
    std::queue<AVT::VmbAPI::FramePtr> allFrames;
    QMutex allFramesMutex;
};

#endif
