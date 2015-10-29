#include <vimbaframeobserver.h>

using namespace AVT::VmbAPI;

VimbaFrameObserver::VimbaFrameObserver(CameraPtr pCamera) : IFrameObserver( pCamera ) {
    //connect(this,SIGNAL(FrameReceivedSignal(int)),consumer,SLOT(GrabFrame()));
}


void VimbaFrameObserver::FrameReceived( const FramePtr pFrame )
{
    qDebug()<<"VimbaFrameObserver::FrameReceived()--->";

    bool bQueueDirectly = true;
    VmbFrameStatusType eReceiveStatus;

    if( VmbErrorSuccess == pFrame->GetReceiveStatus( eReceiveStatus ) )
    {
        qDebug()<<"VimbaFrameObserver::FrameReceived--> OK! and push a pframe!";

        // Lock the frame queue
        allFramesMutex.lock();
        // Add frame to queue
        allFrames.push( pFrame );
        // Unlock frame queue
        allFramesMutex.unlock();
        // Emit the frame received signal
        //emit FrameReceivedSignal( eReceiveStatus );
        bQueueDirectly = false;
    }
    else
    {
        qDebug()<<"VimbaFrameObserver::FrameReceived--> No!";
    }

    // If any error occurred we queue the frame without notification
    //if( true == bQueueDirectly )
    {
        m_pCamera->QueueFrame( pFrame ); //m_pCamera is the inherent name of the camera pointer of an IFrameObserver
    }
}

// Returns the oldest frame that has not been picked up yet
FramePtr VimbaFrameObserver::GetFrame()
{
    qDebug()<<"VimbaFrameObserver::GetFrame()-->";
    // Lock the frame queue
    allFramesMutex.lock();
    FramePtr res;
    if(allFrames.size()>0){
        qDebug()<<"VimbaFrameObserver::GetFrame()--> allFrame Size=" << allFrames.size();
        // Pop frame from queue
        res = allFrames.front();
        allFrames.pop();
    }
    else
    {
        qDebug()<<"VimbaFrameObserver::GetFrame()--> No frames!";
    }
    // Unlock frame queue
    allFramesMutex.unlock();
    return res;
}

void VimbaFrameObserver::ClearFrameQueue()
{
    // Lock the frame queue
    allFramesMutex.lock();
    // Clear the frame queue and release the memory
    std::queue<FramePtr> empty;
    std::swap( allFrames, empty );
    // Unlock the frame queue
    allFramesMutex.unlock();
}
