#include "CameraVimba.h"


static std::vector<CameraInfo> getCameraList(){
    using namespace AVT::VmbAPI;

    std::vector<CameraInfo> ret;

    VimbaSystem&    sys = VimbaSystem::GetInstance();  // Get a reference to the VimbaSystem singleton

    VmbErrorType    err = sys.Startup();               // Initialize the Vimba API
    CameraPtrVector cameras;                           // A vector of std::shared_ptr<AVT::VmbAPI::Camera> objects

    std::stringstream strError;

    if( VmbErrorSuccess == err )
    {
        err = sys.GetCameras( cameras );            // Fetch all cameras known to Vimba
        if( VmbErrorSuccess == err )
        {
            std::cout << "Cameras found: " << cameras.size() <<"\n\n";

            // Query all static details of all known cameras and print them out.
            // We don't have to open the cameras for that.
            std::for_each( cameras.begin(), cameras.end(), PrintCameraInfo );
        }
        else
        {
            std::cout << "Could not list cameras. Error code: " << err << "("<<AVT::VmbAPI::Examples::ErrorCodeToMessage(err)<<")"<< "\n";
        }

        sys.Shutdown();                             // Close Vimba
    }
    else
    {
        std::cout << "Could not start system. Error code: " << err <<"("<<AVT::VmbAPI::Examples::ErrorCodeToMessage(err)<<")"<< "\n";
    }

    return ret;
}

//CameraVimba(unsigned int camNum, CameraTriggerMode triggerMode){}

//CameraSettings getCameraSettings(){}
//void setCameraSettings(CameraSettings){}
//void startCapture(){}
//void stopCapture(){}
//CameraFrame getFrame(){}
//size_t getFrameSizeBytes(){}
//size_t getFrameWidth(){}
//size_t getFrameHeight(){}
//~CameraVimba(){}
