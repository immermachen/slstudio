#include "CameraVimba.h"

std::vector<CameraInfo> CameraVimba::getCameraList(){
    using namespace AVT::VmbAPI;
    std::vector<CameraInfo> ret;

    VimbaSystem&    sys = VimbaSystem::GetInstance();  // Get a reference to the VimbaSystem singleton
    std::cout<<"Vimba Version V"<<sys<<"\n";           // Print out version of Vimba
    VmbErrorType    err = sys.Startup();               // Initialize the Vimba API
    CameraPtrVector cameras;                           // A vector of std::shared_ptr<AVT::VmbAPI::Camera> objects

    //std::stringstream strError;

    if( VmbErrorSuccess != err )
    {
        std::cout << "Could not start system. Error code: " << err <<"("<<AVT::VmbAPI::Examples::ErrorCodeToMessage(err)<<")"<< "\n";
        return ret;
    }

    err = sys.GetCameras( cameras );            // Fetch all cameras known to Vimba
    if( VmbErrorSuccess == err )
    {
        unsigned int numCameras = cameras.size();
        std::cout << "Cameras found: " << cameras.size() <<"\n\n";

        // Query all static details of all known cameras and print them out.
        // We don't have to open the cameras for that.
        //std::for_each( cameras.begin(), cameras.end(), PrintCameraInfo );


    }
    else
    {
        std::cout << "Could not list cameras. Error code: " << err << "("<<AVT::VmbAPI::Examples::ErrorCodeToMessage(err)<<")"<< "\n";
    }

    sys.Shutdown();                             // Close Vimba

    return ret;
}
