#include "CameraVimba.h"
#include <stdlib.h>  //for atoi

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

        for(int i=0; i<numCameras;i++){
            CameraInfo info;
            CameraPtr cam = cameras[i];
            info.vendor = "Vimba";
            cam->GetModel(info.model);
            std::string cameraID;
            cam->GetID(cameraID);
            std::cout << "cameraID: " << cameraID <<"\n\n";
            info.busID = atoi(cameraID.c_str());
            ret.push_back(info);
        }
    }
    else
    {
        std::cout << "Could not list cameras. Error code: " << err << "("<<AVT::VmbAPI::Examples::ErrorCodeToMessage(err)<<")"<< "\n";
    }

    sys.Shutdown();                             // Close Vimba

    return ret;
}

CameraVimba::CameraVimba(unsigned int camNum, CameraTriggerMode triggerMode) : Camera(triggerMode){
    using namespace AVT::VmbAPI;

    std::cout<<"Vimba Version V"<<vimbaSystem<<"\n";           // Print out version of Vimba
    VmbErrorType    err = vimbaSystem.Startup();               // Initialize the Vimba API
    CameraPtrVector cameras;                           // A vector of std::shared_ptr<AVT::VmbAPI::Camera> objects

    if( VmbErrorSuccess != err )
    {
        std::cout << "Could not start system. Error code: " << err <<"("<<AVT::VmbAPI::Examples::ErrorCodeToMessage(err)<<")"<< "\n";
        return;
    }

    err = vimbaSystem.GetCameras( cameras );            // Fetch all cameras known to Vimba
    if( VmbErrorSuccess == err ){
        unsigned int numCameras = cameras.size();
        camera = cameras[camNum-1];
        std::string cameraID;
        camera->GetID(cameraID);
        //vimbaSystem.OpenCameraByID(cameraID.c_str(), triggerMode, camera);
        camera->Open(VmbAccessModeFull);
        std::cout << "Cameras open: " << cameraID <<"\n\n";
    }
    else
    {
        std::cout << "Could not list cameras. Error code: " << err << "("<<AVT::VmbAPI::Examples::ErrorCodeToMessage(err)<<")"<< "\n";
    }

    return;
}


CameraSettings CameraVimba::getCameraSettings(){

    // Get settings:
    CameraSettings settings;
    return settings;
}



void CameraVimba::setCameraSettings(CameraSettings settings){

}


void CameraVimba::startCapture(){

    if(triggerMode == triggerModeHardware){

    } else if(triggerMode == triggerModeSoftware) {


    }

    capturing = true;
}

void CameraVimba::stopCapture(){
    camera->EndCapture();
    capturing = false;
}

CameraFrame CameraVimba::getFrame(){

    CameraFrame frame;

    if (!capturing) {
        //cerr << "ERROR: Not capturing on camera. Call startCapture() before lockFrame()." << endl;
        return frame;
    }


    if(triggerMode == triggerModeSoftware){

    }

    return frame;
}


size_t CameraVimba::getFrameSizeBytes(){
    if (!capturing) {
        //cerr << "ERROR: Cannot get frame size before capturing. Call startCapture() before getFrameSizeBytes()." << endl;
        return 0;
    }


    return 0;
}

size_t CameraVimba::getFrameWidth(){
    return 0;
}


size_t CameraVimba::getFrameHeight(){
    return 0;
}

CameraVimba::~CameraVimba(){
    vimbaSystem.Shutdown();                             // Close Vimba
}
