#include "CameraVimba.h"
#include <stdlib.h>  //for atoi

using namespace AVT::VmbAPI;
std::vector<CameraInfo> CameraVimba::getCameraList(){

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
        // Configure for hardware trigger

    } else if(triggerMode == triggerModeSoftware) {
        // Configure for software trigger (for getSingleFrame())

    }

    IFrameObserverPtr pObserver( new MyFrameObserver() ); //TODO
    FeaturePtr pFeature;
    VmbUint64_t nPLS;
    camera->GetFeatureByName("PayloadSize", pFeature);
    pFeature->GetValue(nPLS);
    FramePtr frame;
    *frame = new Frame(nPLS);
    frame->RegisterObserver(pObserver);
    camera->AnnounceFrame(*frame);

    // Start aquistion
    camera->StartCapture();
    camera->QueueFrame(*frame);

    camera->GetFeatureByName("AcquisitionStart",pFeature);
    pFeature->RunCommand();

    capturing = true;
}

class MyFrameObserer: public IFrameObserver{
public:
    MyFrameObserer(){

    }
};

void CameraVimba::stopCapture(){
    camera->EndCapture();
    capturing = false;
}

CameraFrame CameraVimba::getFrame(){
    // Create single image buffer
    XI_IMG image;
    image.size = SIZE_XI_IMG_V2; // must be initialized
    image.bp = NULL;
    image.bp_size = 0;

    if(triggerMode == triggerModeSoftware){
        // Fire software trigger
        stat = xiSetParamInt(camera, XI_PRM_TRG_SOFTWARE, 0);
        HandleResult(stat,"xiSetParam (XI_PRM_TRG_SOFTWARE)");

        // Retrieve image from camera
        stat = xiGetImage(camera, 1000, &image);
        HandleResult(stat,"xiGetImage");
    } else {

        // Retrieve image from camera
        stat = xiGetImage(camera, 50, &image);
        HandleResult(stat,"xiGetImage");
    }

    // Empty buffer
    while(xiGetImage(camera, 1, &image) == XI_OK){
        std::cerr << "drop!" << std::endl;
        continue;
    }

    CameraFrame frame;
    frame.height = image.height;
    frame.width = image.width;
    frame.memory = (unsigned char*)image.bp;
    frame.timeStamp = image.tsUSec;
    frame.sizeBytes = image.bp_size;

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
    AVT::VmbAPI::FramePtr frame;

    VmbUint32_t width;
    frame->GetWidth(width);
    return width;
}


size_t CameraVimba::getFrameHeight(){
    AVT::VmbAPI::FramePtr frame;

    VmbUint32_t height;
    frame->GetHeight(height);
    return height;
}

CameraVimba::~CameraVimba(){
    vimbaSystem.Shutdown();                             // Close Vimba
}
