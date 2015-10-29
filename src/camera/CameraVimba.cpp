/*****************************************************************************
**             Yang: some notation
**  Frames contain image meta-data as well as references to the data that were sent by the camera (image
**  and ancillary data). For use in Vimba, they must be created by the application and then be queued at the
**  corresponding camera. When an image was received, the next available frame is filled and handed over to
**  the application through a dedicated notification. After having processed the image data, the application
**  should return the frame to the API by re-enqueuing it at the corresponding camera.

******************************************************************************/


#include "CameraVimba.h"
#include <stdlib.h>  //for atoi
#include <qinputdialog.h>

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

CameraVimba::CameraVimba(unsigned int camNum, CameraTriggerMode triggerMode) :
    system ( AVT::VmbAPI::VimbaSystem::GetInstance() ), bufCount(50), initialStamp(0), Camera(triggerMode){

    Init(camNum);
}

bool CameraVimba::Init(unsigned int camNum){
    VmbErrorType    err=system.Startup();
    CameraPtrVector cameras;

    if( VmbErrorSuccess == err )
    {
        // now find all cameras
        err = system.GetCameras( cameras );            // Fetch all cameras known to Vimba
        if( VmbErrorSuccess == err ) {

            if (cameras.size()>0) {
                if (cameras.size()>1) {
                    qDebug() << "Cameras found: " << cameras.size();  // should also implement Qinputdialog to let the user choose which one to use
                    for (uint i=0;i<cameras.size();i++) {
                        CameraPtr cam=cameras[i];
                        std::string namestr;
                        err=cam->GetName(namestr);
                        qDebug()<<"Next Camera is: "<<QString::fromStdString(namestr);
                    }
                }

                // now open
                pCamera=cameras[camNum];
                std::string camID;
                std::string namestr;
                err=pCamera->GetName(namestr);
                err=pCamera->GetID(camID);
                if( VmbErrorSuccess == err )    {
                    qDebug()<<"Opening camera "<<QString::fromStdString(namestr);

                    err=pCamera->Open(VmbAccessModeFull);
                    if (err==VmbErrorSuccess) {
                        //camera successfully opened. Now do some camera initialisation steps
                        qDebug()<<"camera successfully opened";

                        // Set the GeV packet size to the highest possible value
                        // (In this example we do not test whether this cam actually is a GigE cam)
                        FeaturePtr pCommandFeature;
                        if ( VmbErrorSuccess == pCamera->GetFeatureByName( "GVSPAdjustPacketSize", pCommandFeature ))
                        {
                            if ( VmbErrorSuccess == pCommandFeature->RunCommand() )
                            {
                                bool bIsCommandDone = false;
                                do
                                {
                                    if ( VmbErrorSuccess != pCommandFeature->IsCommandDone( bIsCommandDone ))
                                    {
                                        break;
                                    }
                                } while ( false == bIsCommandDone );
                            }
                        }

                        // get/set some features
                        FeaturePtr pFeature;


                        // get Camera timestamp frequency
                        err=pCamera->GetFeatureByName("GevTimestampTickFrequency",pFeature);
                        if (err==VmbErrorSuccess) {
                            err=pFeature->GetValue(camFreq);
                            if (err==VmbErrorSuccess) {
//                                qDebug()<<"Camera freq is "<<(1.0*camFreq);
                            } else {
                                qDebug()<<"Could not extract freq: "<<err;
                            }
                        } else {
                            qDebug()<<"Could not query frequency: "<<err<<" => Will use LUT";
                            if (namestr=="GE1050") {
                                camFreq=79861111;
                            } else if (namestr=="GE1910") {
                                camFreq=79861111;
                            } else if (namestr=="GE2040") {
                                camFreq=79861111;
                            } else {
                                qDebug()<<"Model not yet in LUT => unreliable timestamps";
                                camFreq=79861111;
                            }
                        }


                        // Set acquisition mode to continuous
                        err=pCamera->GetFeatureByName("AcquisitionMode",pFeature);
                        if (err==VmbErrorSuccess) {
                            pFeature->SetValue("Continuous"); // this should be continuous
                        }

                        // set/get maximum height and width of current camera
                        err=pCamera->GetFeatureByName("WidthMax",pFeature);
                        if (err==VmbErrorSuccess) {
                            pFeature->GetValue(maxWidth);
                        }

                        err=pCamera->GetFeatureByName("Width",pFeature);
                        if (err==VmbErrorSuccess) {
                            pFeature->SetValue(maxWidth);
                            pFeature->GetValue(width);
                        }

                        err=pCamera->GetFeatureByName("HeightMax",pFeature);
                        if (err==VmbErrorSuccess) {
                            pFeature->GetValue(maxHeight);
                        }

                        err=pCamera->GetFeatureByName("Height",pFeature);
                        if (err==VmbErrorSuccess) {
                            pFeature->SetValue(maxHeight);
                            pFeature->GetValue(height);
                        }

                        // make sure shutter time is manual
                        err=pCamera->GetFeatureByName("ExposureAuto",pFeature);
                        if (err==VmbErrorSuccess) {
                            pFeature->SetValue("Off"); // this should be manual exposure setting
                        }

                        // Set up 8bit monochrome color depth
                        err=pCamera->GetFeatureByName("PixelFormat",pFeature);
                        if (err==VmbErrorSuccess) {
                            pFeature->SetValue(VmbPixelFormatMono8);//"Mono8",VmbPixelFormatMono8
                        }
                        else
                        {
                            qDebug()<<"Init-->GetFeatureByName: PixelFormat: err: "<< err;
                        }

                        // Disable gamma mode
                        err=pCamera->GetFeatureByName("Gamma",pFeature);
                        if (err==VmbErrorSuccess) {
                            pFeature->SetValue("OFF");
                        }
                        else
                        {
                            qDebug()<<"Init-->GetFeatureByName: Gamma: err: "<< err;
                        }

                        // Disable frame-rate mode

                        // Set reasonable default settings
                        CameraSettings settings;
                        //settings.shutter = 8.333;
                        settings.shutter = 16.66;
                        settings.gain = 0.0;
                        this->setCameraSettings(settings);


                        qDebug()<<"camera successfully opened --> Set/Get finished!";

                    } else {
                        qDebug()<<"camera did not open successfully";
                        return false;
                    }
                }
            } else {
                qDebug()<<"Zero cameras found";
                return false;
            }



        } else {
            qDebug() << "Could not list cameras. Error code: " << err;
            return false;
        }

    } else {
        qDebug() << "Could not start system. Error code: " << err;
        return false;
    }


    qDebug()<<"Init --> End  ";
    return true;
}


CameraSettings CameraVimba::getCameraSettings(){

    // Get settings:
    CameraSettings settings;
    return settings;
}



void CameraVimba::setCameraSettings(CameraSettings settings){

}


void CameraVimba::startCapture(){
    std::cout << "startCapture --> "<<std::endl;

    if(triggerMode == triggerModeHardware){
        // Configure for hardware trigger

    } else if(triggerMode == triggerModeSoftware) {
        // Configure for software trigger (for getSingleFrame())
        // Set Trigger source to fixedRate or Software or Freerun[default   ]
        err=pCamera->GetFeatureByName("TriggerSource",pFeature);
        if (err==VmbErrorSuccess) {
            pFeature->SetValue("Software");
        }
    }

    // construct the frame observer to the camera
    frameWatcher=new VimbaFrameObserver( pCamera );
    qDebug()<<"camera successfully opened --> VimbaFrameObserver";

    VmbErrorType err;
    IFrameObserverPtr pObserver( frameWatcher ); //TODO

    FramePtrVector frames ( 1);
    FeaturePtr pFeature;
    VmbInt64_t nPLS;

    err = pCamera->GetFeatureByName ( "PayloadSize", pFeature );
    err = pFeature->GetValue( nPLS );
    if (err==VmbErrorSuccess) {
        qDebug()<< "startCapture --> PayloadSize Work, nPLS= " << nPLS;
    }

    for(FramePtrVector::iterator iter = frames.begin (); frames.end () != iter; ++iter ){
        (*iter).reset( new Frame ( nPLS ) );


        err = (*iter)->RegisterObserver ( pObserver );

        if (err==VmbErrorSuccess) {
            qDebug()<< "startCapture --> RegisterObserver Work! ";
        }

        err = pCamera->AnnounceFrame( *iter );

        if (err==VmbErrorSuccess) {
            qDebug()<< "startCapture --> AnnounceFrame  Work! ";
        }
    }

    qDebug()<<"startCapture --> RegisterObserver";

    // Start aquistion
    //err = pCamera->StartCapture();
    err = pCamera->StartContinuousImageAcquisition(3,pObserver);

    if (err==VmbErrorSuccess) {
        qDebug()<< "StartCapture -->StartCapture   Work! ";
    }

    for(FramePtrVector::iterator iter = frames.begin (); frames.end () != iter; ++iter ){
        err = pCamera->QueueFrame( *iter );
        if (err==VmbErrorSuccess) {
            qDebug()<< "StartCapture -->QueueFrame   Work! ";
        }else
        {
            qDebug()<< "StartCapture -->QueueFrame not  Work: err= "<< err;
        }
    }

    pCamera->GetFeatureByName("AcquisitionStart",pFeature); //AcquisitionStop
    err = pFeature->RunCommand();
    if (err==VmbErrorSuccess) {
        qDebug()<< "AcquisitionStart --> RunCommand  Work! ";
    }else
    {
        qDebug()<< "AcquisitionStart -->RunCommand not  Work: err= "<< err;
    }


    qDebug()<<"startCapture --> RunCommand -->AcquisitionStart ";
    capturing = true;
}



CameraFrame CameraVimba::getFrame(){

    qDebug()<<"getFrame -->  ";

    CameraFrame frame;

    AVT::VmbAPI::FramePtr pFrame=frameWatcher->GetFrame();

    if(pFrame == NULL)
    {
        qDebug()<<"getFrame --> NO Frame!  ";
        return frame;
    }

    qDebug()<<"getFrame --> GetFrame() works!  ";

    VmbErrorType err;
    VmbUint32_t width,height,size;
    VmbPixelFormatType pixFormat;
    VmbUchar_t* image;
    err=pFrame->GetPixelFormat(pixFormat);
    err=pFrame->GetHeight(height);
    err=pFrame->GetWidth(width);
    err=pFrame->GetImage(image);
    err=pFrame->GetImageSize(size);

    VmbUint64_t stamp;
    err=pFrame->GetTimestamp(stamp);
//    if (initialStamp==0) {
//        initialStamp=stamp;
//        qint64 currMsec=QDateTime::currentMSecsSinceEpoch();
//        timeOffset=currMsec;
//    }
//    int timeStamp=timeOffset+1000.0*(stamp-initialStamp)/(1.0*camFreq);
//    qDebug()<<"TODO????: Time Stamp in ms: "<<(timeStamp-timeOffset);

    pCamera->QueueFrame( pFrame ); // requeue here. Not sure what will happen if buffer too small!

    frame.height = height;
    frame.width = width;
    frame.memory = (unsigned char*)image;
    frame.timeStamp = stamp;
    frame.sizeBytes = size;

    qDebug()<<"getFrame --> GetFrame-->End";
    return frame;
}


size_t CameraVimba::getFrameSizeBytes(){
    if (!capturing) {
        //cerr << "ERROR: Cannot get frame size before capturing. Call startCapture() before getFrameSizeBytes()." << endl;
        return 0;
    }


    return 0;
}


void CameraVimba::stopCapture(){
    FeaturePtr pFeature;
    pCamera->GetFeatureByName("AcquisitionStop",pFeature); //AcquisitionStop
    pFeature->RunCommand();
    pCamera->EndCapture();
    capturing = false;
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
    VmbErrorType err=pCamera->Close();
    if (err!=VmbErrorSuccess) {
        qDebug()<<"Problem closing the camera";
    }
    err=system.Shutdown();
    if (err!=VmbErrorSuccess) {
        qDebug()<<"Problem shutting down Vimba";
    }
}

std::vector<std::string> CameraVimba::listPixelFormats() {
    FeaturePtr pFeature;
    VmbErrorType err;
    err = pCamera->GetFeatureByName( "PixelFormat", pFeature );
    if ( VmbErrorSuccess == err ) {
        return listOptions(pFeature);
    } else {
        StringVector vals;
        return vals;
    }
}
std::vector<std::string> CameraVimba::listOptions(FeaturePtr pFeature) {
    StringVector vals;
    StringVector realVals;
    pFeature->GetValues(vals);
    for (uint i=0;i<vals.size();i++) {
        bool ok=false;
        pFeature->IsValueAvailable(vals[i].c_str(),ok);
        if (ok) {
            realVals.push_back(vals[i]);
//            qDebug()<<"Valid value: " << QString::fromStdString(vals[i]);
        }
    }
    return realVals;
}
void CameraVimba::setFormat(QString formatstring) {
    format=formatstring;
    FeaturePtr pFeature;
    VmbErrorType err;

    // Check/set pixel format
    // Set pixel format. For the sake of simplicity we only support Mono and BGR in this example.
    err = pCamera->GetFeatureByName( "PixelFormat", pFeature );
    if ( VmbErrorSuccess == err )
    {
        if (format=="MONO8") {
            err = pFeature->SetValue( VmbPixelFormatMono8 );
        } else if (format=="MONO12") {
            err = pFeature->SetValue( VmbPixelFormatMono12 );
        } else if (format=="MONO14") {
            err = pFeature->SetValue( VmbPixelFormatMono14 );
        } else if (format=="BAYERRG8") {
            err=pFeature->SetValue(VmbPixelFormatBayerRG8);
        } else if (format=="BAYERGB8") {
            err=pFeature->SetValue(VmbPixelFormatBayerGB8);
        } else {
            qDebug()<<"Pixel Format not yet working in Gigaviewer: "<<format;
        }
        if (err!=VmbErrorSuccess) {
            qDebug()<<"Could not set requested pixel format.";
        }
        std::string form;
        err=pFeature->GetValue(form);
//        qDebug()<<"Working in "<<QString::fromStdString(form)<<" mode";
//                                qDebug()<<"Will work in Mono8 mode";
    }
}
