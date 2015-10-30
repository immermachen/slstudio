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
#include <QDateTime>
#include <QSettings>


#include <opencv2/opencv.hpp>
#include "cvtools.h"

using namespace AVT::VmbAPI;




CameraFrame CameraVimba::getFrame(){

    qDebug()<<"getFrame -->  ";

    CameraFrame frame;
    FramePtr pFrame;
    VmbErrorType err;

    //AVT::VmbAPI::FramePtr pFrame=frameWatcher->GetFrame();

    QSettings settings("SLStudio");
    unsigned int shift = settings.value("trigger/shift", "0").toInt();
    unsigned int delay = settings.value("trigger/delay", "100").toInt();

    err = m_pCamera->AcquireSingleImage( pFrame, delay );//timeout=5000:The time to wait until the frame got filled

    if (err!=VmbErrorSuccess) {
        qDebug()<< "AcquireSingleImage err="<< ErrorCodeToMessage(err).c_str();
        return frame;
    }

    qDebug()<<"getFrame --> GetFrame() works!  ";

    const char *    pFileName   = NULL;             // The filename for the bitmap to save

    pFileName = "SynchronousGrab.bmp";


    VmbPixelFormatType ePixelFormat = VmbPixelFormatMono8;
    err = pFrame->GetPixelFormat( ePixelFormat );
    if ( VmbErrorSuccess != err )
    {
        qDebug()<< "GetPixelFormat err="<< ErrorCodeToMessage(err).c_str();
        return frame;
    }

    if(  ( VmbPixelFormatMono8 != ePixelFormat ) &&  ( VmbPixelFormatRgb8 != ePixelFormat ))
    {
        err = VmbErrorInvalidValue;
        qDebug()<< "VmbPixelFormatMono8 err="<< ErrorCodeToMessage(err).c_str();
        return frame;
    }

    VmbUint32_t nImageSize = 0;
    err = pFrame->GetImageSize( nImageSize );
    if ( VmbErrorSuccess != err )
    {
        qDebug()<< "GetImageSize err="<< ErrorCodeToMessage(err).c_str();
        return frame;
    }

    VmbUint32_t nWidth = 0;
    err = pFrame->GetWidth( nWidth );
    if ( VmbErrorSuccess != err )
    {
        qDebug()<< "GetWidth err="<< ErrorCodeToMessage(err).c_str();
        return frame;
    }

    VmbUint32_t nHeight = 0;
    err = pFrame->GetHeight( nHeight );
    if ( VmbErrorSuccess != err )
    {
        qDebug()<< "GetWidth err="<< ErrorCodeToMessage(err).c_str();
        return frame;
    }

    VmbUchar_t *pImage = NULL;
    err = pFrame->GetImage( pImage );
    if ( VmbErrorSuccess != err )
    {
        qDebug()<< "GetImage err="<< ErrorCodeToMessage(err).c_str();
        return frame;
    }

    AVTBitmap bitmap;

    if( VmbPixelFormatRgb8 == ePixelFormat )
    {
        qDebug()<< "ePixelFormat colorCode=ColorCodeRGB24";
        bitmap.colorCode = ColorCodeRGB24;
    }
    else
    {
        qDebug()<< "ePixelFormat colorCode=ColorCodeMono8";
        bitmap.colorCode = ColorCodeMono8;
    }

    bitmap.bufferSize = nImageSize;
    bitmap.width = nWidth;
    bitmap.height = nHeight;

    // Create the bitmap
    if ( 0 == AVTCreateBitmap( &bitmap, pImage ))
    {
        std::cout<<"Could not create bitmap.\n";
        err = VmbErrorResources;
    }
    else
    {
        // Save the bitmap
        if ( 0 == AVTWriteBitmapToFile( &bitmap,  pFileName) )
        {
            std::cout<<"Could not write bitmap to file.\n";
            err = VmbErrorOther;
        }
        else
        {
            std::cout<<"Bitmap successfully written to file \""<<pFileName<<"\"\n" ;
//            // Release the bitmap's buffer
//            if ( 0 == AVTReleaseBitmap( &bitmap ))
//            {
//                std::cout<<"Could not release the bitmap.\n";
//                err = VmbErrorInternalFault;
//            }
        }
    }

    VmbUint64_t stamp;
    err=pFrame->GetTimestamp(stamp);

    frame.height = nHeight;
    frame.width = nWidth;
    frame.memory = pImage; //(unsigned char*)bitmap.buffer;
    frame.timeStamp = stamp;
    frame.sizeBytes = nImageSize;

    //qDebug()<<"getFrame --> GetFrame-->End. size="<< size << ", Height=" << height << ", width=" << width << ", Format=" << pixFormat << ", stamp="<< stamp;
    qDebug()<<"getFrame --> GetFrame-->End. size="<< frame.sizeBytes << ", Height=" << frame.height << ", width=" << frame.width << ", Format=" << bitmap.colorCode << ", stamp="<< stamp;

    //---debug--------------------------------------------------------
    // Create 8 bit OpenCV matrix
    cv::Mat frameCV(frame.height, frame.width, CV_8UC1, frame.memory);//CV_32S, CV_8U
    //frameCV = frameCV.clone();
    QString filename = QString("frameSeq_%1.bmp").arg(100, 2, 10, QChar('0'));
    cv::imwrite(filename.toStdString(), frameCV);
    //debug end-----------------------------------------------------

    return frame;
}

CameraVimba::CameraVimba(unsigned int camNum, CameraTriggerMode triggerMode) :
    m_system ( AVT::VmbAPI::VimbaSystem::GetInstance() ), bufCount(50), initialStamp(0), Camera(triggerMode){

    Init(camNum);
}

bool CameraVimba::Init(unsigned int camNum){
    VmbErrorType    err=m_system.Startup();
    CameraPtrVector cameras;

    if( VmbErrorSuccess == err )
    {
        // now find all cameras
        err = m_system.GetCameras( cameras );            // Fetch all cameras known to Vimba
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
                m_pCamera=cameras[camNum];
                std::string camID;
                std::string namestr;
                err=m_pCamera->GetName(namestr);
                err=m_pCamera->GetID(camID);
                if( VmbErrorSuccess == err )    {
                    qDebug()<<"Opening camera "<<QString::fromStdString(namestr);

                    err=m_pCamera->Open(VmbAccessModeFull);

                    if (err==VmbErrorSuccess) {
                        //camera successfully opened. Now do some camera initialisation steps
                        qDebug()<<"camera successfully opened";

                        // Set the GeV packet size to the highest possible value
                        // (In this example we do not test whether this cam actually is a GigE cam)
                        FeaturePtr pCommandFeature;
                        if ( VmbErrorSuccess == m_pCamera->GetFeatureByName( "GVSPAdjustPacketSize", pCommandFeature ))
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



void CameraVimba::startCapture(){
    std::cout << "startCapture --> "<<std::endl;

    VmbErrorType err;
    FeaturePtr pFeature;

    if(triggerMode == triggerModeHardware){
        // Configure for hardware trigger

    }
    else if(triggerMode == triggerModeSoftware)
    {
        // Configure for software trigger (for getSingleFrame())
        // Set Trigger source to fixedRate or Software or Freerun[default   ]
        err=m_pCamera->GetFeatureByName("TriggerSource",pFeature);
        if (err==VmbErrorSuccess) {
            pFeature->SetValue("Freerun");
        }
    }


    qDebug()<<"startCapture --> RunCommand -->AcquisitionStart ";
    capturing = true;
}


void CameraVimba::stopCapture(){
    VmbErrorType err=m_system.Shutdown();
    if (err!=VmbErrorSuccess) {
        qDebug()<<"Problem closing the camera";
    }
    capturing = false;
}

CameraVimba::~CameraVimba(){
    VmbErrorType err=m_pCamera->Close();
    if (err!=VmbErrorSuccess) {
        qDebug()<<"Problem closing the camera";
    }
    err=m_system.Shutdown();
    if (err!=VmbErrorSuccess) {
        qDebug()<<"Problem shutting down Vimba";
    }
}

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

CameraSettings CameraVimba::getCameraSettings(){

    // Get settings:
    CameraSettings settings;
    return settings;
}

void CameraVimba::setCameraSettings(CameraSettings settings){

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


std::vector<std::string> CameraVimba::listPixelFormats() {
    FeaturePtr pFeature;
    VmbErrorType err;
    err = m_pCamera->GetFeatureByName( "PixelFormat", pFeature );
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
    err = m_pCamera->GetFeatureByName( "PixelFormat", pFeature );
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

// Translates Vimba error codes to readable error messages
std::string CameraVimba::ErrorCodeToMessage( VmbErrorType eErr ) const
{
    return AVT::VmbAPI::Examples::ErrorCodeToMessage( eErr );
}
