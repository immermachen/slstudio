#include "CalibratorCC.h"
#include "CodecCalibration.h"

#include "CodecGrayPhase.h"

#include "cvtools.h"

#include <QSettings>

CalibratorCC::CalibratorCC(unsigned int _screenCols, unsigned int _screenRows): Calibrator(_screenCols, _screenRows){

    // Create encoder/decoder
    //encoder = new EncoderCalibration(screenCols, screenRows, CodecDirBoth);
    //decoder = new DecoderCalibration(screenCols, screenRows, CodecDirBoth);
    encoder = new EncoderGrayPhase(screenCols, screenRows, CodecDirBoth);
    decoder = new DecoderGrayPhase(screenCols, screenRows, CodecDirBoth);

    this->N = encoder->getNPatterns();

    frameSeqsFromFile.resize(N);

    for(unsigned int i=0; i<N; i++)
        patterns.push_back(encoder->getEncodingPattern(i));
}

CalibrationData CalibratorCC::calibrate()
{
    QSettings settings("SLStudio");

    //Checkerboard parameters
    float checkerSize = settings.value("calibration/checkerSize").toFloat();
    std::cout << "checkerSize== "<< checkerSize <<std::endl;
    unsigned int checkerRows = settings.value("calibration/checkerRows").toInt();
    unsigned int checkerCols = settings.value("calibration/checkerCols").toInt();

    // Number of saddle points on calibration pattern
    cv::Size patternSize(checkerCols,checkerRows);

    // Number of calibration sequences
    unsigned nFrameSeq = frameSeqsFromFile.size();

    // Read frame sequences
    std::cout << "Decode frames: Num="<< nFrameSeq<< ", and begin.....>> ";
    for(unsigned int i=0; i<nFrameSeq; i++)
    {
        vector<cv::Mat> frames;
        vector<std::string> framesFromFile= frameSeqsFromFile[i];
        for(unsigned int m=0; m<framesFromFile.size();m++)
        {
            cv::Mat curFrame = cv::imread(framesFromFile[m],CV_LOAD_IMAGE_GRAYSCALE);            
            curFrame = curFrame.clone();
            frames.push_back(curFrame);
        }
        frameSeqs.push_back(frames);

        std::cout << i << ",";
    }
    std::cout << "-->end||."<<std::endl;

    unsigned int frameWidth = frameSeqs[0][0].cols;
    unsigned int frameHeight = frameSeqs[0][0].rows;

    // Generate local calibration object coordinates [mm]
    std::cout << "Generate local calibration object coordinates"<<std::endl;
    vector<cv::Point3f> Qi;
    for (int h=0; h<patternSize.height; h++)
        for (int w=0; w<patternSize.width; w++)
            Qi.push_back(cv::Point3f(checkerSize * w, checkerSize* h, 0.0));

    // Find calibration point coordinates for cameras
    std::cout<< "Find calibration point coordinates for camera1 and Camera2!" <<std::endl;

    vector< vector<cv::Point2f> > qc, qp; //camera1 and camera2
    vector< vector<cv::Point3f> > Q;

    for(unsigned int i=0; i<nFrameSeq; i++)
    {
        vector<cv::Mat> shading = frameSeqs[i];
        //std::cout << i << " 1" << std::endl;
        vector< vector<cv::Point2f> > qci;
        // Aid checkerboard extraction by slight blur
        //cv::GaussianBlur(shading[i], shading[i], cv::Size(5,5), 2, 2);
        // Extract checker corners

        std::cout << i << ": findChessboardCorners 1:  ......" << std::endl;
        bool success0 = cv::findChessboardCorners(shading[0], patternSize, qci[0], cv::CALIB_CB_ADAPTIVE_THRESH+CV_CALIB_CB_NORMALIZE_IMAGE+CALIB_CB_FAST_CHECK );
        std::cout << " cv::findChessboardCorners: sucess1 = " << success << std::endl;

        std::cout << i << ": findChessboardCorners 2:  ......" << std::endl;
        bool success1 = cv::findChessboardCorners(shading[1], patternSize, qci[1], cv::CALIB_CB_ADAPTIVE_THRESH+CV_CALIB_CB_NORMALIZE_IMAGE+CALIB_CB_FAST_CHECK );
        std::cout << " cv::findChessboardCorners: sucess2 = " << success << std::endl;

        if(!success0 || !success1)
            std::cout << "Calibrator: could not extract chess board corners on both frame seqences " << i << std::endl << std::flush;
        else
        {
            std::cout << i << ": cornerSubPix......" << std::endl;
            // Refine corner locations
            cv::cornerSubPix(shading[0], qci, cv::Size(5, 5), cv::Size(1, 1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 20, 0.01));
            cv::cornerSubPix(shading[1], qpi, cv::Size(5, 5), cv::Size(1, 1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 20, 0.01));
        }
        // Draw colored chessboard
        vector<cv::Mat> shadingColor;
        cv::cvtColor(shading[0], shadingColor[0], cv::COLOR_GRAY2RGB);
        cv::drawChessboardCorners(shadingColor[0], patternSize, qci[0], success0);
        cv::cvtColor(shading[1], shadingColor[1], cv::COLOR_GRAY2RGB);
        cv::drawChessboardCorners(shadingColor[1], patternSize, qci[1], success1);

#if 1
        QString filename0 = QString("am_shadingColor%1_0.bmp").arg(i, 2, 10, QChar('0'));
        QString filename1 = QString("am_shadingColor%1_1.bmp").arg(i, 2, 10, QChar('0'));
        cv::imwrite(filename0.toStdString(), shadingColor[0]);
        cv::imwrite(filename1.toStdString(), shadingColor[1]);
//        filename = QString("am_shadingColor%1.png").arg(i, 2, 10, QChar('0'));
//        cv::imwrite(filename.toStdString(), shadingColor);
#endif
        //Emit chessboard results
        emit newSequenceResult(shadingColor, i, success);

        if(success)
        {
            // Vectors of accepted points for current view
            vector<cv::Point2f> qpi_a;
            vector<cv::Point2f> qci_a;
            vector<cv::Point3f> Qi_a;

            // Loop through checkerboard corners
            for(unsigned int j=0; j<qci.size(); j++)
            { 
                qpi_a.push_back(qpi[j]);
                qci_a.push_back(qci[j]);
                Qi_a.push_back(Qi[j]);
            }

            if(!Qi_a.empty())
            {
                // Store projector corner coordinates
                qp.push_back(qpi_a);

                // Store camera corner coordinates
                qc.push_back(qci_a);

                // Store world corner coordinates
                Q.push_back(Qi_a);
            }
        }
    }

    if(Q.size() < 1){
        std::cerr << "Error: not enough calibration sequences!" << std::endl;
        CalibrationData nanData;
        return nanData;
    }

    //calibrate the camera
    std::cout<< "calibrate the camera!" <<std::endl;

    cv::Mat Kc, kc;
    std::vector<cv::Mat> cam_rvecs, cam_tvecs;
    cv::Size frameSize(frameWidth, frameHeight);
    double cam_error = cv::calibrateCamera(Q, qc, frameSize, Kc, kc, cam_rvecs, cam_tvecs, cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_PRINCIPAL_POINT + cv::CALIB_FIX_K2 + cv::CALIB_FIX_K3 + cv::CALIB_ZERO_TANGENT_DIST,
                                           cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON));

    //calibrate the projector
    std::cout<< "calibrate the projector!" <<std::endl;
    cv::Mat Kp, kp;
    std::vector<cv::Mat> proj_rvecs, proj_tvecs;
    cv::Size screenSize(screenCols, screenRows);
    double proj_error = cv::calibrateCamera(Q, qp, screenSize, Kp, kp, proj_rvecs, proj_tvecs, cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K2 + cv::CALIB_FIX_K3 + cv::CALIB_ZERO_TANGENT_DIST,
                                            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON));

    //stereo calibration
    std::cout<< "stereo calibrate!" <<std::endl;
    cv::Mat Rp, Tp, E, F;
    //Opencv2.x version
    //double stereo_error = cv::stereoCalibrate(Q, qc, qp, Kc, kc, Kp, kp, frameSize, Rp, Tp, E, F,
      //                                        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON), cv::CALIB_FIX_INTRINSIC);

    //Opencv3.x version
    double stereo_error = cv::stereoCalibrate(Q, qc, qp, Kc, kc, Kp, kp, frameSize, Rp, Tp, E, F,
                                              cv::CALIB_FIX_INTRINSIC,cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON));

    CalibrationData calData(Kc, kc, cam_error, Kp, kp, proj_error, Rp, Tp, stereo_error);

    calData.print(std::cout);

    // Determine per-view reprojection errors:
    std::vector<float> cam_error_per_view(Q.size());
    cam_error_per_view.resize(Q.size());
    std::vector<float> proj_error_per_view(Q.size());
    proj_error_per_view.resize(Q.size());
    for(unsigned int i = 0; i < (unsigned int)Q.size(); ++i){
        int n = (int)Q[i].size();

        vector<cv::Point2f> qc_proj;
        cv::projectPoints(cv::Mat(Q[i]), cam_rvecs[i], cam_tvecs[i], Kc, kc, qc_proj);
        float err = 0;
        for(int j=0; j<qc_proj.size(); j++){
            cv::Point2f d = qc[i][j] - qc_proj[j];
            err += cv::sqrt(d.x*d.x + d.y*d.y);
        }
        cam_error_per_view[i] = (float)err/n;

        vector<cv::Point2f> qp_proj;
        cv::projectPoints(cv::Mat(Q[i]), proj_rvecs[i], proj_tvecs[i], Kp, kp, qp_proj);
        err = 0;
        for(int j=0; j<qc_proj.size(); j++){
            cv::Point2f d = qp[i][j] - qp_proj[j];
            err += cv::sqrt(d.x*d.x + d.y*d.y);
        }
        proj_error_per_view[i] = (float)err/n;

        std::cout << "Seq error " << i+1 << " cam:" << cam_error_per_view[i] << " proj:" << proj_error_per_view[i] << std::endl;
    }

    return calData;
//                CalibrationData nanData;
//                return nanData;
}
