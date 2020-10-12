//
//  picoCamera.cpp
//  PicoCameraInfo
//
//  Created by Azhar Aulia Saputra on 10/25/18.
//  Copyright © 2018 Azhar Aulia Saputra. All rights reserved.
//

/****************************************************************************\
 * Copyright (C) 2017 Infineon Technologies & pmdtechnologies ag
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 \****************************************************************************/

#include <opencv2/opencv.hpp>
#include <OpenGL/OpenGL.h>
#include <GLUT/GLUT.h>
#include <royale.hpp>
#include <iostream>
#include <mutex>
#include "picoCamera.h"
#include "gng.hpp"
#include "rnd.h"
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include <sys/types.h>
#include <sample_utils/PlatformResources.hpp>
#include "NgimuReceive.h"
#include "OpenGL.hpp"
#include "projection.h"

double dataCam1[171][224][3];
double dataCam2[171][224][3];
double dataCam3[171][224][3];
double dataCam4[171][224][3];
double allDataCam[4 * 171 * 224][DIM];
double dataCamInt[4 * 171 * 224][DIM];

using namespace royale;
using namespace sample_utils;
using namespace std;
using namespace cv;

bool showObject = false;
bool updataGNG = true;
bool showGNG = true;
bool showTriangle = false;
bool captureData = false;
double dMax = 0;
int ND;

namespace
{
    class MyListener : public IDepthDataListener
    {
        
    public:
        
        MyListener() :
        undistortImage(false)
        {
        }
        
        void onNewData(const DepthData *data)
        {
            // this callback function will be called for every new depth frame
            
            auto exposureTimes = data->exposureTimes;
            std::lock_guard<std::mutex> lock(flagMutex);
            
            int k = 0;
            for (int y = 0; y < data->width; y++)
            {
                for (int x = 0; x < data->height; x++, k++)
                {
                    auto curPoint = data->points.at(k);
                    double norm = sqrt(curPoint.x*curPoint.x + curPoint.y*curPoint.y + curPoint.z*curPoint.z);
                    dMax = max(dMax, norm);
                    if (curPoint.depthConfidence > 0 && norm > 0.05)
                    {
                        dataCam1[x][y][2] = GAIN * data->points.at(k).x;
                        dataCam1[x][y][1] = -GAIN * data->points.at(k).y;
                        dataCam1[x][y][0] = GAIN * data->points.at(k).z;
                        // if the point is valid, map the pixel from 3D world
                        // coordinates to a 2D plane (this will distort the image)
                    }
                    else {

                        dataCam1[x][y][0] = 0;// 500.0 * data->points.at(k).x;
                        dataCam1[x][y][1] = 0;// -500.0 * data->points.at(k).y;
                        dataCam1[x][y][2] = 0;// 500.0 * data->points.at(k).z;
                    }
                }
            }
        }
        
        void setLensParameters(const LensParameters &lensParameters)
        {
            // Construct the camera matrix
            // (fx   0    cx)
            // (0    fy   cy)
            // (0    0    1 )
            cameraMatrix = (Mat1d(3, 3) << lensParameters.focalLength.first, 0, lensParameters.principalPoint.first,
                            0, lensParameters.focalLength.second, lensParameters.principalPoint.second,
                            0, 0, 1);
            
            // Construct the distortion coefficients
            // k1 k2 p1 p2 k3
            distortionCoefficients = (Mat1d(1, 5) << lensParameters.distortionRadial[0],
                                      lensParameters.distortionRadial[1],
                                      lensParameters.distortionTangential.first,
                                      lensParameters.distortionTangential.second,
                                      lensParameters.distortionRadial[2]);
        }
        
        void toggleUndistort()
        {
            std::lock_guard<std::mutex> lock(flagMutex);
            undistortImage = !undistortImage;
        }
        
    private:
        // lens matrices used for the undistortion of
        // the image
        Mat cameraMatrix;
        Mat distortionCoefficients;
        
        std::mutex flagMutex;
        bool undistortImage;
    };
    
    class MyListener2 : public IDepthDataListener
    {
        
    public:
        
        MyListener2() :
        undistortImage(false)
        {
        }
        
        void onNewData(const DepthData *data)
        {
            // this callback function will be called for every new depth frame
            
            auto exposureTimes = data->exposureTimes;
            std::lock_guard<std::mutex> lock(flagMutex);
            
            int k = 0;
            for (int y = 0; y < data->width; y++)
            {
                for (int x = 0; x < data->height; x++, k++)
                {
                    auto curPoint = data->points.at(k);
                    double norm = sqrt(curPoint.x*curPoint.x + curPoint.y*curPoint.y + curPoint.z*curPoint.z);
                    dMax = max(dMax, norm);
                    if (curPoint.depthConfidence > 0 && norm > 0.05)
                    {
                        dataCam2[x][y][2] = GAIN * data->points.at(k).x;
                        dataCam2[x][y][1] = -GAIN * data->points.at(k).y;
                        dataCam2[x][y][0] = GAIN * data->points.at(k).z;
                        // if the point is valid, map the pixel from 3D world
                        // coordinates to a 2D plane (this will distort the image)
                    }
                    else {

                        dataCam2[x][y][0] = 0;// 500.0 * data->points.at(k).x;
                        dataCam2[x][y][1] = 0;// -500.0 * data->points.at(k).y;
                        dataCam2[x][y][2] = 0;// 500.0 * data->points.at(k).z;
                    }
                }
            }
        }
        
        void setLensParameters(const LensParameters &lensParameters)
        {
            // Construct the camera matrix
            // (fx   0    cx)
            // (0    fy   cy)
            // (0    0    1 )
            cameraMatrix = (Mat1d(3, 3) << lensParameters.focalLength.first, 0, lensParameters.principalPoint.first,
                            0, lensParameters.focalLength.second, lensParameters.principalPoint.second,
                            0, 0, 1);
            
            // Construct the distortion coefficients
            // k1 k2 p1 p2 k3
            distortionCoefficients = (Mat1d(1, 5) << lensParameters.distortionRadial[0],
                                      lensParameters.distortionRadial[1],
                                      lensParameters.distortionTangential.first,
                                      lensParameters.distortionTangential.second,
                                      lensParameters.distortionRadial[2]);
        }
        
        void toggleUndistort()
        {
            std::lock_guard<std::mutex> lock(flagMutex);
            undistortImage = !undistortImage;
        }
        
    private:
        // lens matrices used for the undistortion of
        // the image
        Mat cameraMatrix;
        Mat distortionCoefficients;
        
        std::mutex flagMutex;
        bool undistortImage;
    };
    
    class MyListener3 : public IDepthDataListener
    {
        
    public:
        
        MyListener3() :
        undistortImage(false)
        {
        }
        
        void onNewData(const DepthData *data)
        {
            // this callback function will be called for every new depth frame
            
            auto exposureTimes = data->exposureTimes;
            std::lock_guard<std::mutex> lock(flagMutex);
            
            int k = 0;
            for (int y = 0; y < data->width; y++)
            {
                for (int x = 0; x < data->height; x++, k++)
                {
                    auto curPoint = data->points.at(k);
                    double norm = sqrt(curPoint.x*curPoint.x + curPoint.y*curPoint.y + curPoint.z*curPoint.z);
                    dMax = max(dMax, norm);
                    if (curPoint.depthConfidence > 0 && norm > 0.05)
                    {
                        dataCam3[x][y][2] = GAIN * data->points.at(k).x;
                        dataCam3[x][y][1] = -GAIN * data->points.at(k).y;
                        dataCam3[x][y][0] = GAIN * data->points.at(k).z;
                        // if the point is valid, map the pixel from 3D world
                        // coordinates to a 2D plane (this will distort the image)
                    }
                    else {

                        dataCam3[x][y][1] = 0;// 500.0 * data->points.at(k).x;
                        dataCam3[x][y][0] = 0;// -500.0 * data->points.at(k).y;
                        dataCam3[x][y][2] = 0;// 500.0 * data->points.at(k).z;
                    }
                }
            }
        }
        
        void setLensParameters(const LensParameters &lensParameters)
        {
            // Construct the camera matrix
            // (fx   0    cx)
            // (0    fy   cy)
            // (0    0    1 )
            cameraMatrix = (Mat1d(3, 3) << lensParameters.focalLength.first, 0, lensParameters.principalPoint.first,
                            0, lensParameters.focalLength.second, lensParameters.principalPoint.second,
                            0, 0, 1);
            
            // Construct the distortion coefficients
            // k1 k2 p1 p2 k3
            distortionCoefficients = (Mat1d(1, 5) << lensParameters.distortionRadial[0],
                                      lensParameters.distortionRadial[1],
                                      lensParameters.distortionTangential.first,
                                      lensParameters.distortionTangential.second,
                                      lensParameters.distortionRadial[2]);
        }
        
        void toggleUndistort()
        {
            std::lock_guard<std::mutex> lock(flagMutex);
            undistortImage = !undistortImage;
        }
        
    private:
        // lens matrices used for the undistortion of
        // the image
        Mat cameraMatrix;
        Mat distortionCoefficients;
        
        std::mutex flagMutex;
        bool undistortImage;
    };
    
    class MyListener4 : public IDepthDataListener
    {
        
    public:
        
        MyListener4() :
        undistortImage(false)
        {
        }
        
        void onNewData(const DepthData *data)
        {
            // this callback function will be called for every new depth frame
            
            auto exposureTimes = data->exposureTimes;
            std::lock_guard<std::mutex> lock(flagMutex);
            
            int k = 0;
            for (int y = 0; y < data->width; y++)
            {
                for (int x = 0; x < data->height; x++, k++)
                {
                    auto curPoint = data->points.at(k);
                    double norm = sqrt(curPoint.x*curPoint.x + curPoint.y*curPoint.y + curPoint.z*curPoint.z);
                    dMax = max(dMax, norm);
                    if (curPoint.depthConfidence > 0 && norm > 0.05)
                    {
                        dataCam4[x][y][2] = GAIN * data->points.at(k).x;
                        dataCam4[x][y][1] = -GAIN * data->points.at(k).y;
                        dataCam4[x][y][0] = GAIN * data->points.at(k).z;
                        // if the point is valid, map the pixel from 3D world
                        // coordinates to a 2D plane (this will distort the image)
                    }
                    else {

                        dataCam4[x][y][1] = 0;
                        dataCam4[x][y][0] = 0;
                        dataCam4[x][y][2] = 0;
                    }
                }
            }
        }
        
        void setLensParameters(const LensParameters &lensParameters)
        {
            // Construct the camera matrix
            // (fx   0    cx)
            // (0    fy   cy)
            // (0    0    1 )
            cameraMatrix = (Mat1d(3, 3) << lensParameters.focalLength.first, 0, lensParameters.principalPoint.first,
                            0, lensParameters.focalLength.second, lensParameters.principalPoint.second,
                            0, 0, 1);
            
            // Construct the distortion coefficients
            // k1 k2 p1 p2 k3
            distortionCoefficients = (Mat1d(1, 5) << lensParameters.distortionRadial[0],
                                      lensParameters.distortionRadial[1],
                                      lensParameters.distortionTangential.first,
                                      lensParameters.distortionTangential.second,
                                      lensParameters.distortionRadial[2]);
        }
        
        void toggleUndistort()
        {
            std::lock_guard<std::mutex> lock(flagMutex);
            undistortImage = !undistortImage;
        }
        
    private:
        // lens matrices used for the undistortion of
        // the image
        Mat cameraMatrix;
        Mat distortionCoefficients;
        
        std::mutex flagMutex;
        bool undistortImage;
    };
}

int sensorToF = 0;
int CAM_NUM = 0;

int runningCamera(int mode, int argc, char *argv[])
{
    PlatformResources resources;
    
    // This is the data listener which will receive callbacks.  It's declared
    // before the cameraDevice so that, if this function exits with a 'return'
    // statement while the camera is still capturing, it will still be in scope
    // until the cameraDevice's destructor implicitly de-registers the listener.
    MyListener listener;
    MyListener2 listener2;
    MyListener3 listener3;
    MyListener4 listener4;
    
    // this represents the main camera device object
    std::unique_ptr<ICameraDevice> cameraDevice[4];
    
    LensParameters lensParameters;
    LensParameters lensParameters2;
    LensParameters lensParameters3;
    LensParameters lensParameters4;
    sensorToF = 1;
    if (sensorToF == 1) {
        
        // the camera manager will query for a connected camera
        {
            CameraManager manager;
            
            // check the number of arguments
            if (argc > 1)
            {
                // if the program was called with an argument try to open this as a file
                cout << "Trying to open : " << argv[1] << endl;
                cameraDevice[0] = manager.createCamera(argv[1]);
                cameraDevice[1] = manager.createCamera(argv[1]);
                cameraDevice[2] = manager.createCamera(argv[1]);
                cameraDevice[3] = manager.createCamera(argv[1]);
            }
            else
            {
                // if no argument was given try to open the first connected camera
                royale::Vector<royale::String> camlist(manager.getConnectedCameraList());
                cout << "Detected " << camlist.size() << " camera(s)." << endl;
                CAM_NUM = camlist.size();
                
                if (!camlist.empty())
                {
                    for (int i = 0; i < CAM_NUM; i++) {
                        cout << "CamID for first device: " << camlist.at(i).c_str() << " with a length of (" << camlist.at(i).length() << ")" << endl;
                        string A = camlist.at(i).c_str();
                        if (A == "0007-2107-0181-2211")
                            cameraDevice[0] = manager.createCamera(camlist[i]);
                        else if (A == "0007-2109-0181-1520")
                            cameraDevice[1] = manager.createCamera(camlist[i]);
                        else if (A == "0007-2107-0181-0318")
                            cameraDevice[2] = manager.createCamera(camlist[i]);
                        else if (A == "0007-2107-0181-2020")
                            cameraDevice[3] = manager.createCamera(camlist[i]);
                    }
                }
                else
                {
                    cerr << "No suitable camera device detected." << endl
                    << "Please make sure that a supported camera is plugged in, all drivers are "
                    << "installed, and you have proper USB permission" << endl;
                    return 1;
                }
                
                camlist.clear();
            }
        }
        // the camera device is now available and CameraManager can be deallocated here
        for (int i = 0; i < CAM_NUM; i++) {
            if (cameraDevice[i] == nullptr)
            {
                // no cameraDevice available
                if (argc > 1)
                {
                    cerr << "Could not open " << argv[1] << endl;
                    return 1;
                }
                else
                {
                    cerr << "Cannot create the camera device" << endl;
                    return 1;
                }
            }
        }
        
        // IMPORTANT: call the initialize method before working with the camera device
        for (int i = 0; i < CAM_NUM; i++) {
            auto status = cameraDevice[i]->initialize();
            if (status != CameraStatus::SUCCESS)
            {
                cerr << "Cannot initialize the camera device, error string : " << getErrorString(status) << endl;
                return 1;
            }
        }
        
        // retrieve the lens parameters from Royale
        auto status = cameraDevice[0]->getLensParameters(lensParameters);
        auto status2 = cameraDevice[1]->getLensParameters(lensParameters2);
        auto status3 = cameraDevice[2]->getLensParameters(lensParameters3);
        auto status4 = cameraDevice[3]->getLensParameters(lensParameters4);
        if (status != CameraStatus::SUCCESS)
        {
            cerr << "Can't read out the lens parameters" << endl;
            return 1;
        }if (status2 != CameraStatus::SUCCESS)
        {
            cerr << "Can't read out the lens parameters" << endl;
            return 1;
        }if (status3 != CameraStatus::SUCCESS)
        {
            cerr << "Can't read out the lens parameters" << endl;
            return 1;
        }if (status4 != CameraStatus::SUCCESS)
        {
            cerr << "Can't read out the lens parameters" << endl;
            return 1;
        }
        
        listener.setLensParameters(lensParameters);
        listener2.setLensParameters(lensParameters2);
        listener3.setLensParameters(lensParameters3);
        listener4.setLensParameters(lensParameters4);
        
        // register a data listener
        if (cameraDevice[0]->registerDataListener(&listener) != CameraStatus::SUCCESS)
        {
            cerr << "Error registering data listener" << endl;
            return 1;
        }
        if (cameraDevice[1]->registerDataListener(&listener2) != CameraStatus::SUCCESS)
        {
            cerr << "Error registering data listener" << endl;
            return 1;
        }
        if (cameraDevice[2]->registerDataListener(&listener3) != CameraStatus::SUCCESS)
        {
            cerr << "Error registering data listener" << endl;
            return 1;
        }
        if (cameraDevice[3]->registerDataListener(&listener4) != CameraStatus::SUCCESS)
        {
            cerr << "Error registering data listener" << endl;
            return 1;
        }
        
        // create two windows
        //namedWindow("Depth", WINDOW_AUTOSIZE);
        //namedWindow("Gray", WINDOW_AUTOSIZE);
        
        // start capture mode
        if (cameraDevice[0]->startCapture() != CameraStatus::SUCCESS)
        {
            cerr << "Error starting the capturing" << endl;
            return 1;
        }
        if (cameraDevice[1]->startCapture() != CameraStatus::SUCCESS)
        {
            cerr << "Error starting the capturing" << endl;
            return 1;
        }
        if (cameraDevice[2]->startCapture() != CameraStatus::SUCCESS)
        {
            cerr << "Error starting the capturing" << endl;
            return 1;
        }
        if (cameraDevice[3]->startCapture() != CameraStatus::SUCCESS)
        {
            cerr << "Error starting the capturing" << endl;
            return 1;
        }
        
        int currentKey = 0;
    }else{
        readSampleData();
    }
    initialExtGL();
    return 0;
}

void processData()
{
    
    int k = 0;
    for (int y = 0; y < 224; y++) {
        for (int x = 0; x < 171; x++) {
            if (dataCam2[x][y][0] > 0.00001) {
                k++;
                allDataCam[k][2] = dataCam2[x][y][2] / 1;
                double X = dataCam2[x][y][0] + 0.035 * GAIN;
                double Y = dataCam2[x][y][1];
                allDataCam[k][0] = (X * cos(M_PI * (20.0 / 180.0)) - Y * sin(M_PI * (20.0 / 180.0))) / 1;
                allDataCam[k][1] = (X * sin(M_PI * (20.0 / 180.0)) + Y * cos(M_PI * (20.0 / 180.0))) / 1;
            }
        }
    }

    for (int y = 0; y < 224; y++) {
        for (int x = 0; x < 171; x++) {
            if (dataCam4[x][y][0] > 0.00001) {
                k++;
                allDataCam[k][2] = dataCam4[x][y][2] / 1;
                double X = dataCam4[x][y][0] + 0.045 * GAIN;
                double Y = dataCam4[x][y][1];
                allDataCam[k][0] = (X * cos(M_PI * (-60.0 / 180.0)) - Y * sin(M_PI * (-60.0 / 180.0))) / 1;
                allDataCam[k][1] = (X * sin(M_PI * (-60.0 / 180.0)) + Y * cos(M_PI * (-60.0 / 180.0))) / 1;
            }
        }
    }
    for (int y = 0; y < 224; y++) {
        for (int x = 0; x < 171; x++) {
            if (dataCam3[x][y][0] > 0.00001) {
                k++;
                allDataCam[k][2] = (dataCam3[x][y][2]) / 1;
                double X = dataCam3[x][y][0] + 0.035 * GAIN;
                double Y = dataCam3[x][y][1];
                allDataCam[k][0] = (X * cos(M_PI * (-20.0 / 180.0)) - Y * sin(M_PI * (-20.0 / 180.0))) / 1;
                allDataCam[k][1] = (X * sin(M_PI * (-20.0 / 180.0)) + Y * cos(M_PI * (-20.0 / 180.0))) / 1;
            }
        }
    }
    for (int y = 0; y < 224; y++) {
        for (int x = 0; x < 171; x++) {
            if (dataCam1[x][y][0] > 0.00001) {
                k++;
                allDataCam[k][2] = (dataCam1[x][y][2]) / 1;
                double X = dataCam1[x][y][0] + 0.045 * GAIN;
                double Y = dataCam1[x][y][1];
                allDataCam[k][0] = (X * cos(M_PI * (60.0 / 180.0)) - Y * sin(M_PI * (60.0 / 180.0))) / 1;
                allDataCam[k][1] = (X * sin(M_PI * (60.0 / 180.0)) + Y * cos(M_PI * (60.0 / 180.0))) / 1;
            }
        }
    }
    ND = k;//rand()
    
    double F[3][3];
    EulerToMatrix(eulerAngle[0],eulerAngle[1],0, F);
    
    for(int i=0; i<ND; i++){
        double v[3] = {allDataCam[i][0],allDataCam[i][1],allDataCam[i][2]};
        vectorFromMatrixRotation(F, v, allDataCam[i]);
    }
}

double cameraView[4][5][3];

double cameraViewPoint[5][3] = {
    {0.0, 0.0, 0.0},
    { 0.3 * 5 * GAIN, 0.3 * 2.07 * GAIN, 0.3 * 3.008 * GAIN },
    { 0.3 * 5 * GAIN, 0.3 * -2.07 * GAIN, 0.3 * 3.008 * GAIN },
    { 0.3 * 5 * GAIN, 0.3 * -2.07 * GAIN, 0.3 * -3.008 * GAIN},
    { 0.3 * 5 * GAIN, 0.3 * 2.07 * GAIN, 0.3 * -3.008 * GAIN,}
};
void cameraRangeView(){
    int i = 0;
    for (int j = 0; j < 5; j++) {
        cameraView[i][j][2] = cameraViewPoint[j][2];
        double x = cameraViewPoint[j][0] + 0.035 * GAIN;
        double y = cameraViewPoint[j][1];
        
        cameraView[i][j][0] = x * cos(M_PI * (20.0 / 180.0)) - y * sin(M_PI * (20.0 / 180.0));
        cameraView[i][j][1] = x * sin(M_PI * (20.0 / 180.0)) + y * cos(M_PI * (20.0 / 180.0));
    }
    i = 1;
    for (int j = 0; j < 5; j++) {
        cameraView[i][j][2] = cameraViewPoint[j][2];
        double x = cameraViewPoint[j][0] + 0.035 * GAIN;
        double y = cameraViewPoint[j][1];
        
        cameraView[i][j][0] = x * cos(M_PI * (-20.0 / 180.0)) - y * sin(M_PI * (-20.0 / 180.0));
        cameraView[i][j][1] = x * sin(M_PI * (-20.0 / 180.0)) + y * cos(M_PI * (-20.0 / 180.0));
    }
    i = 2;
    for (int j = 0; j < 5; j++) {
        cameraView[i][j][2] = cameraViewPoint[j][2];
        double x = cameraViewPoint[j][0] + 0.045 * GAIN;
        double y = cameraViewPoint[j][1];
        
        cameraView[i][j][0] = x * cos(M_PI * (60.0 / 180.0)) - y * sin(M_PI * (60.0 / 180.0));
        cameraView[i][j][1] = x * sin(M_PI * (60.0 / 180.0)) + y * cos(M_PI * (60.0 / 180.0));
    }
    i = 3;
    for (int j = 0; j < 5; j++) {
        cameraView[i][j][2] = cameraViewPoint[j][2];
        double x = cameraViewPoint[j][0] + 0.045 * GAIN;
        double y = cameraViewPoint[j][1];
        
        cameraView[i][j][0] = x * cos(M_PI * (-60.0 / 180.0)) - y * sin(M_PI * (-60.0 / 180.0));
        cameraView[i][j][1] = x * sin(M_PI * (-60.0 / 180.0)) + y * cos(M_PI * (-60.0 / 180.0));
    }
//    i=0;
//    for (int j = 0; j < 5; j++) {
//        cameraView[i][j][2] = cameraViewPoint[j][2];
//        double x = cameraViewPoint[j][0] + 0.035 * GAIN;
//        double y = cameraViewPoint[j][1];
//
//        cameraView[i][j][0] = x * cos(M_PI * (0 / 180.0)) - y * sin(M_PI * (0 / 180.0));
//        cameraView[i][j][1] = x * sin(M_PI * (0 / 180.0)) + y * cos(M_PI * (0 / 180.0));
//    }
    
    double F[3][3];
    EulerToMatrix(eulerAngle[0],eulerAngle[1],0, F);
    
    for (int k = 0; k < 4; k++) {
        for (int i = 1; i < 5; i++) {
            double v[3] = {cameraView[k][i][0],cameraView[k][i][1],cameraView[k][i][2]};
            vectorFromMatrixRotation(F, v, cameraView[k][i]);
        }
        for (int i = 1; i < 5; i++) {
            int j = i + 1; if (j == 5) j = 1;
            drawLine(cameraView[k][0], cameraView[k][i], 5);
            drawLine(cameraView[k][i], cameraView[k][j], 5);
        }
    }
}
