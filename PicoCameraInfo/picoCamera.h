//
//  picoCamera.hpp
//  PicoCameraInfo
//
//  Created by Azhar Aulia Saputra on 10/25/18.
//  Copyright © 2018 Azhar Aulia Saputra. All rights reserved.
//

#ifndef picoCamera_h
#define picoCamera_h


#define rad 57.295779513082320876798154814105
#define phi 3.1415926535897932384626433832795
#define sp2 1.4142135623730950488016887242097
#define _USE_MATH_DEFINES // for C++
#include <cmath>

#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <iostream>
#include <vector>
#include "NgimuReceive.h"

#define iteLength 40
//#define maxLength 100
#define minLength 1
#define CENTER_X 0
#define CENTER_Y 0
#define CENTER_Z 0

#define iteDirection 1
#define maxDirection (phi/8)
#define minDirection (-phi/8)


#define GAIN 1


extern int ND;
extern bool enable_sensor;
extern int fd,fd2;
extern char buffer[10000];
extern int dstSocket;
extern bool CONNECT;
extern double dataCam1[171][224][3];
extern double dataCam2[171][224][3];
extern double dataCam3[171][224][3];
extern double dataCam4[171][224][3];

extern double dataCamInt[4 * 171 * 224][3];

extern double allDataCam[4 * 171 * 224][3];

extern float *jointAngle;
extern float **jointPos, **jointPos1;
extern int flag;
extern int sensorToF;


int set_interface_attribs (int fd, int speed, int parity);
void set_blocking (int fd, int should_block);
int threadInit();
void ngimuReceiveErrorCallback(const char* const errorMessage);
void ngimuSensorsCallback(const NgimuSensors ngimuSensors);
void ngimuQuaternionCallback(const NgimuQuaternion ngimuQuaternion);
void ngimuEulerCallback(const NgimuEuler ngimuEuler);
void cameraRangeView();
int readSampleData();
int runningCamera(int mode, int argc, char *argv[]);
void processData();
extern int D_TYPE;   // drawing type
extern double rotate_x, rotate_y, rotate_z;
extern double zoom, area_size, axis_leng ;
void readSenserIMU();
extern int showPointCloud;
extern bool SENDDATA,CONNECT;

extern NgimuEuler ngimuEuler;
extern double eulerAngle[3];
class SolidSphere
{
protected:
    std::vector<float> vertices;
    std::vector<float> normals;
    std::vector<float> texcoords;
    std::vector<ushort> indices;
public:
    SolidSphere(float radius, unsigned int rings, unsigned int sectors);
    void draw(float x, float y, float z);
};


#endif /* picoCamera_hpp */
