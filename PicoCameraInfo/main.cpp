/****************************************************************************\
 * Copyright (C) 2017 Infineon Technologies & pmdtechnologies ag
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 \****************************************************************************/

#include <OpenGL/OpenGL.h>
#include <GLUT/GLUT.h>
#include <royale.hpp>
#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <sample_utils/PlatformResources.hpp>
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include <sys/types.h>
#include <pthread.h>
#include "picoCamera.h"
#include "NgimuReceive.h"

using namespace royale;
using namespace sample_utils;
using namespace std;
using namespace cv;

int fd,fd2;
int set_interface_attribs (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        printf ("error from tcgetattr");
        return -1;
    }
    
    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);
    
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // ignore break signal
    tty.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
    
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    
    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    
    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        printf ("error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}

void set_blocking (int fd, int should_block)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        printf ("error %d from tggetattr", errno);
        return;
    }
    
    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 1;                // 0.5 seconds read timeout
    
    if (tcsetattr (fd, TCSANOW, &tty) != 0)
        printf ("error %d setting term attributes", errno);
}




void ngimuReceiveErrorCallback(const char* const errorMessage) {
//    printf("/error, ");
//    Serial.print(errorMessage);
//    Serial.print("\r\n");
}

// This function is called each time a "/sensors" message is received
void ngimuSensorsCallback(const NgimuSensors ngimuSensors) {
//    printf("/sensors, ");
//    send("/sensors, ");
//    Serial.print(ngimuSensors.gyroscopeX);
//    Serial.print(", ");
//    Serial.print(ngimuSensors.gyroscopeY);
//    Serial.print(", ");
//    Serial.print(ngimuSensors.gyroscopeZ);
//    Serial.print(", ");
//    Serial.print(ngimuSensors.accelerometerX);
//    Serial.print(", ");
//    Serial.print(ngimuSensors.accelerometerY);
//    Serial.print(", ");
//    Serial.print(ngimuSensors.accelerometerZ);
//    Serial.print(", ");
//    Serial.print(ngimuSensors.magnetometerX);
//    Serial.print(", ");
//    Serial.print(ngimuSensors.magnetometerY);
//    Serial.print(", ");
//    Serial.print(ngimuSensors.magnetometerZ);
//    Serial.print(ngimuSensors.barometer);
//    Serial.print("\r\n");
}
NgimuEuler toEulerAngle(const NgimuQuaternion q)
{
    NgimuEuler euler;
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    euler.roll = atan2(sinr_cosp, cosr_cosp);
    
    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        euler.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        euler.pitch = asin(sinp);
    
    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    euler.yaw = atan2(siny_cosp, cosy_cosp);
    
    return euler;
}
NgimuEuler ngimuEuler;
// This function is called each time a "/quaternion" message is received
void ngimuQuaternionCallback(const NgimuQuaternion ngimuQuaternion) {
//    printf("/quaternion, ");
////    Serial.print("/quaternion, ");
//    printf(", %f",ngimuQuaternion.w);
//    printf(", %f",ngimuQuaternion.x);
//    printf(", %f",ngimuQuaternion.y);
//    printf(", %f",ngimuQuaternion.z);
//    printf("\r\n");
    ngimuEuler = toEulerAngle(ngimuQuaternion);
    
    printf("Euler:\t %.2f,\t %.2f,\t %.2f\n", rad*ngimuEuler.pitch, rad*ngimuEuler.roll, rad*ngimuEuler.yaw);
    
}
NgimuEarth outEarthAcc;
void ngimuEarthCallback(const NgimuEarth ngimuEarth) {
    float min = 100, max = -100;
    double timestep = 1.0/100.0;
    
    
    outEarthAcc.x = ngimuEarth.x * 9.81;
    outEarthAcc.y = ngimuEarth.y * 9.81;
    outEarthAcc.z = ngimuEarth.z * 9.81;
//    if(ngimuEarth.x < 0.03 && ngimuEarth.x > -0.03) outEarthAcc.x = 0.0;
    
    if(outEarthAcc.x > max) max = outEarthAcc.x;
    if(outEarthAcc.x < min) min = outEarthAcc.x;
    
    velo.x = velo0.x + timestep * (ngimuEarth0.x + outEarthAcc.x)/2;
    velo.y = velo0.y + timestep * (ngimuEarth0.y + outEarthAcc.y)/2;
    velo.z = velo0.z + timestep * (ngimuEarth0.z + outEarthAcc.z)/2;
    
    dis.x = dis.x + timestep * (velo0.x + velo.x)/2;
    dis.y = dis.y + timestep * (velo0.y + velo.y)/2;
    dis.z = dis.z + timestep * (velo0.z + velo.z)/2;
    
    velo0.x = velo.x;
    velo0.y = velo.y;
    velo0.z = velo.z;
    
    ngimuEarth0.x = outEarthAcc.x;
    ngimuEarth0.y = outEarthAcc.y;
    ngimuEarth0.z = outEarthAcc.z;
    
    
//    printf("/earth,\t%u\t%f\t%f\t%f\t%f\t%f\t%f\n",ngimuEarth.timestamp.dwordStruct.seconds, dis.x, min, max, ngimuEarth.x, ngimuEarth.y, ngimuEarth.z);
}
// This function is called each time a "/euler" message is received.
double eulerAngle[3] = {0,0,0};
void ngimuEulerCallback(const NgimuEuler ngimuEuler) {
//    printf("/euler,\t%u\t%f\t%f\t%f\n", ngimuEuler.timestamp.dwordStruct.seconds ,ngimuEuler.roll, ngimuEuler.pitch,ngimuEuler.yaw);
    eulerAngle[0] = ngimuEuler.roll / rad;
    eulerAngle[1] = ngimuEuler.pitch / rad;
    eulerAngle[2] = ngimuEuler.yaw / rad;
}

bool enable_sensor = false;
bool sensorIMU = true;
void *thread_InSensor(void *arg)
{
    char buf [1000];
    char *buffer1;
    buffer1 = (char *) calloc(100, sizeof(char));
    int n;
    
    if(sensorIMU){
        char *portname = "/dev/cu.usbmodem1234567899992";
            
        fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0)
        {
            printf("error %d opening %s: %s", errno, portname, strerror (errno));
        }else{
        
            set_interface_attribs (fd, B115200, 0);        // set speed to 115,200 bps, 8n1 (no parity)
            set_blocking (fd, 0);                        // set no blocking
                    
            NgimuReceiveInitialise();
            NgimuReceiveSetEulerCallback(ngimuEulerCallback);
            NgimuReceiveSetEarthCallback(ngimuEarthCallback);
        }
    }
    while(1){
        n = read(fd, buf, sizeof buf);
        if(n > 0){
            int i;
            for (i = 0; i < n; i++) {
                NgimuReceiveProcessSerialByte(buf[i]);
            }
            n = 0;
        }
        
    }
    return arg;
}
void readSenserIMU(){
    char buf [1000];
    int n=0,i;
    n = read(fd, buf, sizeof buf);
    if(n > 0){
        for (i = 0; i < n; i++) {
            NgimuReceiveProcessSerialByte(buf[i]);
        }
        n = 0;
    }
}
void *thread_Main(void *arg)
{
    runningCamera(0, 0, 0);
    return arg;
}
int threadInit(){
    
    pid_t   p_pid;
    pthread_t    thread_id1,thread_id2,thread_id3;
    int    status;
    void     *result;
    
    printf("[%d]start\n",p_pid);
    
    status=pthread_create(&thread_id1,NULL,thread_Main,(void *)NULL);
    if(status!=0){
        fprintf(stderr,"pthread_create : %s",strerror(status));
    }
    status=pthread_create(&thread_id2,NULL,thread_InSensor,(void *)NULL);
    if(status!=0){
        fprintf(stderr,"pthread_create : %s",strerror(status));
    }
    
    pthread_join(thread_id1,&result);
    //    printf("[%d]thread_id1 = %d end\n",p_pid,(int)thread_id1);
    pthread_join(thread_id2,&result);
    //    printf("[%d]thread_id2 = %d end\n",p_pid,(int)thread_id2);
    
    printf("[%d]end\n",p_pid);
    
    
    return 0;
}

int readSampleData(){
    
    double tx;
//    FILE *fp = fopen("/Users/azhar/Data/330.dat", "r");
//    FILE *fp = fopen("/Users/azhar/Data/data_4.dat", "r");
    FILE *fp = fopen("data_capture_2.dat", "r");
//    FILE *fp = fopen("/Users/azhar/data.dat", "r");
    int ct = 0;
    
    printf("\nStart to read data.\n");
    while(1){
        // for decided the starting data scan
        for(int i=0;i<100;i++){
            ND = ct*100 + i;
            for(int j=0;j<2;j++){
                if(fscanf(fp, "%lf,",&tx) == EOF){
                    return 0;
                }
                dataCamInt[ct*100 + i][j] = tx;
                printf("%.1f,",tx);
            }for(int j=0;j<1;j++){
                if(fscanf(fp, "%lf;",&tx) == EOF){
                    return 0;
                }
                dataCamInt[ct*100 + i][2] = tx;
                printf("%.1f;",tx);
            }
            printf("");
        }
        
        printf("\n");
        ct++;
    }
    
    fclose(fp);
    
    printf("Finish reading data.\n\n");
    return 0;
    
}
int main (int argc, char *argv[])
{
    
    if(sensorIMU){
        char *portname = "/dev/cu.usbmodem1234567899992";
            
        fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0)
        {
            printf("error %d opening %s: %s", errno, portname, strerror (errno));
        }else{
        
            set_interface_attribs (fd, B115200, 0);        // set speed to 115,200 bps, 8n1 (no parity)
            set_blocking (fd, 0);                        // set no blocking
                    
            NgimuReceiveInitialise();
            NgimuReceiveSetEulerCallback(ngimuEulerCallback);
            NgimuReceiveSetEarthCallback(ngimuEarthCallback);
        }
    }
    
    runningCamera(0, 0, 0);
    threadInit();
    
    
    return 0;
}
