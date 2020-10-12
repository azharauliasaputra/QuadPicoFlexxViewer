//
//  projection.cpp
//  SimplePP-1
//
//  Created by Azhar Aulia Saputra on 4/28/16.
//  Copyright © 2016 Azhar Aulia Saputra. All rights reserved.
//

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <GLUT/glut.h>
#include <OpenGL/gl.h>

#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include "projection.h"

//#define CENTER (STARTMAP + (SCALE * sizeMap)/2)
#define CENTER 400

float dataTry[3] = {10,10,10};

float **MatrixMultiplication(float **m1, float **m2, int c){
    int i,j,k,l;
    float sum;
    float **out = (float **) malloc(sizeof (float *) * c);
    for (int i = 0; i < c; i++) {
        out[i] = (float *) malloc(sizeof (float) * c);
    }
    
    for(i=0; i<c; i++){
        for(j=0; j<c; j++){
            sum = 0;
            for(k=0; k<c; k++){
                sum += m1[i][k]*m2[k][j];
            }
            out[i][j] = sum;
        }
    }
    
    return out;
}


void DrawVector(int x1, int y1, int x2, int y2){
    
    glBegin(GL_LINES);
    glColor3f(0, 0, 0);
    glVertex2s(x1,y1);
    glVertex2s(x2,y2);
    glEnd();
}

float *vectorAdd(float v1[], float v2[], int j){
    float *v = (float *) malloc(sizeof (float) * j);
    
    for(int i = 0; i<j; i++)
        v[i] = v1[i] + v2[i];
    return v;
}double *vectorAddD(double *v1, double *v2, int j){
    double *v = (double *) malloc(sizeof (double) * j);
    
    for(int i = 0; i<j; i++)
        v[i] = v1[i] + v2[i];
//    free(v1);
//    free(v2);
    return v;
}float *vectorSubtraction(float *v1, float *v2, int j){
    float *v = (float *) malloc(sizeof (float) * j);
    
    for(int i = 0; i<j; i++)
        v[i] = v1[i] - v2[i];
    return v;
}double *vectorSubtractionD(double *v1, double *v2, int j){
    double *v = (double *) malloc(sizeof (double) * j);
    
    for(int i = 0; i<j; i++)
        v[i] = v1[i] - v2[i];
    return v;
}float *vectorProduct(float v1[], float v2[], int j){
    float *v = (float *) malloc(sizeof (float) * j);
    
    for(int i = 0; i<j; i++)
        v[i] = v1[i] * v2[i];
    return v;
}double *vectorProductD(double v1[], double v2[], int j){
    double *v = (double *) malloc(sizeof (double) * j);
    
    for(int i = 0; i<j; i++)
        v[i] = v1[i] * v2[i];
    return v;
}float *vectorScale(float a, float v2[], int j){
    float *v = (float *) malloc(sizeof (float) * j);
    
    for(int i = 0; i<j; i++){
        v[i] = a * v2[i];
    }
    return v;
}double *vectorScaleD(double a, double v2[], int j){
    double *v = (double *) malloc(sizeof (double) * j);
    
    for(int i = 0; i<j; i++){
        v[i] = a * v2[i];
    }
    return v;
}

float cosValue(float v1[], float v2[], int p){
    float a, b, out;
    a = dotProductPar(v1, v2, p);
    b = vectorValue2(v1, p) * vectorValue2(v2, p);
    
    out = a/b;
    
    return out;
}
double dotProductParD(double v1[], double v2[], int p){
    double v=0;
    for(int i=0; i < p; i++){
        v += v1[i]*v2[i];
    }
    return v;
}float dotProductPar(float v1[], float v2[], int p){
    float v=0;
    for(int i=0; i < p; i++){
        v += v1[i]*v2[i];
    }
    return v;
}float *redProductPar(float v1[], float v2[], int p){
    float *v = (float *) malloc(sizeof (float) * 3);
    for(int i=0; i < p; i++){
        v[i] = v1[i] - v2[i];
    }
    return v;
}float *addProductPar(float v1[], float v2[], int p){
    float *v = (float *) malloc(sizeof (float) * 3);
    for(int i=0; i < p; i++){
        v[i] = v1[i] + v2[i];
    }
    return v;
}float dotProduct(float v1[], float v2[]){
    float v;
    v = v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
    return v;
}

float *crossProduct(float *v1, float *v2){
    float *v = (float *) malloc(sizeof (float) * 3);
    v[0] = v1[1]*v2[2] - v2[1]*v1[2];
    v[1] = -(v1[0]*v2[2] - v2[0]*v1[2]);
    v[2] = v1[0]*v2[1] - v2[0]*v1[1];
    return v;
}
float *Angle3DVector(float *x1, float *x2, float *y1, float *y2, float *z1, float *z2){
    float *v = (float *) malloc(sizeof (float) * 3);
    float *v1 = (float *) malloc(sizeof (float) * 3);
    float *v2 = (float *) malloc(sizeof (float) * 3);
    float **dv1=(float **) malloc(3*sizeof(float *));
    for(int i=0;i<3;i++)
        dv1[i]=(float *) malloc(3*sizeof(float));
    float **dv2=(float **) malloc(3*sizeof(float *));
    for(int i=0;i<3;i++)
        dv2[i]=(float *) malloc(3*sizeof(float));
    
    dv1[0] = vectorUnit(x1);
    dv1[1] = vectorUnit(y1);
    dv1[2] = vectorUnit(z1);
    dv2[0] = vectorUnit(x2);
    dv2[1] = vectorUnit(y2);
    dv2[2] = vectorUnit(z2);
    
    v1[0] = atan2(dv1[0][2], dv1[0][1]);
    v1[1] = atan2(dv1[1][1], dv1[1][0]);
    v1[2] = atan2(dv1[2][2], dv1[2][0]);
    
    v2[0] = atan2(dv2[0][2], dv2[0][1]);
    v2[1] = atan2(dv2[1][1], dv2[1][0]);
    v2[2] = atan2(dv2[2][2], dv2[2][0]);
    
    for(int i=0;i<3;i++){
        v[i] = v2[i] - v1[i];
        if(v[i] > M_PI) v[i] -= 2*M_PI;
        else if(v[i] < -M_PI) v[i] += 2*M_PI;
    }
    printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", dv1[2][0], dv1[2][1], dv1[2][2], dv2[2][0], dv2[2][1], dv2[2][2]);
    printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", v1[0], v2[0], v1[1], v2[1], v1[2], v2[2]);
    return v;
}
float vectorValue(float v1[]){
    float v;
    v = sqrt(v1[0]*v1[0] + v1[1]*v1[1] + v1[2]*v1[2]);
    return v;
}double vectorValueD(double v1[]){
    double v;
    v = sqrt(v1[0]*v1[0] + v1[1]*v1[1] + v1[2]*v1[2]);
    return v;
}float vectorValue2(float v1[], int p){
    float v, a = 0;
    for(int i=0; i < p; i++){
        a += v1[i]*v1[i];
    }
    v = sqrt(a);
    return v;
}float *vectorUnit(float v1[]){
    float *v = (float *) malloc(sizeof (float) * 3);
    float a;
    a = vectorValue(v1);
    v[0] = v1[0]/a;
    v[1] = v1[1]/a;
    v[2] = v1[2]/a;
    
    return v;
}double *vectorUnitD(double *v1){
    double *v = (double *) malloc(sizeof (double) * 3);
    double a;
    a = vectorValueD(v1);
    v[0] = v1[0]/a;
    v[1] = v1[1]/a;
    v[2] = v1[2]/a;
//    free(v1);
    return v;
}float *centerPointTriangle(double v1[], double v2[], double v3[]){
    float *v = (float *) malloc(sizeof (float) * 3);
    
    v[0] = (v1[0] + v2[0] + v3[0])/3;
    v[1] = (v1[1] + v2[1] + v3[1])/3;
    v[2] = (v1[2] + v2[2] + v3[2])/3;
    
    return v;
}

float *vectorRed(float v1[], float v2[]){
    float *v = (float *) malloc(sizeof (float) * 3);
    v[0] = v1[0] - v2[0];
    v[1] = v1[1] - v2[1];
    v[2] = v1[2] - v2[2];
    return v;
}

float *location(float *Neuron0, float *Neuron1, float b, float c){
    float *pos1 = (float *) malloc(sizeof (float) * 3);
    pos1[0] = Neuron0[0] + (Neuron1[0] - Neuron0[0]) * (c/b);
    pos1[1] = Neuron0[1] + (Neuron1[1] - Neuron0[1]) * (c/b);
    pos1[2] = Neuron0[2] + (Neuron1[2] - Neuron0[2]) * (c/b);
    
    return pos1;
}
float *rotation_x(float a, float *pos){
    float *pos1 = (float *) malloc(sizeof (float) * 3);
    
    pos1[0] = pos[0];
    pos1[1] = pos[1] * cos(a) - pos[2] * sin(a);
    pos1[2] = pos[1] * sin(a) + pos[2] * cos(a);
    
    return pos1;
}float *rotation_y(float a, float *pos){
    float *pos1 = (float *) malloc(sizeof (float) * 3);
    
    pos1[0] = pos[2] * sin(a) + pos[0] * cos(a);
    pos1[1] = pos[1];
    pos1[2] = pos[2] * cos(a) - pos[0] * sin(a);
    
    return pos1;
}void rotation_x2(float a, double pos[3], double pos1[3]){
    
    pos1[0] = pos[0];
    pos1[1] = pos[1] * cos(a) - pos[2] * sin(a);
    pos1[2] = pos[1] * sin(a) + pos[2] * cos(a);
    
}void rotation_y2(float a, double pos[3], double pos1[3]){
    
    pos1[0] = pos[2] * sin(a) + pos[0] * cos(a);
    pos1[1] = pos[1];
    pos1[2] = pos[2] * cos(a) - pos[0] * sin(a);
    
}void rotation_z2(float a, double pos[3], double pos1[3]){
    
    pos1[0] = pos[0] * cos(a) + pos[1] * sin(a);
    pos1[1] = - pos[0] * sin(a) + pos[1] * cos(a);
    pos1[2] = pos[2];
    
}float *rotation_z(float a, float *pos){
    float *pos1 = (float *) malloc(sizeof (float) * 3);
    
    pos1[0] = pos[0] * cos(a) - pos[1] * sin(a);
    pos1[1] = pos[0] * sin(a) + pos[1] * cos(a);
    pos1[2] = pos[2];
    
    return pos1;
}
float *outputMatrixRotation(float *pos, float *pos0, float ax,float ay,float az){
    float *pos1 = (float *) malloc(sizeof (float) * 3);
    pos1[0] = pos[0] - pos0[0];
    pos1[1] = pos[1] - pos0[1];
    pos1[2] = pos[2] - pos0[2];
    
    pos1 = rotation_x(ax, pos1);
    pos1 = rotation_y(ay, pos1);
    pos1 = rotation_z(az, pos1);
    
    pos1[0] = pos1[0] + pos0[0];
    pos1[1] = pos1[1] + pos0[1];
    pos1[2] = pos1[2] + pos0[2];
    
    //    printf("%.3f\t%.3f\t%.3f\n",pos1[0], pos1[1], pos1[2]);
    //    DrawVector(pos1[0]*10 + 1000, pos1[1]*10 + 200 , 0 + 1000,0 + 200);
    return pos1;
}

float *Rotation3D(float *R, float *pos0){
    float *pos1 = (float *) malloc(sizeof (float) * 3);
    
    pos1[0] = pos0[0]*R[0] + pos0[1]*R[1] + pos0[2]*R[2];
    pos1[1] = pos0[0]*R[4] + pos0[1]*R[5] + pos0[2]*R[6];
    pos1[2] = pos0[0]*R[8] + pos0[1]*R[9] + pos0[2]*R[10];

    return pos1;
}
float *Rotation3D_2(float *Theta, float *pos0){
    float *pos1 = (float *) malloc(sizeof (float) * 3);
    float *pos2 = (float *) malloc(sizeof (float) * 3);
    float *pos3 = (float *) malloc(sizeof (float) * 3);
    
//    printf("%.3f\t%.3f\t%.3f\t", Theta[0], Theta[1], Theta[2]);
//    printf("%.3f\t%.3f\t%.3f\n", pos0[0], pos0[1], pos0[2]);
    
//    Theta[1] = 0;
//    Theta[2] = 0;
    
    pos1[0] = pos0[0];
    pos1[1] = pos0[1]*cos(Theta[0]) - pos0[2]*sin(Theta[0]);
    pos1[2] = pos0[1]*sin(Theta[0]) + pos0[2]*cos(Theta[0]);
    
    pos2[0] = pos1[0]*cos(Theta[1]) + pos1[2]*sin(Theta[1]);
    pos2[1] = pos1[1];
    pos2[2] = -pos1[0]*sin(Theta[1]) + pos1[2]*cos(Theta[1]);

    pos3[0] = pos2[0]*cos(Theta[2]) - pos2[1]*sin(Theta[2]);
    pos3[1] = pos2[0]*sin(Theta[2]) + pos2[1]*cos(Theta[2]);
    pos3[2] = pos2[2];

    
    return pos3;
}
float norm(float V[], int j){
    float sum = 0, out;
    
    for(int i = 0; i < j; i++){
        sum += V[i]*V[i];
    }
    out = sqrt(sum);
    return out;
}
double normD(double V[], int j){
    double sum = 0, out;
    
    for(int i = 0; i < j; i++){
        sum += V[i]*V[i];
    }
    out = sqrt(sum);
    return out;
}
void EulerToMatrix(
            double roll,     //Yaw   angle (radians)
            double pitch,   //Pitch angle (radians)
            double yaw, double A[3][3] )   //Roll  angle (radians)
{
        //Precompute sines and cosines of Euler angles
        double su = sin(roll);
        double cu = cos(roll);
        double sv = sin(pitch);
        double cv = cos(pitch);
        double sw = sin(yaw);
        double cw = cos(yaw);
        
        A[0][0] = cv*cw;
        A[0][1] = su*sv*cw - cu*sw;
        A[0][2] = su*sw + cu*sv*cw;
        A[1][0] = cv*sw;
        A[1][1] = cu*cw + su*sv*sw;
        A[1][2] = cu*sv*sw - su*cw;
        A[2][0] = -sv;
        A[2][1] = su*cv;
        A[2][2] = cu*cv;
}
int vectorFromMatrixRotation(double a[3][3], double pos0[], double pos1[])
{
//    double a[3][3];
//
//    for(int i=0; i<3; i++){
//        a[0][i] = R[0+i];
//        a[1][i] = R[4+i];
//        a[2][i] = R[8+i];
//    }
    int i,j;
    double sum;
    
    for(i=0;i<3;i++)
    {
        sum=0;
        for(j=0;j<3;j++)   {
            sum=sum+a[i][j]*pos0[j];
//            printf("%.2f\t%.2f\t",a[i][j],pos0[j]);
        }
        pos1[i]=sum;
    }
//     for(i=0;i<3;i++)
//     {
//         printf("%.2f\t",pos0[i]);
//         printf("%.2f\t",pos1[i]);
//         printf("\n");
//     }
    return 0;
}
