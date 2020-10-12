//
//  projection.hpp
//  SimplePP-1
//
//  Created by Azhar Aulia Saputra on 4/28/16.
//  Copyright © 2016 Azhar Aulia Saputra. All rights reserved.
//

#ifndef projection_hpp
#define projection_hpp

#include <stdio.h>
float *centerPointTriangle(double v1[], double v2[], double v3[]);
float **MatrixMultiplication(float **m1, float **m2, int c);

float dotProduct(float v1[], float v2[]);
float dotProductPar(float v1[], float v2[], int p);
float vectorValue(float v1[]);
float vectorValue2(float v1[], int p);
float *vectorRed(float v1[], float v2[]);
float *vectorUnit(float v1[]);
float *redProductPar(float v1[], float v2[], int p);
float *addProductPar(float v1[], float v2[], int p);
float cosValue(float v1[], float v2[], int p);
float *outputMatrixRotation(float *pos, float *pos0, float ax,float ay,float az);

float *vectorSubtraction(float *v1, float *v2, int j);
float *vectorAdd(float v1[], float v2[], int j);
float *vectorProduct(float v1[], float v2[], int j);
float *vectorScale(float a, float v2[], int j);
float *Angle3DVector(float *x1, float *x2, float *y1, float *y2, float *z1, float *z2);
float *Rotation3D(float *R, float *pos0);
float norm(float V[], int j);
double normD(double V[], int j);
float *crossProduct(float *v1, float *v2);

double *vectorSubtractionD(double *v1, double *v2, int j);
double *vectorUnitD(double v1[]);
double *vectorAddD(double *v1, double *v2, int j);
double dotProductParD(double v1[], double v2[], int p);
double *vectorProductD(double v1[], double v2[], int j);
double *vectorScaleD(double a, double v2[], int j);

void EulerToMatrix(
            double roll,     //Yaw   angle (radians)
            double pitch,   //Pitch angle (radians)
                   double yaw, double A[3][3] );  //Roll  angle (radians)
int vectorFromMatrixRotation(double a[3][3], double pos0[], double pos1[]);

void rotation_x2(float a, double pos[3], double pos1[3]);
void rotation_z2(float a, double pos[3], double pos1[3]);
void rotation_y2(float a, double pos[3], double pos1[3]);

#endif /* projection_hpp */
