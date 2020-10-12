//
//  OpenGL.cpp
//  DenaulayTriangulation
//
//  Created by Azhar Aulia Saputra on 2020/09/12.
//  Copyright © 2020 Azhar Aulia Saputra. All rights reserved.
//

#include <opencv2/opencv.hpp>
#include <OpenGL/OpenGL.h>
#include "OpenGL.hpp"
#include <OpenGL/OpenGL.h>
#include <GLUT/GLUT.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <math.h>
#include "picoCamera.h"


int showSurface = 0;
void GetColourGL(double v, double vmin, double vmax)
{
    double dv;
    float r = 1.0, g = 1.0, b = 1.0;

    if (v < vmin)
        v = vmin;
    if (v > vmax)
        v = vmax;
    dv = vmax - vmin;

    if (v < (vmin + 0.25 * dv)) {
        r = 0;
        g = 4 * (v - vmin) / dv;
    }
    else if (v < (vmin + 0.5 * dv)) {
        r = 0;
        b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
    }
    else if (v < (vmin + 0.75 * dv)) {
        r = 4 * (v - vmin - 0.5 * dv) / dv;
        b = 0;
    }
    else {
        g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
        b = 0;
    }
    glColor3d(r, g, b);
}
void drawGridLine(){
    glLineWidth(1);
    glColor3f(0.5, 0.5, 0.5);

    for(float i = -10; i < 10; i++){
        glBegin(GL_LINES);
            glVertex2f(i, -10);
            glVertex2f(i, 10);
        glEnd();
        glBegin(GL_LINES);
            glVertex2f(-10, i);
            glVertex2f(10, i);
        glEnd();
    }
    
    glLineWidth(3);
    glColor3f(0,0, 1);
    glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(1, 0, 0);
    glEnd();
    glColor3f(0,1,0);
    glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 1, 0);
    glEnd();
    glColor3f(1,0,0);
    glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 1);
    glEnd();
}

int D_TYPE = GL_POINTS;   // drawing type

static float view_xyz[3];    // position x,y,z
static float view_hpr[3];    // heading, pitch, roll (degrees)

static void setCamera (float x, float y, float z, float h, float p, float r)
{
  glMatrixMode (GL_MODELVIEW);
  glLoadIdentity();
  glRotatef (90, 0,0,1);
  glRotatef (90, 0,1,0);
  glRotatef (r, 1,0,0);
  glRotatef (p, 0,1,0);
  glRotatef (-h, 0,0,1);
  glTranslatef (-x,-y,-z);
}
void updateGLView(float hrp[], float xyz[]){
    memcpy (view_xyz,xyz,sizeof(float)*3);
    memcpy (view_hpr,hrp,sizeof(float)*3);
}
void drawLine(double p0[], double p1[], double s){
    glLineWidth(s);
    glBegin(GL_LINES);
        glVertex3f(p0[0], p0[1], p0[2]);
        glVertex3f(p1[0], p1[1], p1[2]);
    glEnd();
}
void drawTriangle(double p0[], double p1[], double p2[]){
    glBegin(GL_TRIANGLES);
    glVertex3f(p0[0], p0[1], p0[2]);
    glVertex3f(p1[0], p1[1], p1[2]);
    glVertex3f(p2[0], p2[1], p2[2]);
    glEnd();
}
void drawTriangleColor(double p0[], double p1[], double p2[], double c0[], double c1[], double c2[]){
    glBegin(GL_TRIANGLES);
    glColor3f(c0[0], c0[1], c0[2]);
    glVertex3f(p0[0], p0[1], p0[2]);
    glColor3f(c1[0], c1[1], c1[2]);
    glVertex3f(p1[0], p1[1], p1[2]);
    glColor3f(c2[0], c2[1], c2[2]);
    glVertex3f(p2[0], p2[1], p2[2]);
    glEnd();
}
void drawPoint(double p[], double s){
    glPointSize(s);
    glBegin(GL_POINTS);
    glVertex3d(p[0],p[1],p[2]);
    glEnd();
}
void drawBitmapText(char *string,float x,float y,float z)
{
    char *c;
    glRasterPos3f(x, y,z);
    
    for (c=string; *c != NULL; c++)
    {
        glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, *c);
    }
}

void cameraSetting(){
        glClearColor(0.0,0.0,0.0, 0.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //    glLoadIdentity();
        
        // snapshot camera position (in MS Windows it is changed by the GUI thread)
        float view2_xyz[3];
        float view2_hpr[3];
        memcpy (view2_xyz,view_xyz,sizeof(float)*3);
        memcpy (view2_hpr,view_hpr,sizeof(float)*3);
        setCamera(view2_xyz[0],view2_xyz[1],view2_xyz[2],
               view2_hpr[0],view2_hpr[1],view2_hpr[2]);
}
int tLoop = 0;
int countCapture = 0;
int tUpdate = 20;
double realPos[3] = {0,0,0};
double testAngle = 0;

void reshape(int width, int height)
{
//    glViewport(0, 0, width, height);    // 画面のリサイズ（画面サイズに合わせる）
//    glMatrixMode(GL_PROJECTION);
//    glLoadIdentity();

//    gluPerspective(60.0, (double)width/(double)height, 1.0, 100000.0);   //(fovy,aspect,zNear,zFar)
//    gluLookAt(-10000.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 0.0, 1.0);//視点
    
    // setup viewport
//    glViewport (0,0,width,height);
    glViewport (0,0,2*width,2*height);
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity();
    const float vnear = 0.1f;
    const float vfar = 100.0f;
    const float k = 0.8f;     // view scale, 1 = +/- 45 degrees
    if (width >= height) {
      float k2 = float(height)/float(width);
      glFrustum (-vnear*k,vnear*k,-vnear*k*k2,vnear*k*k2,vnear,vfar);
    }
    else {
      float k2 = float(width)/float(height);
      glFrustum (-vnear*k*k2,vnear*k*k2,-vnear*k,vnear*k,vnear,vfar);
    }
    
    glMatrixMode(GL_MODELVIEW);
}
int window;
#pragma mark Timer
GLfloat top = -0.9;
int tcounter=0;
void InitGL(int Width, int Height)
{
    initMotionModel();
    glClearColor(0.0f, 0.0f, 0.0f, 0.5f);
    glEnable(GL_DEPTH_TEST);
    reshape(Width, Height);
}void cal_env(int vl)//使う
{
    static GLboolean isUp = GL_TRUE;
    
    if (top > 0.9F) isUp = GL_FALSE;
    else if (top <= -0.9F) isUp = GL_TRUE;
    top += (isUp == GL_TRUE ? 0.01 : -0.01);
    glutSetWindow(window);
    display();
    glutPostRedisplay();
    glutTimerFunc(100 , cal_env , 0);
    tcounter++;
}
void initialExtGL(){
    
    int argc = 1;
    char* argv[2];
    argv[0] = (char*)"";
    argv[1] = NULL;
    glutInit(&argc, argv);
    glutInitWindowSize(600, 800);
    glutInitWindowPosition(0, 0);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    window = glutCreateWindow("Simulation");

    glutTimerFunc(100, cal_env, 0);
    glutDisplayFunc(&display);
    glutReshapeFunc(&reshape);
    glutKeyboardFunc(&keyboard);
    glutMouseFunc(&MouseEventHandler);
    glutMotionFunc(&MotionEventHandler);
    InitGL(800, 600);
    
    glutMainLoop();
}

void keyboard(unsigned char key, int x, int y)
{
    switch (key) {

        case 's':
            if(showSurface) showSurface = 0;
            else showSurface = 1;
            break;
        case 'w':
            switch (D_TYPE) {
                case GL_POINTS:
                    D_TYPE = GL_LINE_LOOP;
                    break;
                case GL_LINE_LOOP:
                    D_TYPE = GL_POINTS;
                    break;
            }
            break;
    }
}

static int  mouseButtonMode = 0;
static bool  mouseWithOption = false;    // Set if dragging the mouse with alt pressed
static bool mouseWithControl = false;    // Set if dragging the mouse with ctrl pressed

static int prev_x = 0;
static int prev_y = 0;

int GetModifierMask()
{
    return glutGetModifiers() & ~GLUT_ACTIVE_SHIFT;
}
void MouseEventHandler(int button, int state, int x, int y)
{
    prev_x = x;
    prev_y = y;
    bool buttonDown = false;
    switch( state ){
        case GLUT_DOWN:
            buttonDown = true;
        case GLUT_UP:
            if( button == GLUT_LEFT_BUTTON ){
                int modifierMask = GetModifierMask();
                if( modifierMask & GLUT_ACTIVE_CTRL ){
                    // Ctrl+button == right
                    button = GLUT_RIGHT_BUTTON;
                    mouseWithControl = true;
                }
                else if( modifierMask & GLUT_ACTIVE_ALT ){
                    // Alt+button == left+right
                    mouseButtonMode = 5;
                    mouseWithOption = true;
                    return;
                }
            }
            if( buttonDown ){
                if( button == GLUT_LEFT_BUTTON ) mouseButtonMode |= 1;        // Left
                if( button == GLUT_MIDDLE_BUTTON ) mouseButtonMode |= 2;    // Middle
                if( button == GLUT_RIGHT_BUTTON ) mouseButtonMode |= 4;        // Right
            }
            else{
                if( button == GLUT_LEFT_BUTTON ) mouseButtonMode &= (~1);    // Left
                if( button == GLUT_MIDDLE_BUTTON ) mouseButtonMode &= (~2);    // Middle
                if( button == GLUT_RIGHT_BUTTON ) mouseButtonMode &= (~4);  // Right
            }
            return;
    }
}

// constants to convert degrees to radians and the reverse
#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD (M_PI/180.0)

static void wrapCameraAngles()
{
  for (int i=0; i<3; i++) {
    while (view_hpr[i] > 180) view_hpr[i] -= 360;
    while (view_hpr[i] < -180) view_hpr[i] += 360;
  }
}
void dsTrackMotion (int mode, int deltax, int deltay)
{
  float side = 0.01f * float(deltax);
  float fwd = (mode==4) ? (0.01f * float(deltay)) : 0.0f;
  float s = (float) sin (view_hpr[0]* DEG_TO_RAD);
  float c = (float) cos (view_hpr[0]* DEG_TO_RAD);

  if (mode==1) {
    view_hpr[0] += float (deltax) * 0.5f;
    view_hpr[1] += float (deltay) * 0.5f;
  }
  else {
    view_xyz[0] += -s*side + c*fwd;
    view_xyz[1] += c*side + s*fwd;
    if (mode==2 || mode==5) view_xyz[2] += 0.01f * float(deltay);
  }
  wrapCameraAngles();
}
void MotionEventHandler(int x, int y)
{
    dsTrackMotion( mouseButtonMode, x - prev_x, y - prev_y );
    prev_x = x;
    prev_y = y;
}
void initMotionModel()
{
  view_xyz[0] = -0.5;
  view_xyz[1] = 0;
  view_xyz[2] = 1;
  view_hpr[0] = 0;
  view_hpr[1] = -10;
  view_hpr[2] = 0;
}
void drawPointcloud(){
    
    const clock_t begin_time = clock();
    
    glPointSize(2);
    glBegin(GL_POINTS);
    for (int y = 0; y < ND; y+=4) {
        float a = sqrt(allDataCam[y][0] * allDataCam[y][0] + allDataCam[y][1] * allDataCam[y][1] + allDataCam[y][2] * allDataCam[y][2]);
        GetColourGL(1+sin(a), 0.1 * GAIN, 2 * GAIN);
        glVertex3f(allDataCam[y][0], allDataCam[y][1], allDataCam[y][2]);
    }
    
    glEnd();
    
    printf("point cloud drawing cost: %.6f\n",float( clock () - begin_time ) /  CLOCKS_PER_SEC);
}
int showPointCloud = 1;
void display()
{
    cameraSetting();
/*----------------------------------------------------------*/
    
    if(tLoop == 0){
        
        
    }
    if(sensorToF){
        processData();
        cameraRangeView();
        if (showPointCloud == 1) {
            drawPointcloud();
        }
    }
    readSenserIMU();
/*----------------------------------------------------------*/
    
    drawGridLine();
    
    tLoop++;
    glutSwapBuffers();
    glFlush();
    
}
