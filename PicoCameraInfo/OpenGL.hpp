//
//  OpenGL.hpp
//  DenaulayTriangulation
//
//  Created by Azhar Aulia Saputra on 2020/09/12.
//  Copyright © 2020 Azhar Aulia Saputra. All rights reserved.
//

#ifndef OpenGL_hpp
#define OpenGL_hpp

#include <stdio.h>

void MotionEventHandler(int x, int y);
void MouseEventHandler(int button, int state, int x, int y);
void keyboard(unsigned char key, int x, int y);
void reshape(int width, int height);
void initMotionModel();
void updateGLView(float hrp[], float xyz[]);
void initialExtGL();
void drawLine(double p0[], double p1[], double s);
void drawPoint(double p[], double s);
void drawTriangle(double p0[], double p1[], double p2[]);
void drawTriangleColor(double p0[], double p1[], double p2[], double c0[], double c1[], double c2[]);
void display();
void drawBitmapText(char *string,float x,float y,float z);

#endif /* OpenGL_hpp */
