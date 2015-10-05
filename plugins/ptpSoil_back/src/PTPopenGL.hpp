#ifndef __PTPOPENGL_H__
#define __PTPOPENGL_H__
//#include <windows.h>
//#include <glut.h>
#include <stdio.h>
#include <stdlib.h>
#include <GL/glut.h>
#include <GL/gl.h>
#include "PTPSystem.hpp"
//#pragma comment( lib, "C:/Library/glut-3.7/glut32" )
#include <string.h>
#include "KalmanFilter.hpp"
#include <iostream>
#include <time.h>

#define I_NON 0
#define I_TRS 1
#define I_ROT 2
#define I_MAG 3
#define BUF_SIZE 800

using namespace soildynamics;
//namespace soildynamics {

	typedef struct EXPDATA{
		int msec; 
		double Joint0; 
		double Joint1; 
		double Joint2; 
		double Joint3; 
		double Joint4; 
		double Joint5; 
		double Joint6; 
		double 	Fx; 
		double Fy; 
		double	Fz; 
		double Tx; 
		double Ty; 
		double Tz; 
		int FTStatus; 
		int RTDSeq; 
		int FTSeq; 
		int Marker; 
		int currentPointID; 
		double X; 
		double Y; 
		double Z; 
		double Rx; 
		double Ry; 
		double Rz;
	}EXPDATA;

	void MyTimer(int Value);
	static void out_string_at(void *font, float x, float y, float z, char *string);
	

	void ReadExperimentData(int type);
	void DoReshape(GLsizei width, GLsizei height);
	void DoDisplay(void);
	void DoKeyboard(unsigned char key, int x, int y);
	void DoSpecial(int key, int x, int y);
	void DoMouse(int button, int state, int x, int y);
	void DoMouseMotion(int x, int y );
	void DoMenu(int value);
	void printFOOT();
	
//}// end of namespace soildynamics

#endif 
