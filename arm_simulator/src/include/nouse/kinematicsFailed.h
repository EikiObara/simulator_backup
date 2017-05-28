#pragma once

#ifndef __kinematics_H_INCLUDED__
#define __kinematics_H_INCLUDED__

#define _USE_MATH_DEFINES

#include <stdio.h>
#include <math.h>

typedef struct {
	int row;
	int column;
	double *m;
}Matrix;

#define FREE_ARG char*
const int MAXJOINT = 8; //7 axis and hand tip DoF
const int NR_END = 1;
const double PI = 3.14159265359;
const double COMPARE_ZERO = 1e-9;

//シミュレータ側で使用する関数
//*
//速度計算1ループごとに進む時間
const double TIME_SPAN = 0.001;

//計算結果の描画を何回に一回にするか
const double LOOP_SPAN = TIME_SPAN * 100;

//台形補間の計算総時間
const double TIME_LENGTH = 2.0;

//台形補間の加減速時間
const double ACCEL_TIME = TIME_LENGTH / 4; //総時間の25%

//手先位置収束しきい値
const double FINGER_THRESHOLD = 0.001;

//スペースマウスの値強度
const double SPACEMOUSE_TRANSLATION_GAIN = 0.0f;
const double SPACEMOUSE_ROTATION_GAIN = 0.0005f;

//
const double HAND_POSTURE_GAIN = 0.01;
//*/

/////////////////////////////////////////////////

Matrix *CreateDiMatrix(const int row, const int column);
Matrix *CreateTriMatrix(const int row, const int column, const int numMat);

void FreeMatrix(Matrix* matrix);
double *SetVector(int nl);
void FreeVector(double *v);

void CalcHTM(Matrix *OM, Matrix *HTMMat);

class kinematics {
public:
	//肘:elbow:e,手首:wrist:w,手先:finger:f
	double eX, eY, eZ;
	double wX, wY, wZ;
	double X, Y, Z;
public:
	kinematics();
	~kinematics();
	void DisplayCoordinate();
	void GetElbowCoordinate(double *ec);
	void GetwristCoordinate(double *wc);
	void GetCoordinate(double *fc);
	void CalcFK(double *currentRadian, double *currentCoordinate);
	int CalcIK(double *currentRadian, double *handVelocity, double *nextRadVerocity, bool selfMotion);
	void kinematics::VelocityRegulation(const double *targetCoordinate, double *velocity, double *currentTime);
};

void Dbar();

#endif