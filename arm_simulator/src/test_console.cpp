// 2017/04/28
// created by eiki obara
#include "include\Test_console.h"

//この下にテストしたいプログラムのインクルードを書こう
#include "include\kine_debag.h"
#include "include\kine_quat.h"
#include "include\trapezoidal_interpolation.h"
#include "include\kine_config.h"
#include "include\kine_trajectory.h"

#include <stdio.h>

double outQ[4 * 100000] = {};
double outM[16 * 100000] = {};
double outE[3 * 100000] = {};

void TestConsole(void){

	double f[3] = { 1,2,3 };
	double v[3] = { 4,5,6 };
	double e[3] = { 9,8,7 };

	for (double i = 0; i < 1; i += 0.1) {
		double buf[3] = {};
		TrapeInterpolate(1, 1, i);
		CalcVelocitySpline(f, v, e, i, buf);
		DisplayVector(3, buf);
	}
}


/*
Quat init(0.5, -0.5, 0.5, 0.5);
Quat end(0.653281, 0.270598, 0.270598, 0.653281);
Quat current;

double recQ[4] = {};
double recM[16] = {};
double recE[3] = {};

int counter = 0;

double currentTime = 0;

for (double i = 0; i < 1; i += TIME_SPAN) {

	current = init.slerp(i, end);
	current.Quat2array(recQ);
	for (int j = 0; j < 4; ++j) outQ[4 * counter + j] = recQ[j];
	current.quat2RotM(recM);
	for (int j = 0; j < 16; ++j) outM[16 * counter + j] = recM[j];
	current.quat2Euler(recE);
	for (int j = 0; j < 3; ++j) outE[3 * counter + j] = recE[j];
	++counter;
}

OutputMatrixTxt(outQ, 4, counter, "quatTest");
OutputMatrixTxt(outM, 16, counter, "matTest");
OutputMatrixTxt(outE, 3, counter, "eulerTest");
*/
