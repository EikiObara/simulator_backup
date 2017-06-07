// 2017/04/28
// created by eiki obara
#include "include\Test_console.h"

//この下にテストしたいプログラムのインクルードを書こう
#include "include\kine_debag.h"
//#include "include\kine_quat.h"
#include "include\trapezoidal_interpolation.h"
//#include "include\kine_config.h"
//#include "include\kine_trajectory.h"
#include "include\kine_spline.h"

#include <cstdio>
#include <cmath>
#include <ctime>
#include <cstdlib>

double output[10000] = {};

const int nodeValue = 3;

const double span = 0.01;

static void InitRand() {
	srand((unsigned int)time(NULL));
}

static int randamValue(int maxValue) {
	return rand() % maxValue + 1;
}

void TestConsole(void){

	InitRand();

	double x[nodeValue];
	int loopNum = 0;

	//for (int i = 0; i < nodeValue; ++i) x[i] = randamValue(10);
	
	x[0] = 1;
	x[1] = 5;
	x[2] = 5;


	DisplayVector(nodeValue, x);

	double nowT = 0;
	double beforeT = 0;

	double bufNow = 0;
	double bufBefore = 0;

	for (double t = 0; t <= 1; t += span) {
		beforeT = nowT;
		nowT += TrapeInterpolate(1, 1, t);

		bufNow = BSpline(x, nodeValue, nowT);
		bufBefore = BSpline(x, nodeValue, beforeT);

		output[loopNum] = bufNow - bufBefore;

		++loopNum;
	}
	OutputMatrixTxt(output, 1, loopNum, "BSpline_");

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
