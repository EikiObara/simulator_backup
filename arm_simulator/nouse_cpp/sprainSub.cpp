#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "include\kinemaDebag.h"
#include "include\sprainSub.h"
#include "include\lu.h"

//calculation difference
double calcDif(const int n, double *p) {
	return p[n + 1] - p[n];
}

double calcV(const int n, double *x, double *y) {
	double x_n = calcDif(n, x);
	double x_n_1 = calcDif(n - 1, x);
	double y_n = calcDif(n, y);
	double y_n_1 = calcDif(n - 1, y);

	return 6 * ((y_n / x_n) - (y_n_1 / x_n_1));
}

// n : date number
// x : coordinate x matrix
// y : coordinate y matrix
// u : sprain function's coefficient
int CalcU(const int numMat,double *x, double *y, double *u) {
	Decom("calculation U started");

	DisplayVector(numMat, x);
	DisplayVector(numMat, y);
	DisplayVector(numMat, u);

	//Decom("ensure memory started");
	//relation calcU

	double *bufH = Dvector(numMat, numMat);
	double *invH = Dvector(numMat, numMat);
	double *v = Dvector(1, numMat);

	//relation LU separation
	double d = 0;
	int check = 0;
	int *index = Ivector(1, numMat);
	double *col = Dvector(numMat, numMat);

	//Decom("buffer H matrix initializing");

	for (int i = 0; i < numMat; ++i) {
		for (int j = 0; j < numMat; ++j) {
			bufH[numMat * i + j] = 0;
		}
	}

	DisplayRegularMatrix(numMat, bufH);

	////////////////////////////////////
	//Decom("sprain main started");
	//main//////////////////////////////

	bufH[0] = 2 * (calcDif(0, x) + calcDif(1, x));
	bufH[1] = calcDif(1, x);

	for (int i = 1; i < numMat - 2; ++i) {
		//printf("row number-> %d\n", i);
		bufH[(numMat - 1) * i + i - 1] = calcDif(i, x);
		bufH[(numMat - 1) * i + i] = 2 * (calcDif(i, x) + calcDif(i + 1, x));
		bufH[(numMat - 1) * i + i + 1] = calcDif(i + 1, x);
	}
	
	bufH[(numMat - 1)*(numMat - 1) - 2] = calcDif(numMat - 2, x);
	bufH[(numMat-1)*(numMat-1) - 1] = 2 * (calcDif(numMat - 2, x) + calcDif(numMat - 1, x));

	DisplayRegularMatrix(numMat,bufH);

	//////////////////////////////////////////
	//Decom("calculation V started");
	//////////////////////////////////////////

	/*
	for (int i = 0; i < numMat-1; ++i) {
		v[i] = calcV(i+1, x, y);
		printf("%lf\n", v[i]);
	}
	*/

	//LU function/////////////////////
	//Decom("LU function started");
	//////////////////////////////////

	check = Ludcmp(bufH, numMat-1, index, &d);

	if (check > 0) return 1;

	DisplayRegularMatrix(numMat, bufH);

	if (check > 0) {
		Ercom("---fatal error---\n---LU algolism---");
		exit(0);
	}

	//Decom("lubksb started");

	for (int i = 0; i < numMat-1; ++i) {
		for (int j = 0; j < numMat-1; ++j) { col[j] = 0.0; }
		col[i] = 1.0;

		Lubksb(bufH, numMat-1, index, col);

		for (int j = 0; j < numMat-1; ++j) {
			invH[(numMat-1) * i + j] = col[j];
			//printf("col[%d,%d] -> %lf \n", i, j, col[j]);
		}
	}
	
	Decom("display matrix current V vector");

	DisplayVector(numMat, v);

	Decom("display Matrix current inverse H Matrix");

	DisplayRegularMatrix(numMat, invH);

	Decom("calculation U matrix");

	for (int i = 0; i < numMat-1; ++i) {
		for (int j = 0; j < numMat - 1; ++j) {
			//printf("invH(%d,%d) = %lf\n", i, j, invH[((numMat - 1) * i) + j]);
			//printf("V(%d9 = %lf\n", i, v[j]);
			//printf("(%d,%d) = %lf\n", i, j, invH[((numMat - 1) * i) + j] * v[j]);
			u[i] += invH[((numMat - 1) * i) + j] * v[j];
		}
	}

	DisplayVector(numMat, u);

	//Decom("free memory");

	//free(bufH);
	//free(invH);
	//free(v);
	//free(index);
	//free(col);

	//Decom("end calc U function");

	//printf("check -> %d\n", check);

	return 0;
}

double CalcPlaneSprain(int numMat, double *x, double *y, double *u, double tarX) {
	int lo = 0;
	int hi = numMat;
	int k;
	double *uu = (double *)malloc((numMat + 2)  * sizeof(double));

	uu[0] = 0;
	uu[numMat + 1] = 0;
	for (int i = 0; i < numMat; ++i) {
		uu[i+1] = u[i];
	}

	while (hi - lo > 1) {
		k = (hi + lo) / 2;
		if (tarX < x[k]) {
			hi = k;
		}
		else {
			lo = k;
		}
	}

	double constNum[4];

	constNum[0] = (u[hi] - u[lo]) / (6 * (x[hi] - x[lo]));
	constNum[1] = u[lo] / 2.0;
	constNum[2] = (y[hi] - y[lo]) / (x[hi] - x[lo]) - (x[hi] - x[lo])*(2 * u[lo] + u[hi]) / 6.0;
	constNum[3] = y[lo];

	double yy = 0;
	double h = tarX - x[lo];

	for (int i = 0; i < 4; ++i) {
		yy += constNum[i];
		if (i < 3) {
			yy = yy * h;
		}
	}

	return yy;
}

double PlaneSprain(int numMat, double *x, double *y, double tarX) {
	double *u = (double *)malloc(sizeof(double) * numMat);
	int check = 0;
	double yy = 0;

	//DisplayVector(numMat, x);
	//DisplayVector(numMat, y);

	if (!u) {
		Ercom("fail to prepare memory");
		exit(0);
	}

	for (int i = 0; i < numMat; ++i) u[i] = 0.0;

	check += CalcU(numMat, x, y, u);

	if (check > 0) {
		Ercom(" ******* LU Disassembly FAILED ******* \n ***** PROGRAM FORCED TEMINATION ***** ");
		exit(1);
	}

	yy = CalcPlaneSprain(numMat, x, y, u, tarX);
	
	free(u);

	return yy;
}