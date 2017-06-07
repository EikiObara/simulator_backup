// 2017/05/06
// written by eiki obara
// reference http://www5d.biglobe.ne.jp/stssk/maze/spline.html

#include <math.h>
#include <stdio.h>
#include "include\kine_spline.h"
#include "include\kine_debag.h"

Spline::Spline() {
	num = 0;
}

Spline::~Spline() {
}

void Spline::initPoint(double *points, int pointValue) {
	double w[MAX_SPLINE_POINT];
	double temp;

	num = pointValue - 1;

	//a‚Ì’l‚Ì‰Šú‰»
	for (int i = 0; i <= num; ++i) {
		a[i] = points[i];
	}

	//‚ƒ‚Ì’l‚Ì‰Šú‰»
	c[0] = c[num] = 0.0;
	for (int i = 1; i < num; ++i) {
		c[i] = 3 * (a[i - 1] - 2 * a[i] + a[i + 1]);
	}

	w[0] = 0.0;
	for (int i = 1; i < num; ++i) {
		temp = 4.0 - w[i - 1];
		c[i] = (c[i] - c[i - 1]) / temp;
		w[i] = 1.0 / temp;
	}

	for (int i = num - 1; i > 0; --i) {
		c[i] = c[i] - c[i + 1] * w[i];
	}

	//b,d‚Ì‰Šú‰»
	b[num] = d[num] = 0.0;

	for (int i = 0; i < num; ++i) {
		d[i] = (c[i + 1] - c[i]) / 3.0;
		b[i] = a[i + 1] - a[i] - c[i] - d[i];
	}

	//for (int i = 0; i < num; ++i) {
	//	printf("a -> %lf\t", a[i]);
	//	printf("b -> %lf\t", b[i]);
	//	printf("c -> %lf\t", c[i]);
	//	printf("d -> %lf\n", d[i]);
	//}
}

double Spline::calc(double variable) {
	//DebagComment("spline::calc");
	//printf("variable -> %lf\n", variable);

	double dt = 0;	//minute time

	int i = (int)floor(variable);	//®”•”

	if (i < 0) {
		i = 0;
	}
	else if (i >= num) {
		i = num - 1;
	}

	dt = variable - (double)i;

	double buf = a[i] + (b[i] + (c[i] + d[i] * dt) * dt) * dt;

	//printf("buf -> %lf\n", buf);

	return buf;
}

int Combination(unsigned int n, unsigned int r) {
	//—v‘fn‚ªr‚æ‚è­‚È‚¢‚Æ‚«
	if (r * 2 > n) r = n - r;

	int dividend = 1;
	int divisor = 1;
	for (int i = 1; i <= r; ++i) {
		dividend *= (n - i + 1);
		divisor *= i;
	}
	return dividend / divisor;
}

double BSpline(std::vector<double> node, int nodeValue, double t) {
	double sum = 0;
	double c1 = 0;
	double c2 = 0;

	int NumOfTimes = nodeValue - 1;

	for (int r = 0; r <= NumOfTimes; ++r) {
		c1 = pow(1 - t, NumOfTimes - r);
		c2 = pow(t, r);
		sum += Combination(NumOfTimes, r) * c1 * c2 * node[r];
	}

	return sum;
}

double BSpline(double *node, int nodeValue, double t) {
	double sum = 0;
	double c1 = 0;
	double c2 = 0;

	int NumOfTimes = nodeValue - 1;

	for (int r = 0; r <= NumOfTimes; ++r) {
		c1 = pow(1 - t, NumOfTimes - r);
		c2 = pow(t, r);
		sum += Combination(NumOfTimes, r) * c1 * c2 * node[r];
	}

	return sum;
}