// 2017/04/28
// created by eiki obara

#pragma once

#ifndef __SPLINE_CLASS_H__
#define __SPLINE_CLASS_H__

#include <vector>

const int MAX_SPLINE_POINT = 10;

class Spline {
public:
	Spline();
	~Spline();
	void initPoint(double *points, int pointValue);
	double calc(double t);
private:
	double a[MAX_SPLINE_POINT];
	double b[MAX_SPLINE_POINT];
	double c[MAX_SPLINE_POINT];
	double d[MAX_SPLINE_POINT];
	int num;

};

// 0 <= t <= 1, nodeValueŒÂ‚Ìnode‚ª‚ ‚é‚Æ‚« n = nodeValue - 1
//         n        n!
//f(t) =  ‡”   (---------) * (1-t)^(n-r) * t^r * node[r]
//       r = 0   r!(n-r)!
double BSpline(std::vector<double> node, int nodeValue, double t);

double BSpline(double *node, int nodeValue, double t);


#endif // !__SPLINE_CLASS_H__
