// 2017/04/28
// created by eiki obara

#pragma once

#ifndef __SPLINE_CLASS_H__
#define __SPLINE_CLASS_H__

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
#endif // !__SPLINE_CLASS_H__
