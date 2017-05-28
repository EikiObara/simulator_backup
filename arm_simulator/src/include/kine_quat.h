// 2017/04/28
// created by eiki obara

#pragma once

#ifndef __CALC_QUAT_H__
#define __CALC_QUAT_H__

#include "kine_vector.h"

#define ELEM(array) (sizeof(array)/sizeof *(array))
#define ROOT_2_INV 0.70710678118

/* クォータニオン構造体 */
enum Element {
	wq = 0,	xq = 1,	yq = 2,	zq = 3,
};

class Quat {
private:
	double w, x, y, z;
public:
	Quat();
	Quat(double wValue, double xValue, double yValue, double zValue);
	Quat(double wValue, std::vector<double> v);
	Quat(std::vector<double> vec4);
	Quat(double *vec4);

	~Quat();

	void display();

	//Quat quatFromTwoVector(const std::vector<double> v1, const std::vector<double> v2);
	//void identify();
	
	void Quat2array(double array4[4]);

	void assign();
	void assign(double wValue, double xValue, double yValue, double zValue);
	void assign(double wValue, std::vector<double> v);
	void assign(std::vector<double> vec4);
	void assign(double *vec4);
	void assign(Quat originQ);

	Quat add(Quat c);
	Quat sub(Quat c);
	Quat mul(Quat nulQ);
	Quat mulReal(const double s);
	Quat divReal(const double s);
	
	Quat conjugate();
	void normalize();
	//double normSqr();
	double norm();

	//Quat inverse();
	double dot(Quat dotQ);
	std::vector<double> cross(Quat crossQ);
	
	Quat slerp(double t, Quat tarQuat);

	void quat2RotM(double *rotMat);
	void quat2Euler(double *euler);
};

#endif //__CALC_QUAT_H__