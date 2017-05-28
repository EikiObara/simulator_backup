// 2017/04/28
// created by eiki obara

#pragma once

#ifndef __HAND_POSTURE_H__
#define __HAND_POSTURE_H__

#include <vector>

class TarPoints{
public:
	std::vector<double> top;	//コルク部
	std::vector<double> mid;	//果柄中点
	std::vector<double> btm;	//付け根部
	
	TarPoints();
	~TarPoints();
	//void CoordinateSwap(std::vector<double> tar1);
	void display();

	std::vector<double> graspDirection();

	void pointAssignMid(double *vec3);
	void pointAssignTop(double *vec3);
	void pointAssignBtm(double *vec3);
};

#endif // !__HAND_POSTURE_H__
