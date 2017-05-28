// 2017/04/28
// created by eiki obara

#pragma once

#ifndef __HAND_POSTURE_H__
#define __HAND_POSTURE_H__

#include <vector>

class TarPoints{
public:
	std::vector<double> top;	//�R���N��
	std::vector<double> mid;	//�ʕ����_
	std::vector<double> btm;	//�t������
	
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
