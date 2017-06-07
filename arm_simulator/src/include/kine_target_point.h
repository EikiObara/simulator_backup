// 2017/04/28
// created by eiki obara

#pragma once

//�ڕW�_������N���X�̎���

#ifndef __HAND_POSTURE_H__
#define __HAND_POSTURE_H__

#include <vector>

class TarPoints{
public:
	
	TarPoints();
	~TarPoints();
	//void CoordinateSwap(std::vector<double> tar1);
	void display();

	std::vector<double> top;	//�R���N��
	std::vector<double> mid;	//�ʕ����_
	std::vector<double> btm;	//�t������

	std::vector<double> graspDirection();
	void assignMid(double *points3);

	void getMidPoint(double *point3);
};

#endif // !__HAND_POSTURE_H__
