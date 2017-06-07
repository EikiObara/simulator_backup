// 2017/04/28
// created by eiki obara

#pragma once

//目標点を入れるクラスの実装

#ifndef __HAND_POSTURE_H__
#define __HAND_POSTURE_H__

#include <vector>

class TarPoints{
public:
	
	TarPoints();
	~TarPoints();
	//void CoordinateSwap(std::vector<double> tar1);
	void display();

	std::vector<double> top;	//コルク部
	std::vector<double> mid;	//果柄中点
	std::vector<double> btm;	//付け根部

	std::vector<double> graspDirection();
	void assignMid(double *points3);

	void getMidPoint(double *point3);
};

#endif // !__HAND_POSTURE_H__
