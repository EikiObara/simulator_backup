// 2017/04/28
// created by eiki obara
#include <cmath>

#include "include\kinematics.h"
#include "include\kine_debag.h"
#include "include\kine_vector.h"
#include "include\kine_target_point.h"
#include "include\kine_config.h"

TarPoints::TarPoints() {
	top = { 0,0,0 };
	mid = { 0,0,0 };
	btm = { 0,0,0 };
}

TarPoints::~TarPoints() {}

/*
void TarPoints::CoordinateSwap(std::vector<double> tar1) {
	std::vector<double> buf(3,0);
}
*/

void TarPoints::display() {
	DebagComment("target top coordinate");
	DisplayVector(top);
	DebagComment("target middle coordinate");
	DisplayVector(mid);
	DebagComment("target bottom coordinate");
	DisplayVector(btm);
}

double calcMiddlePoint(double top, double bottom) {
	double buf = 0;

	buf = fabs(top - bottom) / 2;

	return bottom + buf;
}

std::vector<double> TarPoints::graspDirection() {
	//DebagComment("GraspDirection");
	//DisplayVector(top);
	//DisplayVector(mid);
	//DisplayVector(btm);

	std::vector<double> top2midV;
	std::vector<double> top2btmV;
	std::vector<double> crossV;
	std::vector<double> returnV(3, 0);

	top2midV = SubVector(mid, top);
	top2btmV = SubVector(btm, top);

	VectorNormalize(top2midV);
	VectorNormalize(top2btmV);

	//DebagComment("top2midV");	DisplayVector(top2midV);
	//DebagComment("top2btmV");	DisplayVector(top2btmV);

	//ベクトル方向が一緒か確認
	int sameVectorFlag = 0;
	
	for (int i = 0; i < 3; ++i) {
		if (top2midV[i] == top2btmV[i]) {
			++sameVectorFlag;
		}
	}

	if (sameVectorFlag == 3) {
		DebagComment(" grasp direction : same vector ");
		returnV[0] = 1.0;
		returnV[1] = 0.0;
		returnV[2] = 0.0;

		//for (int i = 0; i < 3; ++i) tempMidV[i] = calcMiddlePoint(top[i], btm[i]);
		//for (int i = 0; i < 3; ++i) returnV[i] = tempMidV[i] - mid[i];

		VectorNormalize(returnV);

		DebagComment("graspVector");
		DisplayVector(returnV);

		return returnV;
	}


	crossV = CrossVector(top2midV, top2btmV);
	VectorNormalize(crossV);
	//DebagComment("crossVec");	DisplayVector(crossV);

	returnV = CrossVector(crossV, top2btmV);
	VectorNormalize(returnV);

	int errorFlag = 0;
	for (int i = 0; i < 3; ++i) {
		if (std::isnan(returnV[i])) {
			++errorFlag;
		}
		if (std::isinf(returnV[i])) {
			++errorFlag;
		}
	}

	if (errorFlag > 0) {
		DebagComment("grasp direction : cannot calculate");
		double tempMidV[3] = {};
		
		for (int i = 0; i < 3; ++i) tempMidV[i] = calcMiddlePoint(top[i], btm[i]);
		for (int i = 0; i < 3; ++i) returnV[i] = tempMidV[i] - mid[i];

		VectorNormalize(returnV);

		return returnV;
	}

	//もし腕が180°ひっくり返るような把持方向なら直す
	if (returnV[0] < 0) {
		for (int i = 0; i < 3; ++i) {
			returnV[i] = -1 * returnV[i];
		}
	}

	//xの方向を向く

	//もし，crossベクトルのZ軸が負の場合は，returnVecの値にマイナスを掛けて逆方向にする
	//if (crossV[2] > 0) for (int i = 0; i < graspV.size(); ++i)graspV[i] = -1 * graspV[i];

	//DebagComment("grasp Vector");	DisplayVector(graspV);
	return	returnV;
}

void TarPoints::assignMid(double *points3) {
	for (int i = 0; i < 3; ++i) {
		mid[i] = points3[i];
	}
}
