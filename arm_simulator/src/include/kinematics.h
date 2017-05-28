// 2017/04/28
// created by eiki obara

#pragma once

#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include "kine_matrix.h"
#include "kine_target_point.h"
#include "kine_quat.h"

namespace kine {

	//関節角度から同次変換行列を生成する
	void InitOM(double *currentRadian, Matrix &OM);

	//同次変換行列を関節ごとに算出する。
	void CalcHTM(Matrix OM, Matrix &HTM);

	//ヤコビ行列を生成する。
	void CalcJacob(Matrix HTM, int selfmotion, Matrix &Jacob);

	//疑似逆行列を生成する。
	int PIM(Matrix mat, Matrix &PIMat);

	//運動学クラス
	class Kinematics {
	public:
		//TarPoints *ch;
		Kinematics();
		~Kinematics();

		void DisplayCoordinate();

		void GetElbowCoordinate(double *ec);
		void GetwristCoordinate(double *wc);
		void GetCoordinate(double *fc);
		void GetHandHTM(double *currentRadian, double *htmMat);

		void CalcFK(double *currentRadian);
		int CalcIK(double *currentRadian, double *handVelocity, double *nextRadVerocity);
		int CalcPIK(double *currentRadian, double *handVelocity, double *nextRadVerocity);

		void Pick(double *curJointRad, TarPoints tarCoord, double *currentTime, double *nextRadVelocity);

	private:
		//肘:elbow:e,手首:wrist:w,手先:finger:f

		//よく呼び出す変数
		Matrix om;
		Matrix htm;
		Matrix jacob;
		Matrix iJacob;

		//座標値を持っている関数
		double eX, eY, eZ;
		double wX, wY, wZ;
		double X, Y, Z;
	};
}
#endif// !__KINEMATICS_H__