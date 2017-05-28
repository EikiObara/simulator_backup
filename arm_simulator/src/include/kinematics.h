// 2017/04/28
// created by eiki obara

#pragma once

#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include "kine_matrix.h"
#include "kine_target_point.h"
#include "kine_quat.h"

namespace kine {

	//�֐ߊp�x���瓯���ϊ��s��𐶐�����
	void InitOM(double *currentRadian, Matrix &OM);

	//�����ϊ��s����֐߂��ƂɎZ�o����B
	void CalcHTM(Matrix OM, Matrix &HTM);

	//���R�r�s��𐶐�����B
	void CalcJacob(Matrix HTM, int selfmotion, Matrix &Jacob);

	//�^���t�s��𐶐�����B
	int PIM(Matrix mat, Matrix &PIMat);

	//�^���w�N���X
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
		//�I:elbow:e,���:wrist:w,���:finger:f

		//�悭�Ăяo���ϐ�
		Matrix om;
		Matrix htm;
		Matrix jacob;
		Matrix iJacob;

		//���W�l�������Ă���֐�
		double eX, eY, eZ;
		double wX, wY, wZ;
		double X, Y, Z;
	};
}
#endif// !__KINEMATICS_H__