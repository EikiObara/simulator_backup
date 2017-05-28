// 2017/04/28
// created by eiki obara

#define _USE_MATH_DEFINES

#include <math.h>
#include "include\kinematics.h"
#include "include\camera_position.h"
#include "include\kine_Debag.h"
#include "include\kine_config.h"



//�ߋ����J�����̃I�t�Z�b�g�B
//originMat�ɃJ����������ۂɓ���3�������W��n����
//�I�t�Z�b�g����convertMat�ɕԂ��B(mm)
void ShortCameraOffset(const double *originMat, double *convertMat) {
	convertMat[0] = originMat[2] - 100;
	convertMat[1] = -originMat[1] - 8;
	convertMat[2] = -originMat[0] - 50;
}

//�������J�����̃I�t�Z�b�g�B
//originMat�ɃJ����������ۂɓ���3�������W��n����
//�I�t�Z�b�g����convertMat�ɕԂ��B
void LongCameraOffset(const double *originMat, double *convertMat) {
	convertMat[0] = originMat[2] + 143.0;
	convertMat[1] = originMat[1] + 208.0;
	convertMat[2] = originMat[0] - 314.0;
}

void CameraPositionInit(double *inRad, Matrix &OM) {
	//DebagCom("Camera Init started");

	int row, jnt;
	double theta[kine::MAXJOINT];
	double alpha[kine::MAXJOINT];
	double alength[kine::MAXJOINT];
	double dlength[kine::MAXJOINT];
	
	/*DebagBar();�@for (row = 0; row < MAXJOINT; row++) {	cout << inRad[row] << endl;	}*/

	//a(i-1) �����N�Ԃ̋���
	for (row = 0; row < kine::MAXJOINT; row++) { alength[row] = 0; }

	alength[kine::MAXJOINT-2] = kine::CAMERA_OFFSET;

	//alpha(i-1) �֐߂̂˂���̈ʒu
	alpha[0] = 0;		alpha[1] = -M_PI / 2;
	alpha[2] = M_PI / 2;	alpha[3] = -M_PI / 2;
	alpha[4] = M_PI / 2;	alpha[5] = -M_PI / 2;
	alpha[6] = M_PI / 2;	alpha[7] = M_PI / 2;

	//d(i) �����N����
	dlength[0] = 0;	dlength[1] = 0;
	dlength[2] = kine::U_ARM_LENGTH;	dlength[3] = 0;
	dlength[4] = kine::F_ARM_LENGTH;	dlength[5] = 0;
	dlength[6] = kine::WRIST2CAMERA_LENGTH;	dlength[7] = kine::CAMERA_POSITION;

	//theta(i) �֐ߊp�x
	for (row = 0; row < (kine::MAXJOINT - 1); row++) {
		theta[row] = inRad[row];
	}

	//���Ԗڂ̊֐߂͂Ȃ��̂�
	theta[7] = 0.0;

	//�f�o�b�O�p

	double cos_t, sin_t, cos_a, sin_a;

	for (jnt = 0; jnt < kine::MAXJOINT; jnt++) {
		cos_t = cos(theta[jnt]);
		sin_t = sin(theta[jnt]);
		cos_a = cos(alpha[jnt]);
		sin_a = sin(alpha[jnt]);
		
		cos_t = cos(theta[jnt]);
		sin_t = sin(theta[jnt]);
		cos_a = cos(alpha[jnt]);
		sin_a = sin(alpha[jnt]);

		OM.Mat3D(jnt, 0, 0, cos_t);
		OM.Mat3D(jnt, 0, 1, -sin_t);
		OM.Mat3D(jnt, 0, 2, 0.0);
		OM.Mat3D(jnt, 0, 3, alength[jnt]);

		OM.Mat3D(jnt, 1, 0, cos_a*sin_t);
		OM.Mat3D(jnt, 1, 1, cos_a*cos_t);
		OM.Mat3D(jnt, 1, 2, -sin_a);
		OM.Mat3D(jnt, 1, 3, -sin_a*dlength[jnt]);

		OM.Mat3D(jnt, 2, 0, sin_a*sin_t);
		OM.Mat3D(jnt, 2, 1, sin_a*cos_t);
		OM.Mat3D(jnt, 2, 2, cos_a);
		OM.Mat3D(jnt, 2, 3, cos_a*dlength[jnt]);

		OM.Mat3D(jnt, 3, 0, 0);
		OM.Mat3D(jnt, 3, 1, 0);
		OM.Mat3D(jnt, 3, 2, 0);
		OM.Mat3D(jnt, 3, 3, 1);
	}

	for (int jnt = 0; jnt < kine::MAXJOINT; ++jnt){
		for (int i = 0; i < OM.Row(); ++i) {
			for (int j = 0; j < OM.Column(); ++j) {
				if (fabs(OM.Mat3D(jnt, i, j) < kine::COMPARE_ZERO)) {	//cos()�ɂ�镂�������_�덷���O�ɂ���
					OM.Mat3D(jnt, i, j, 0.0);
				}
				if (OM.Mat3D(jnt, i, j) == -0.0) {	//-0��0�ɂ���
					OM.Mat3D(jnt, i, j, 0.0);
				}
				
			}
		}
	}


	//debag
	//DebagBar();
	//DisplayTriMatrix(OM, MAXJOINT);
}

void CalcHandCameraPosition(double *currentRadian, double *cameraCoord) {
	//DebagCom("FK kinematics position started");

	Matrix om;
	om.CreateTriMatrix(4, 4, kine::MAXJOINT);

	Matrix htm;
	htm.CreateTriMatrix(4, 4, kine::MAXJOINT);

	//(1)�����ϊ��s������
	CameraPositionInit(currentRadian, om);
	//DisplayTriMatrix(om, MAXJOINT);

	//(2)�����ϊ��s������
	kine::CalcHTM(om, htm);
	//DisplayTriMatrix(htm, MAXJOINT);

	//(3)�����ϊ��s��̎��ʒu�Ɋւ�����������o���D

	for (int i = 0; i < 3; ++i) {
		cameraCoord[i] = htm.Mat3D(kine::MAXJOINT - 1, i, 3);
	}

	//DisplayVector(3, cameraCoord);

	om.~Matrix();
	htm.~Matrix();
}