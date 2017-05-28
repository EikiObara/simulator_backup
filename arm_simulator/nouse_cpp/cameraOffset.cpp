#include "include\cameraOffset.h"


//�ߋ����J�����̃I�t�Z�b�g�B
//originMat�ɃJ����������ۂɓ���3�������W��n����
//�I�t�Z�b�g����convertMat�ɕԂ��B
void ShortCameraOffset(const double *originMat, double *convertMat) {
	convertMat[0] = originMat[2] - 109;
	convertMat[1] = originMat[1] - 2;
	convertMat[2] = originMat[0] + 100;
}

//�������J�����̃I�t�Z�b�g�B
//originMat�ɃJ����������ۂɓ���3�������W��n����
//�I�t�Z�b�g����convertMat�ɕԂ��B
void LongCameraOffset(const double *originMat, double *convertMat) {
	convertMat[0] = originMat[2] + 143.0;
	convertMat[1] = originMat[1] + 208.0;
	convertMat[2] = originMat[0] - 314.0;
}