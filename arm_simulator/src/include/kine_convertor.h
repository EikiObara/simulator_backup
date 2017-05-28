// 2017/04/28
// created by eiki obara

#pragma once

#ifndef __CONVERTOR_H__
#define __CONVERTOR_H__

#include <vector>
#include "kine_quat.h"

void Euler2Angular(const double nowEuler[3], const double eulerVeclocity[3], double *angleVelocity);

//quatanion -> RotateMatrix

void RotMat2Quat(const double *rotMat, Quat &returnQuat);

//�x�N�g����]�s��ϊ�
//�n���x�N�g���͒P�ʃx�N�g��(�m��������)�ł���悤��
void DirectVector2RotMat(std::vector<double> directionX, std::vector<double> directionY, double *matrix);


#endif // !__CONVERTOR_H__
