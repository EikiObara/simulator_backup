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

//ベクトル回転行列変換
//渡すベクトルは単位ベクトル(ノルムが一)であるように
void DirectVector2RotMat(std::vector<double> directionX, std::vector<double> directionY, double *matrix);


#endif // !__CONVERTOR_H__
