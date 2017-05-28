#pragma once
#include "kine_target_point.h"

//スプライン曲線
void CalcVelocitySpline(double *firstPos, double *viaPos, double *endPos, double currentTime, double *moveSpeed);
//直線軌道
void CalcVelocityLinear(double *firstPos, double *endPos, double currentTime, double *speed);

//手先のクォータニオン姿勢操作
void CalcVelocityPosture(double *curJointRad, TarPoints *targetCoord, double currentTime, double *postureSpeed);

void CalcViaPos(TarPoints targetPoints, double via2endLength, double *returnPos3);