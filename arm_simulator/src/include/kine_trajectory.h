#pragma once
#include "kine_config.h"

#include "kine_target_point.h"

//直線軌道
void CalcVelocityLinear(double *firstPos, double *endPos, double currentTime, double *speed);

//スプライン曲線
void CalcVelocitySpline(double *firstPos, double *viaPos, double *endPos, double currentTime, double *moveSpeed);

void CalcVelocitySplineWithB(double *firstPos3, double *viaPos3, double *endPos3, double currentTime, double *moveSpeed);

//手先のクォータニオン姿勢操作
void CalcVelocityPosture(double *curJointRad, TarPoints *targetCoord, double currentTime, double *postureSpeed);

//経由点を生成するプログラム＊＊＊まだ危ない＊＊＊
void CalcViaPos(TarPoints targetPoints, double via2endLength, double *returnPos3);

void SelectTarget(kine::targetType type, TarPoints *tarPos);
