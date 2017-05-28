#pragma once
#include "kine_target_point.h"

//�X�v���C���Ȑ�
void CalcVelocitySpline(double *firstPos, double *viaPos, double *endPos, double currentTime, double *moveSpeed);
//�����O��
void CalcVelocityLinear(double *firstPos, double *endPos, double currentTime, double *speed);

//���̃N�H�[�^�j�I���p������
void CalcVelocityPosture(double *curJointRad, TarPoints *targetCoord, double currentTime, double *postureSpeed);

void CalcViaPos(TarPoints targetPoints, double via2endLength, double *returnPos3);