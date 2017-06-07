#pragma once
#include "kine_config.h"

#include "kine_target_point.h"

//�����O��
void CalcVelocityLinear(double *firstPos, double *endPos, double currentTime, double *speed);

//�X�v���C���Ȑ�
void CalcVelocitySpline(double *firstPos, double *viaPos, double *endPos, double currentTime, double *moveSpeed);

void CalcVelocitySplineWithB(double *firstPos3, double *viaPos3, double *endPos3, double currentTime, double *moveSpeed);

//���̃N�H�[�^�j�I���p������
void CalcVelocityPosture(double *curJointRad, TarPoints *targetCoord, double currentTime, double *postureSpeed);

//�o�R�_�𐶐�����v���O�����������܂���Ȃ�������
void CalcViaPos(TarPoints targetPoints, double via2endLength, double *returnPos3);

void SelectTarget(kine::targetType type, TarPoints *tarPos);
