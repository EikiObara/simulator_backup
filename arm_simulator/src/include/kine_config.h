#pragma once
// 2017/04/28
// created by eiki obara

//���������p����������j�b�g�̍��W���Ԉ���Ă���Ȃ�΁C
// src/kine_trajectory.cpp �� selectTarget �֐���ύX���Ă��������D


#ifndef __MY_CONFIG_H__
#define __MY_CONFIG_H__

namespace kine {
	enum targetType {
		CALIB_IN	= 0,	//�V�D�L�����u���[�V����������10cm��
		CALIB_OUT	= 1,	//�P�D�L�����u���[�V����������10�p
		CALIB_RIGHT = 2,
		INIT_POS	= 3,	//�Q�D�L�����u���[�V����������10cm���珉���p��
		PICK_POS	= 4,	//�R�D�����p������c���p��
		PICKING		= 5,	//�S�D�c���p������E�ݎ�蓮����s��
		CONVEY		= 6,	//�T�D�E�ݎ�蓮�����n�����@�\��
		KEYBOARD	= 7		//�U�D�L�[�{�[�h������
	};

	//�ʉߓ_�̐ݒ�l(�����ʒu�ƍŏI�ʒu���܂ޓ_�̐�)
	const int ROUTE_POINTS = 3;

	//�ʉ߂��郋�[�g�̃����N��(�ʉߓ_���Ȃ����̂���)
	const int ROUTE_LINK = ROUTE_POINTS - 1;

	//���R�x�̐ݒ�
	const int MAXJOINT = 8; //7 axis and hand tip DoF

	//���x�v�Z1���[�v���Ƃɐi�ގ���
	const double TIME_SPAN = 0.001;

	//���ړ�����
	const double TIME_LENGTH = 1.0;

	//�w��O���̌v�Z������
	const double POSITION_CHANGE_TIME = TIME_LENGTH;

	//���p���̌v�Z������
	const double POSTURE_CHANGE_TIME = TIME_LENGTH * 1;

	//��`��Ԃ̉���������
	const double ACCEL_TIME = TIME_LENGTH / 4;

	//���ʒu�����������l
	const double FINGER_THRESHOLD = 0.001;

	//�����N����
	//������(base)
	const double B_ARM_LENGTH = 0.164;
	//��r(upper)
	const double U_ARM_LENGTH = 0.322;
	//�O�r(forward)
	const double F_ARM_LENGTH = 0.257;
	//���(hand)
	//const double H_ARM_LENGTH = 0.157;	//light simulator
	const double H_ARM_LENGTH = 0.1407;	//real simulator

	//�o�R�_��ڕW�_�̔��a������ɐݒ肷�邩�D�P�ʂ̓��[�g��
	const double VIA_LENGTH = 0.08;

	//�X�y�[�X�}�E�X�̒l���x
	//const double SPACEMOUSE_TRANSLATION_GAIN = 0.0f;
	//const double SPACEMOUSE_ROTATION_GAIN = 0.0005f;

	//NULL���̃������m�ۗp
	const int NR_END = 1;

	//���l�̔�r�p�萔
	const double COMPARE_ZERO = 1.0e-6;
}
#endif // !__MY_CONFIG_H__