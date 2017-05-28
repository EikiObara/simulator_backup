#pragma once
// 2017/04/28
// created by eiki obara

#ifndef __MY_CONFIG_H__
#define __MY_CONFIG_H__

namespace kine {
	//�ʉߓ_�̐ݒ�l(�����ʒu�ƍŏI�ʒu���܂ޓ_�̐�)
	const int ROUTE_POINTS = 3;

	//�ʉ߂��郋�[�g�̃����N��(�ʉߓ_���Ȃ����̂���)
	const int ROUTE_LINK = ROUTE_POINTS - 1;

	//���R�x�̐ݒ�
	const int MAXJOINT = 8; //7 axis and hand tip DoF

	//���x�v�Z1���[�v���Ƃɐi�ގ���
	const double TIME_SPAN = 0.01;

	//��`��Ԃ̌v�Z������
	const double TIME_LENGTH = 2.0;

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
	const double H_ARM_LENGTH = 0.135;	//real simulator

	//��񂩂�J�����ݒu�ʒu�̋���
	const double WRIST2CAMERA_LENGTH = 0.0405;	

	const double CAMERA_POSITION = 0.10;	//�J�����ݒu�ʒu����J�����܂ł̋���

	const double CAMERA_OFFSET = -0.006;	//���ƃJ�����̒��S�܂ł̃Y��



	//�X�y�[�X�}�E�X�̒l���x
	//const double SPACEMOUSE_TRANSLATION_GAIN = 0.0f;
	//const double SPACEMOUSE_ROTATION_GAIN = 0.0005f;

	//NULL���̃������m�ۗp
	const int NR_END = 1;

	//���l�̔�r�p�萔
	const double COMPARE_ZERO = 1.0e-6;
}
#endif // !__MY_CONFIG_H__
