// 2017/04/28
// created by eiki obara

/////////////////////////////////////////////////////////
//���ꂪ���C���v���O�����D
//���main()���ɂ���֐���������Ăяo���Ďg�����ƁD
/////////////////////////////////////////////////////////

#define _USE_MATH_DEFINES

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include "include\arm_simulator.h"
#include "include\kine_debag.h"
#include "include\cone_simulator.h"
#include "include\Test_console.h"

void main() {

	kinemaDebagON();

	int check = 0;

	for (;;) {
		int function = -1;

		char ss[10] = {};

		fflush(stdin);

		std::cout << "*** �g�p����@�\��I��ł� ***" << std::endl;
		std::cout << "	1	: 7���R�x���{�b�g�A�[���V�~�����[�^ (�y�ʔ�)" << std::endl;
		std::cout << "	2	: 7���R�x���{�b�g�A�[���V�~�����[�^ (�t���O���t�B�N�X)" << std::endl;
		std::cout << "	3	: �~���V�~�����[�^" << std::endl;
		std::cout << "	4	: �R���\�[��" << std::endl;
		std::cout << "	99	: �v���O�����I��" << std::endl;
		std::cout << "\n-> ";
		
		fgets(ss, sizeof(ss), stdin);

		function = atoi(ss);

		switch (function) {
		case 1:
			std::cout << "*** Start : 7 DoF Arm Simulator (Light Version) ***\n";
			ArmSimulator(0);
			break;

		case 2:
			std::cout << "*** Start : 7 DoF Arm Simulator (Full Graphics) ***\n";
			ArmSimulator(1);
			break;

		case 3:
			std::cout << "*** Start : Cone Simulator ***\n";
			ConeSimulator();
			break;
		case 4:
			std::cout << "*** Start : console ***\n";
			TestConsole();
			break;

		case 99:
			std::cout << "*** program Shutdown ***\n";
			exit(1);
			break;

		default:
			DebagComment("*** Error : illegal Input ***");
			break;
		}
	}
}
