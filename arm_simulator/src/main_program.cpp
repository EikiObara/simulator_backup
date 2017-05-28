// 2017/04/28
// created by eiki obara

/////////////////////////////////////////////////////////
//これがメインプログラム．
//主にmain()内にある関数を一つだけ呼び出して使うこと．
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

		std::cout << "*** 使用する機能を選んでね ***" << std::endl;
		std::cout << "	1	: 7自由度ロボットアームシミュレータ (軽量版)" << std::endl;
		std::cout << "	2	: 7自由度ロボットアームシミュレータ (フルグラフィクス)" << std::endl;
		std::cout << "	3	: 円錐シミュレータ" << std::endl;
		std::cout << "	4	: コンソール" << std::endl;
		std::cout << "	99	: プログラム終了" << std::endl;
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
