// 2017/04/28
// created by eiki obara
//7���R�x�A�[���V�~�����[�^�̐S�����B
//���3�������f���̐����A����Ɋւ���R�[���o�b�N�֐�(callback)
//�L�[�{�[�h����󂯂��֐�(mykeypress)
//�^���w�v�Z(calcCore)��myKeyPress�̒��ŁC�l�X�ȏ������󂯂�

///////////////////////////////////////////////////////////////////
//����
//c:\coin3d\include\inventor\system\inttypes.h
//��
//#if !defined(HAVE_INT8_T) && defined(COIN_INT8_T)
//typedef COIN_INT8_T int8_t;
//#define HAVE_INT8_T 1
//#endif /* !HAVE_INT8_T && COIN_INT8_T */
//���`���Ă��܂���
//���̃v���O�����ŃG���[���o��\������D
/////////////////////////////////////////////////
//�֐��͐擪�������啶���C�ϐ��͏������C�P�ꂲ�Ƃɓ��������啶��
//�萔�͑S���啶��
///////////////////////////////////////////////////////////////////

#define _CRT_SECURE_NO_WARNINGS

#include "include\camera_position.h"
#include "include\kine_target_point.h"
#include "include\kine_debag.h"
#include "include\kinematics.h"
#include "include\kine_vector.h"
#include "include\3Dobjects.h"
#include "include\kine_config.h"
#include "include\kine_trajectory.h"

#include <stdio.h>
#include <stdlib.h>
#include <Inventor\Win\SoWin.h>
#include <Inventor\Win\viewers\SoWinExaminerViewer.h>
#include <Inventor\So.h>
//#include <Inventor\Win\devices\SoWinSpaceball.h>

////////////////////////////////////////////////////////////////
//coin3D�Ɖ^���w�ŋ��ʂ̕ϐ��D
//coin3D�̈������ς����Ȃ�����O���[�o���ɂ��邵���������̂������ɒu��

//�v�Z���ʂ̕`�������Ɉ��ɂ��邩
const double LOOP_SPAN = kine::TIME_SPAN * 1;

//�o�R�_��ڕW�_�̔��a������ɐݒ肷�邩�D�P�ʂ̓��[�g��
const double VIA_LENGTH = 0.1;

//�������ڂ̖ڕW�ʒu�D�P�ʂ�m(���[�g��)
static TarPoints cherryPos;

//�֐߂̊p�x���i�[����
static double currentJointRad[kine::MAXJOINT] = {};

//���J�����̈ʒu
static double handCameraCoordinate[3] = {};

//Recording array
double positionRec[3 * 100000] = {};
double velocityRec[7 * 100000] = {};

static int loopnum = 0;

///////////////////////////////////////////////////////////////////////////////
//��������֐��Q
//�ڕW���W�ݒ�
void pointInitialize() {
	//�����p�����̎��ʒu
	cherryPos.mid = { 0.601, -0.04, 0.0 };

	//�������̈ʒu
	//cherryPos.mid = { 0.193, -0.435, -0.250 };

	//�L�����u���[�V���������10cm������������
	//cherryPos.mid = { -0.116, -0.589, 0.0 };

	cherryPos.top = { cherryPos.mid[0] + 0.2,	cherryPos.mid[1] + 0.0,	cherryPos.mid[2] + 0.0 };
	cherryPos.btm = { cherryPos.mid[0] - 0.0,	cherryPos.mid[1] - 0.2,	cherryPos.mid[2] - 0.0 };
}

//�A�[���e�֐߂̏����l��^����֐��D�O���[�o���ϐ��ւ̃A�N�Z�X�Ȃ̂ŁD
//�v���O�����̂ǂ�����ł��N���ł��邩�璍��
void CurrentJointRadInit() {
	currentJointRad[0] = 60 * M_PI / 180 - M_PI / 2;//�����[��
	currentJointRad[1] = 0 * M_PI / 180 + M_PI / 2;	//���s�b�`
	currentJointRad[2] = 0 * M_PI / 180 + M_PI / 2;	//�����[
	currentJointRad[3] = 70 * M_PI / 180;			//�I
	currentJointRad[4] = 0 * M_PI / 180;			//��񃍁[��
	currentJointRad[5] = -40 * M_PI / 180;			//���s�b�`
	currentJointRad[6] = 0 * M_PI / 180;			//��惍�[��
	currentJointRad[7] = 0;							//�g��Ȃ��p�����[�^
}

void WriteOut() {
	printf("output velocity start\n");

	//OutputMatrixTxt(positionRec, 3, loopnum, "handPosi_");
	OutputMatrixTxt(velocityRec, 8, loopnum, "Velo_");

	printf("output velocity ended\n");
}

//�X�y�[�X�}�E�X����󂯎�鐔�l���i�[����
//float spaceMouseInput[6];

//�g�����̐���
void DisplayDescription() {
	DebagBar();
	printf("==============================================\n");
	printf("= �g�p���@\n");
	printf("= 1.�V�~�����[�V������ʂ��N���b�N�ŃA�N�e�B�u�ɂ���\n");
	printf("= 2.Esc�L�[�ŃL�[�{�[�h���샂�[�h�ɐ؂�ւ���\n");
	printf("= 3.�V�~�����[�V�����J�n�\�ɂȂ�܂�\n\n");

	printf("= available key -> A, G, H, J, K, L, I, P, R\n\n");
	printf("= A key -> move to Goal point\n\n");
	printf("= G, H key -> Selfmotion \n\n");
	printf("= J, L key -> Hand Yaw control \n\n");
	printf("= I, K key -> Hand Pitch control \n\n");
	printf("= P key -> Information current position i.e. coordinate, error... \n\n");
	printf("= R key -> Reset to initial position, and elapsed times \n\n");
	printf("= T key -> Reset Goal position, and elapsed times \n\n");
	printf("= W key -> write out parametors \n\n");
	printf("= Q key -> Quit Program \n");
	printf("==============================================\n");
}

//�p�����[�^�̕\��
void DisplayParametors(kine::Kinematics *arm){
	DebagBar();

	printf("currentJointRad(deg) 1-> %3.8lf\n", currentJointRad[0] * 180 / M_PI);
	printf("currentJointRad(deg) 2-> %3.8lf\n", currentJointRad[1] * 180 / M_PI);
	printf("currentJointRad(deg) 3-> %3.8lf\n", currentJointRad[2] * 180 / M_PI);
	printf("currentJointRad(deg) 4-> %3.8lf\n", currentJointRad[3] * 180 / M_PI);
	printf("currentJointRad(deg) 5-> %3.8lf\n", currentJointRad[4] * 180 / M_PI);
	printf("currentJointRad(deg) 6-> %3.8lf\n", currentJointRad[5] * 180 / M_PI);
	printf("currentJointRad(deg) 7-> %3.8lf\n", currentJointRad[6] * 180 / M_PI);

	DebagBar();

	//printf("currentJointRad(rad) 1-> %3.8lf\n",currentJointRad[0]);
	//printf("currentJointRad(rad) 2-> %3.8lf\n",currentJointRad[1]);
	//printf("currentJointRad(rad) 3-> %3.8lf\n",currentJointRad[2]);
	//printf("currentJointRad(rad) 4-> %3.8lf\n",currentJointRad[3]);
	//printf("currentJointRad(rad) 5-> %3.8lf\n",currentJointRad[4]);
	//printf("currentJointRad(rad) 6-> %3.8lf\n",currentJointRad[5]);
	//printf("currentJointRad(rad) 7-> %3.8lf\n",currentJointRad[6]);

	DebagBar();

	arm->DisplayCoordinate();

	DebagBar();

	printf("goal_x -> %3.8lf\n", cherryPos.mid[0]);
	printf("goal_y -> %3.8lf\n", cherryPos.mid[1]);
	printf("goal_z -> %3.8lf\n", cherryPos.mid[2]);

	double finger[3] = {};
	double wrist[3] = {};

	arm->GetCoordinate(finger);
	arm->GetwristCoordinate(wrist);

	printf("error_X -> %3.8lf\n", finger[0] - cherryPos.mid[0]);
	printf("error_Y -> %3.8lf\n", finger[1] - cherryPos.mid[1]);
	printf("error_Z -> %3.8lf\n", finger[2] - cherryPos.mid[2]);

	DebagBar();

	std::vector<double> f = { finger[0], finger[1], finger[2]};
	std::vector<double> w = { wrist[0], wrist[1],wrist[2] };
	std::vector<double> initPos = { 1,0,0 };
	std::vector<double> target(3, 0);

	for (int i = 0; i < 3; ++i) {
		target[i] = cherryPos.top[i] - cherryPos.btm[i];
	}

	std::vector<double> vec;

	vec = SubVector(f, w);

	VectorNormalize(vec);

	DebagComment("hand direction vector");
	DisplayVector(vec);

	DebagBar();

	double postureMatrix[16] = {};
	arm->GetHandHTM(currentJointRad, postureMatrix);

	DisplayRegularMatrix(4, postureMatrix);
}

//�t�^���w���v�Z
int CalcInverseKinematics(double *speed, kine::Kinematics *arm, bool selfMotion) {
	//�G���[�`�F�b�N
	int errorCheck = 0;

	//CalcIK����Ԃ��Ă���v�Z�l
	double nextRadVelocity[kine::MAXJOINT] = {};

	//�Z���t���[�V��������
	if (selfMotion == 1) errorCheck = arm->CalcIK(currentJointRad, speed, nextRadVelocity);
	
	//�[���t�s��@
	//else if (selfMotion == 0) errorCheck = arm->CalcPIK(currentJointRad, speed, nextRadVelocity);
	
	//�G���[�`�F�b�N
	if (errorCheck > 0) return 1;

	//�o�͂��ꂽ�֐ߊp���x�����݂̊֐ߊp�x�ɑ����D
	for (int i = 0; i < kine::MAXJOINT - 1; i++)	currentJointRad[i] += nextRadVelocity[i];
	
	return 0;
}

//�w��ړ��O�����v�Z
void PositionSpeed(double *currentTime, kine::Kinematics *arm, double *speed) {
	//���ړ����x�̎Z�o
	double firstPos[3] = {};
	double viaPos[3] = {};
	double endPos[3] = {};

	double positionBuf[3] = {};

	//���ʒu
	arm->GetCoordinate(firstPos);
	for (int i = 0; i < 3; ++i) endPos[i] = cherryPos.mid[i];

	//�o�R�_�̌v�Z
	CalcViaPos(cherryPos, VIA_LENGTH, viaPos);

	//�����O��
	//CalcVelocityLinear(firstPos, endPos, *currentTime, positionBuf);

	//�X�v���C��
	CalcVelocitySpline(firstPos, viaPos, endPos, *currentTime, positionBuf);

	//DebagComment("position buffer");	DisplayVector(3, positionBuf);

	for (int i = 0; i < 3; ++i) speed[i] = positionBuf[i];
}

//���p���v�Z
void PostureSpeed(double *currentTime, double *speed) {
	//���p���̑��x�Z�o/////////////////////////////////////////////////////////
	//speed�́@3 -> roll, 4 -> yaw , 5 -> pitch
	double postureBuf[3] = {};

	CalcVelocityPosture(currentJointRad, &cherryPos, *currentTime, postureBuf);
	//DebagComment("posture buffer");	DisplayVector(3, postureBuf);

	//�v�Z�����w��O���C���p�����쑬�x�̑��
	for (int i = 0; i < 3; ++i) speed[i + 3] = postureBuf[i];
	//speed[6] = 0;
}

//�^���w�v�Z�֐������s����֐��DLOOP_SPAN�̉񐔃��[�v������
void CalcCore(double *currentTime, kine::Kinematics *arm, bool selfMotion, double *speed) {
	//calcCore�̃��[�v�^�C�}�[
	double loopTimer = 0;

	//�G���[�`�F�b�N
	int errorCheck = 0;

	while (1) {
		//printf("currentTime -> %lf\n", *currentTime);

		//���ʒu�̎Z�o
		arm->CalcFK(currentJointRad);

		//���J�����̎Z�o
		//CalcHandCameraPosition(currentJointRad, handCameraCoordinate);

		//�O���v�Z
		PositionSpeed(currentTime, arm, speed);

		//���p���v�Z
		PostureSpeed(currentTime, speed);

		//���R�r�s���p�����t�^���w�̌v�Z
		errorCheck = CalcInverseKinematics(speed, arm, selfMotion);

		if (errorCheck == 1) {
			ErrComment("*** calculation error happened ***");
			//WriteOut();
			exit(1);
		}

		//�����o���p�z��
		velocityRec[loopnum * 8 + 0] = *currentTime;	//�f�o�b�O�p
		for (int i = 1; i < 8; ++i) velocityRec[8 * loopnum + i] = speed[i - 1];

		//�A�E�g�v�b�g�p�̏����o���񐔃J�E���^
		++loopnum;

		//���ݎ��Ԃ̍X�V
		*currentTime += kine::TIME_SPAN;

		//�V�~�����[�^��ŕ`�悷�鎞�ԁB
		//���̏������𔲂���ƃO���t�B�N�X��`�悷��B
		loopTimer += kine::TIME_SPAN;
		if (loopTimer >= LOOP_SPAN) break;
	}

	//DebagComment("calcCore : finished");
}

//�ڕW���W�̕ύX���s���֐�
void ChengeGoalValue(kine::Kinematics *arm){
	printf("Started --- Input Amount of movement ---\n");

	//���I��
	double axisInputBuf = 0;

	//
	int axisChoiseNum = -1;

	//
	char chooseAxis[2];

	while (1) {
		axisChoiseNum = -1;

		printf("enter to OVER WRITE Axis key (X or Y or Z)\n");
		printf("when finished, enter \"Q\" \n ->");
		fgets(chooseAxis, 2, stdin);

		if (chooseAxis[0] == 'x' || chooseAxis[0] == 'X') {
			printf("Input axis X \n");	axisChoiseNum = 0;
		}
		else if (chooseAxis[0] == 'y' || chooseAxis[0] == 'Y') {
			printf("Input axis Y \n");	axisChoiseNum = 1;
		}
		else if (chooseAxis[0] == 'z' || chooseAxis[0] == 'Z') {
			printf("Input axis Z \n");	axisChoiseNum = 2;
		}
		else if (chooseAxis[0] == 'i') {
			printf("Input 'I' (Reset the goal point to now place\n");	axisChoiseNum = 3;
		}
		else if (chooseAxis[0] == 'q') {
			printf("Exit enter Amount of movement\n\n\n");
			break;
		}
		else {
			printf("Unrecognizable. enter again\n\n");
			continue; 
		}

		if (axisChoiseNum > -1 && axisChoiseNum < 3) {
			printf("enter Value of movement [mm]\n-> ");

			scanf("%lf", &axisInputBuf);

			getchar();

			axisInputBuf *= 0.001;
			//cherryPos[i] += axisInputBuf;
			cherryPos.mid[axisChoiseNum] += axisInputBuf;

			printf("finished enter value\n\n\n");
		}
		else if (axisChoiseNum = 3) {
			double handPositionCoord[3] = {};

			getchar();

			arm->CalcFK(currentJointRad);

			arm->DisplayCoordinate();

			arm->GetCoordinate(handPositionCoord);

			cherryPos.pointAssignMid(handPositionCoord);

			printf("reset complite\n");
			axisChoiseNum = -1;
		}
	}
}

//���W�i���C���C���j�ɂ�葀�삷��L�[�{�[�h�R�[���o�b�N�֐�
void SimulatorKeyPressCB(void *userData, SoEventCallback *eventCB) {
	//clock_t start, end;
	//start = clock();

	//�I�C���C���ʒu���i�[����
	static kine::Kinematics arm;
	
	//��摬�x�i�[�z��
	double speed[7] = {};

	//�G���[�`�F�b�N�t���O
	int check = 0;

	//�o�ߎ��Ԋi�[�ϐ�
	static double CurrentTime = 0;

	//�v�Z�@�N���t���O	FALSE�Ȃ�off,TRUE�Ȃ�on
	bool calcSwitch = FALSE;
	//�Z���t���[�V�����t���O�@True �Ȃ� ON
	int selfMotionOn = TRUE;

	//�֐ߕϐ��̏������t���O
	static bool currentJointRadInitSwitch = TRUE;

	const SoEvent *event = eventCB->getEvent();

	//�֐ߕϐ��̏�����
	if (currentJointRadInitSwitch == TRUE) { 
		CurrentJointRadInit();
		currentJointRadInitSwitch = FALSE;
		pointInitialize();
	}

	//�X�y�[�X�}�E�X�̒l�擾
	/*
	if (event->isOfType(SoMotion3Event::getClassTypeId())) {
		SoMotion3Event* tr = (SoMotion3Event*)event;
		spaceMouseInput[0] = SPACEMOUSE_TRANSLATION_GAIN * tr->getTranslation().getValue()[0];	//x��
		spaceMouseInput[1] = SPACEMOUSE_TRANSLATION_GAIN * tr->getTranslation().getValue()[1];	//y��
		spaceMouseInput[2] = SPACEMOUSE_TRANSLATION_GAIN * tr->getTranslation().getValue()[2];	//z��
		spaceMouseInput[3] = SPACEMOUSE_ROTATION_GAIN * tr->getRotation().getValue()[0];		//��
		spaceMouseInput[4] = SPACEMOUSE_ROTATION_GAIN * tr->getRotation().getValue()[1];		//y������]
		spaceMouseInput[5] = SPACEMOUSE_ROTATION_GAIN * tr->getRotation().getValue()[2];

		//speed[3] = -spaceMouseInput[4];	//���[��
		speed[4] = -spaceMouseInput[5];		//���[
		speed[5] = spaceMouseInput[3];		//�s�b�`

		calcSwitch = 1;
	}
	*/

	if (SO_KEY_PRESS_EVENT(event, T)) {
		ChengeGoalValue(&arm);
		CurrentTime = 0;	//Reset VelocityRegulation
	}

	//�K�v�Ȍv�Z�����ׂĂ��s���L�[�@A (all)
	if (SO_KEY_PRESS_EVENT(event, A)) {
		calcSwitch = TRUE;
		selfMotionOn = 1;
		speed[6] = 0.0;
	}

	//���ʒu�������p���ɖ߂��L�[�@R (restart)
	if (SO_KEY_PRESS_EVENT(event, R)) {
		system("cls");
		currentJointRadInitSwitch = TRUE;
		CurrentTime = 0;
		DisplayDescription();
	}

	//�Z���t���[�V����
	if (SO_KEY_PRESS_EVENT(event, S)) {	
		calcSwitch = TRUE;	
		selfMotionOn = 1;
		speed[6] = M_PI * kine::TIME_SPAN;
	}
	if (SO_KEY_PRESS_EVENT(event, D)) {
		calcSwitch = TRUE;	
		selfMotionOn = 1;
		speed[6] = -M_PI * kine::TIME_SPAN;
	}

	//��摀��
	if (SO_KEY_PRESS_EVENT(event, J)) {
		calcSwitch = TRUE;	
		selfMotionOn = 1;
		speed[4] = -M_PI * kine::TIME_SPAN;
		speed[6] = speed[4] / 3;
	}//Yaw
	if (SO_KEY_PRESS_EVENT(event, L)) {
		calcSwitch = TRUE;
		selfMotionOn = 1;
		speed[4] = M_PI * kine::TIME_SPAN;
		speed[6] = speed[4] / 3;
	}//Yaw
	if (SO_KEY_PRESS_EVENT(event, K)) {
		calcSwitch = TRUE;
		selfMotionOn = 1;
		speed[5] = M_PI * kine::TIME_SPAN;
		speed[6] = 0;
	}//pitch
	if (SO_KEY_PRESS_EVENT(event, I)) {
		calcSwitch = TRUE;
		selfMotionOn = 1;
		speed[5] = -M_PI * kine::TIME_SPAN;
		speed[6] = 0;
	}//pitch

	//�p�����[�^�\��
	if (SO_KEY_PRESS_EVENT(event, P)) {
		arm.CalcFK(currentJointRad);
		DisplayParametors(&arm);
	}

	if (SO_KEY_PRESS_EVENT(event, Z)) {
		kinemaDebagON();
	}
	if (SO_KEY_PRESS_EVENT(event, X)) {
		kinemaDebagOFF();
	}

	//�v�Z�v���O�����J�n
	if (calcSwitch == TRUE) { 
		CalcCore(&CurrentTime, &arm, selfMotionOn, speed);
	}
	
	if (SO_KEY_PRESS_EVENT(event, W)) {
		WriteOut();		
	}

	//�v���O�����I��
	if (SO_KEY_PRESS_EVENT(event, Q)) { 
		//DebagCom("program finished");
		exit(1);
	}

	//SpeedIntegration(speed);

	if (check > 0) {
		ErrComment("\n\n\n*** fatal error ***\n*** please enter 'q' for finish this program ***\n");
		exit(1);
	}

	//end = clock();	printf("��������:%lf[ms]\n",(double)(end - start)/CLOCKS_PER_SEC);
	//DebagComment("call back finished\n");
}

/*SoWinSpaceball* SpaceMouseSet() {
	float f;
	if (SoWinSpaceball::exists()) {
		printf("�X�y�[�X�}�E�X�����o�ł��܂���\n");
		exit(1);
	}
	SoWinSpaceball* sb = new SoWinSpaceball;
	f = sb->getRotationScaleFactor();
	printf("Spacemouse::RotationScaleFactor\t= %f\n", f);
	f = sb->getTranslationScaleFactor();
	printf("Spacemouse::TranslationScaleFactor\t= %f\n", f);
	return sb;
}
*/




//�e�֐߂̉�]�R�[���o�b�N
static void Arm1RotSensorCallback(void *b_data1, SoSensor *) {
	SoTransform *rot1 = (SoTransform *)b_data1;
	rot1->center.setValue(0, 0, 0);
	rot1->rotation.setValue(SbVec3f(0, 0, 1), currentJointRad[0]);
}
static void Arm2RotSensorCallback(void *b_data2, SoSensor *) {
	SoTransform *rot2 = (SoTransform *)b_data2;
	rot2->center.setValue(0, 0, 0);
	rot2->rotation.setValue(SbVec3f(0, 0, 1), currentJointRad[1]);
}
static void Arm3RotSensorCallback(void *b_data3, SoSensor *) {
	SoTransform *rot3 = (SoTransform *)b_data3;
	rot3->center.setValue(0, 0, 0);
	rot3->rotation.setValue(SbVec3f(0, 0, 1), currentJointRad[2]);
}
static void Arm4RotSensorCallback(void *b_data4, SoSensor *) {
	SoTransform *rot4 = (SoTransform *)b_data4;
	rot4->center.setValue(0, 0, 0);
	rot4->rotation.setValue(SbVec3f(0, 0, 1), currentJointRad[3]);
}
static void Arm5RotSensorCallback(void *b_data5, SoSensor *) {
	SoTransform *rot5 = (SoTransform *)b_data5;
	rot5->center.setValue(0, 0, 0);
	rot5->rotation.setValue(SbVec3f(0, 0, 1), currentJointRad[4]);	//Z
}
static void Arm6RotSensorCallback(void *b_data6, SoSensor *) {
	SoTransform *rot6 = (SoTransform *)b_data6;
	rot6->center.setValue(0, 0, 0);
	rot6->rotation.setValue(SbVec3f(0, 0, 1), currentJointRad[5]);
}
static void Arm7RotSensorCallback(void *b_data7, SoSensor *) {
	SoTransform *rot7 = (SoTransform *)b_data7;
	rot7->center.setValue(0, 0, 0);
	rot7->rotation.setValue(SbVec3f(0, 0, 1), currentJointRad[6]);	//Y

	/*
	static int i = 0;
	++i;
	if (i == 10) printf("fps check start\n");
	if (i == 1010) {
		printf("1000 roops\n");
		i = 0;
	}
	*/
}
static void RedPointSensorCallback(void *b_data8, SoSensor *) {
	SoTranslation *trans8 = (SoTranslation*)b_data8;

	trans8->translation.setValue(handCameraCoordinate[0], handCameraCoordinate[1], handCameraCoordinate[2]);
}
static void GoalPointSensorCallback(void *b_data, SoSensor *) {
	SoTranslation *trans = (SoTranslation*)b_data;

	trans->translation.setValue(cherryPos.mid[0], cherryPos.mid[1], cherryPos.mid[2]);
}

void ArmSimulator(int argc){
	//�������̕\��
	DisplayDescription();

	pointInitialize();

	//��ʂ̏�����
	HWND myWindow = SoWin::init("");

	//�v���O�����c���[�̎n�܂�
	SoSeparator *root = new SoSeparator;
	SoSeparator *arms = new SoSeparator;
	
	//�e�I�u�W�F�N�g�c���[�̃Z�p���[�^�̍쐬
	SoSeparator *basesep = new SoSeparator;	SoSeparator *arm1sep = new SoSeparator;
	SoSeparator *arm2sep = new SoSeparator;	SoSeparator *arm3sep = new SoSeparator;
	SoSeparator *arm4sep = new SoSeparator;	SoSeparator *arm5sep = new SoSeparator;
	SoSeparator *arm6sep = new SoSeparator;	SoSeparator *arm7sep = new SoSeparator;

	//�X�y�[�X�}�E�X�̊m�F
	//if (SoWinSpaceball::exists()) { printf("spaceball No Exists\n");	exit(1); }
	//spacemouse callback
	//SoEventCallback *spaceMouseCB = new SoEventCallback;
	//spaceMouseCB->addEventCallback(SoMotion3Event::getClassTypeId(), MyKeyPressCB, root);

	//�R�[���o�b�N
	SoEventCallback *eventcallback = new SoEventCallback;
	eventcallback->addEventCallback(SoKeyboardEvent::getClassTypeId(), SimulatorKeyPressCB, root);

	//�ڕW�`��
	SoSeparator *cherryPosSep = new SoSeparator;
	GoalCherry(cherryPos.mid, cherryPosSep);

	//�J�����ʒu�m�F�p�ԋ�
	SoSeparator *redPointSep = new SoSeparator;

	SoTranslation *redPointTrans = new SoTranslation;
	SoTimerSensor *redPointTransMoveSensor = new SoTimerSensor(RedPointSensorCallback, redPointTrans);
	redPointTransMoveSensor->setInterval(0.01);
	redPointTransMoveSensor->schedule();
	
	redPointSep->addChild(redPointTrans);
	PointObj(redPointSep);

	//////////////////////////////////////////
	//�ڕW���W�O�_�m�F�p
	SoSeparator *goalPoints = new SoSeparator;

	SoSeparator *topSep = new SoSeparator;
	SoSeparator *midSep = new SoSeparator;
	SoSeparator *btmSep = new SoSeparator;

	PointObj(cherryPos.top, topSep);
	PointObj(cherryPos.mid, midSep);
	PointObj(cherryPos.btm, btmSep);

	SoTranslation *goalPointTrans = new SoTranslation;
	SoTimerSensor *goalPointTransMoveSensor = new SoTimerSensor(GoalPointSensorCallback, goalPointTrans);
	goalPointTransMoveSensor->setInterval(0.01);
	goalPointTransMoveSensor->schedule();


	goalPoints->addChild(topSep);
	goalPoints->addChild(midSep);
	goalPoints->addChild(btmSep);

	//�x���̃A���~�t���[��
	SoSeparator *almiFlame1 = new SoSeparator;
	SoSeparator *almiFlame2 = new SoSeparator;

	//std::vector<double> alm1Position = { 0.11, -0.916, -0.210 };
	std::vector<double> alm1Position = { 0.0, -0.916, -0.210 };
	std::vector<double> alm2Position = { -0.08, -0.916, -0.210 };

	AlmiFlame(alm1Position, almiFlame1);
	AlmiFlame(alm2Position, almiFlame2);

	//���W�n�\��
	SoSeparator *coordinateSystem = new SoSeparator;
	CoordinateSystem(coordinateSystem);

	//���W�ϊ��p�ϐ� arm(x)trans x�͑Ή�����֐ߔԍ�
	SoTranslation *baseTrans = new SoTranslation;	SoTranslation *arm1Trans = new SoTranslation;
	SoTranslation *arm2Trans = new SoTranslation;	SoTranslation *arm3Trans = new SoTranslation;
	SoTranslation *arm4Trans = new SoTranslation;	SoTranslation *arm5Trans = new SoTranslation;
	SoTranslation *arm6Trans = new SoTranslation;	SoTranslation *arm7Trans = new SoTranslation;

	//���i�ǂݍ���
	SoInput baseIpt;	SoInput arm1Ipt;	SoInput arm2Ipt;	SoInput arm3Ipt;
	SoInput arm4Ipt;	SoInput arm5Ipt;	SoInput arm6Ipt;	SoInput arm7Ipt;

	if (argc == 0) {
		//���ȈՔŃA�[��/////////////////////////////////////////////////
		if (!baseIpt.openFile("Arms1.0/base.wrl")) exit(1);
		if (!arm1Ipt.openFile("Arms1.0/arm1.wrl")) exit(1);
		if (!arm2Ipt.openFile("Arms1.0/arm2.wrl")) exit(1);
		if (!arm3Ipt.openFile("Arms1.0/arm3.wrl")) exit(1);
		if (!arm4Ipt.openFile("Arms1.0/arm4.wrl")) exit(1);
		if (!arm5Ipt.openFile("Arms1.0/arm5.wrl")) exit(1);
		if (!arm6Ipt.openFile("Arms1.0/arm6.wrl")) exit(1);
		if (!arm7Ipt.openFile("Arms1.0/arm7fix.wrl")) exit(1);
		baseTrans->translation.setValue(0.0, 0.0, -0.164);
		arm1Trans->translation.setValue(0.0, 0.0, 0.01);
		arm2Trans->translation.setValue(0.0, 0.0, 0.154);
		arm3Trans->translation.setValue(0.0, 0.0, 0.0);
		arm4Trans->translation.setValue(0.0, 0.0, 0.322);
		arm5Trans->translation.setValue(0.0, -0.22, 0.0);
		arm6Trans->translation.setValue(0.0, 0.0, 0.037);
		arm7Trans->translation.setValue(0.0, -0.022, 0.0);
	}
	else if (argc == 1) {
		//�J�����t���A�[����//////////////////////////////////////////////
		if (!baseIpt.openFile("Arms4.0/ArmBaseRefine.wrl")) exit(1);
		if (!arm1Ipt.openFile("Arms4.0/Arm1Refine.wrl")) exit(1);
		if (!arm2Ipt.openFile("Arms4.0/Arm2Refine.wrl")) exit(1);
		if (!arm3Ipt.openFile("Arms4.0/Arm3Refine.wrl")) exit(1);
		if (!arm4Ipt.openFile("Arms4.0/Arm4Refine.wrl")) exit(1);
		if (!arm5Ipt.openFile("Arms4.0/Arm5Refine.wrl")) exit(1);
		if (!arm6Ipt.openFile("Arms4.0/Arm6Refine.wrl")) exit(1);
		if (!arm7Ipt.openFile("Arms4.0/Arm7Finished.wrl")) exit(1);
		baseTrans->translation.setValue(0.0, 0.0, -0.164);
		arm1Trans->translation.setValue(0.0, 0.0, 0.006);
		arm2Trans->translation.setValue(0.0, 0.0, 0.158);
		arm3Trans->translation.setValue(0.0, 0.007, 0.0);
		arm4Trans->translation.setValue(0.0, 0.0, 0.329);
		arm5Trans->translation.setValue(0.0, -0.237, 0.0);
		arm6Trans->translation.setValue(0.0, 0.0, 0.020);
		arm7Trans->translation.setValue(0.0, -0.008, 0.0);
	}

	//�I�u�W�F�N�g���Z�p���[�^�Ɋi�[////////////////////////////////////////////////////////////////////////
	SoSeparator *baseObj = SoDB::readAll(&baseIpt);	SoSeparator *arm1Obj = SoDB::readAll(&arm1Ipt);
	SoSeparator *arm2Obj = SoDB::readAll(&arm2Ipt);	SoSeparator *arm3Obj = SoDB::readAll(&arm3Ipt);
	SoSeparator *arm4Obj = SoDB::readAll(&arm4Ipt);	SoSeparator *arm5Obj = SoDB::readAll(&arm5Ipt);
	SoSeparator *arm6Obj = SoDB::readAll(&arm6Ipt);	SoSeparator *arm7Obj = SoDB::readAll(&arm7Ipt);
	//�I�u�W�F�N�g���Z�p���[�^�Ɋi�[////////////////////////////////////////////////////////////////////////

	//.wrl�̓ǂݍ��݃G���[����////////////////////////////////////////////////
	if (baseObj == NULL) exit(1);	if (arm1Obj == NULL) exit(1);
	if (arm2Obj == NULL) exit(1);	if (arm3Obj == NULL) exit(1);
	if (arm4Obj == NULL) exit(1);	if (arm5Obj == NULL) exit(1);
	if (arm6Obj == NULL) exit(1);	if (arm7Obj == NULL) exit(1);
	//.wrl�̓ǂݍ��݃G���[����////////////////////////////////////////////////

	//�֐߉�]�R�[���o�b�N/////////////////////////////////////////////////////////////////////////////////////////
	SoTransform *arm1Rot = new SoTransform;	SoTransform *arm2Rot = new SoTransform;
	SoTransform *arm3Rot = new SoTransform;	SoTransform *arm4Rot = new SoTransform;	
	SoTransform *arm5Rot = new SoTransform;	SoTransform *arm6Rot = new SoTransform;
	SoTransform *arm7Rot = new SoTransform;

	SoTimerSensor *arm1MoveSensor = new SoTimerSensor(Arm1RotSensorCallback, arm1Rot);
	SoTimerSensor *arm2MoveSensor = new SoTimerSensor(Arm2RotSensorCallback, arm2Rot);
	SoTimerSensor *arm3MoveSensor = new SoTimerSensor(Arm3RotSensorCallback, arm3Rot);
	SoTimerSensor *arm4MoveSensor = new SoTimerSensor(Arm4RotSensorCallback, arm4Rot);
	SoTimerSensor *arm5MoveSensor = new SoTimerSensor(Arm5RotSensorCallback, arm5Rot);
	SoTimerSensor *arm6MoveSensor = new SoTimerSensor(Arm6RotSensorCallback, arm6Rot);
	SoTimerSensor *arm7MoveSensor = new SoTimerSensor(Arm7RotSensorCallback, arm7Rot);

	arm1MoveSensor->setInterval(0.01);	arm2MoveSensor->setInterval(0.01);
	arm3MoveSensor->setInterval(0.01);	arm4MoveSensor->setInterval(0.01);	
	arm5MoveSensor->setInterval(0.01);	arm6MoveSensor->setInterval(0.01);
	arm7MoveSensor->setInterval(0.01);	

	arm1MoveSensor->schedule();	arm2MoveSensor->schedule();
	arm3MoveSensor->schedule();	arm4MoveSensor->schedule();
	arm5MoveSensor->schedule();	arm6MoveSensor->schedule();
	arm7MoveSensor->schedule();
	//�֐߉�]�R�[���o�b�N/////////////////////////////////////////////////////////////////////////////////////////

	
	//���i���Ƃ̏����ݒ�///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	SoTransform *arm1InitialRot = new SoTransform;	arm1InitialRot->rotation.setValue(SbVec3f(1, 0, 0), 0);
	SoTransform *arm2InitialRot = new SoTransform;	arm2InitialRot->rotation.setValue(SbVec3f(1, 0, 0), -M_PI / 2);
	SoTransform *arm3InitialRot = new SoTransform;	arm3InitialRot->rotation.setValue(SbVec3f(1, 0, 0), M_PI / 2);
	SoTransform *arm4InitialRot = new SoTransform;	arm4InitialRot->rotation.setValue(SbVec3f(1, 0, 0), -M_PI / 2);
	SoTransform *arm5InitialRot = new SoTransform;	arm5InitialRot->rotation.setValue(SbVec3f(1, 0, 0), M_PI / 2);
	SoTransform *arm6InitialRot = new SoTransform;	arm6InitialRot->rotation.setValue(SbVec3f(1, 0, 0), -M_PI / 2);
	SoTransform *arm7InitialRot = new SoTransform;	arm7InitialRot->rotation.setValue(SbVec3f(1, 0, 0), M_PI / 2);
	//���i���Ƃ̏����ݒ�///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//�Z�p���[�^�c���[�쐬///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	basesep->addChild(baseTrans);	basesep->addChild(baseObj);
	//basesep->addChild(coordinateSystem);	
	arm1sep->addChild(arm1Trans);	arm1sep->addChild(arm1InitialRot);	arm1sep->addChild(arm1Rot);	arm1sep->addChild(arm1Obj);
	//arm1sep->addChild(coordinateSystem);	
	arm2sep->addChild(arm2Trans);	arm2sep->addChild(arm2InitialRot);	arm2sep->addChild(arm2Rot);	arm2sep->addChild(arm2Obj);
	//arm2sep->addChild(coordinateSystem);	
	arm3sep->addChild(arm3Trans);	arm3sep->addChild(arm3InitialRot);	arm3sep->addChild(arm3Rot);	arm3sep->addChild(arm3Obj);
	//arm3sep->addChild(coordinateSystem);	
	arm4sep->addChild(arm4Trans);	arm4sep->addChild(arm4InitialRot);	arm4sep->addChild(arm4Rot);	arm4sep->addChild(arm4Obj);
	//arm4sep->addChild(coordinateSystem);	
	arm5sep->addChild(arm5Trans);	arm5sep->addChild(arm5InitialRot);	arm5sep->addChild(arm5Rot);	arm5sep->addChild(arm5Obj);
	//arm5sep->addChild(coordinateSystem);	
	arm6sep->addChild(arm6Trans);	arm6sep->addChild(arm6InitialRot);	arm6sep->addChild(arm6Rot);	arm6sep->addChild(arm6Obj);
	//arm6sep->addChild(coordinateSystem);	
	arm7sep->addChild(arm7Trans);	arm7sep->addChild(arm7InitialRot);	arm7sep->addChild(arm7Rot);	arm7sep->addChild(arm7Obj);
	arm7sep->addChild(coordinateSystem);

	//�Z�p���[�^�c���[�̍쐬///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//���i�g�ݗ��ĕ���///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	root->ref();

	SoSeparator *rootSep = new SoSeparator;
	root->addChild(rootSep);

	rootSep->addChild(coordinateSystem);	//world coordinate system
	//rootSep->addChild(cheerySep);
	rootSep->addChild(almiFlame1);
	rootSep->addChild(almiFlame2);
	rootSep->addChild(goalPoints);
	//root->addChild(redPointSep);

	//root->addChild(spaceMouseCB);
	root->addChild(eventcallback);
	root->addChild(arms);
	arms->addChild(basesep);
	basesep->addChild(arm1sep);
	arm1sep->addChild(arm2sep);
	arm2sep->addChild(arm3sep);
	arm3sep->addChild(arm4sep);
	arm4sep->addChild(arm5sep);
	arm5sep->addChild(arm6sep);
	arm6sep->addChild(arm7sep);
	//���i�g�ݗ��ĕ���///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//�r�����[�̐ݒ�///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	SoWinExaminerViewer *myViewer = new SoWinExaminerViewer(myWindow);
	//myViewer->registerDevice(SpaceMouseSet());
	myViewer->setSceneGraph(root);
	myViewer->setTitle("simulator 7dof arm");
	myViewer->setBackgroundColor(SbColor(0.3, 0.3, 0.4));
	myViewer->setSize(SbVec2s(720, 600));
	myViewer->show();

	//�r�����[�̐ݒ�///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	SoWin::show(myWindow);
	SoWin::mainLoop();
}

//*******************��Ώ�����!!!!!**************************
/*
//���W�_�m�F�p�T�N�����{�`��
void GoalCherry(SoSeparator *GoalSep) {
	///////////////////////////////////////

	SoInput GoalIpt;
	if (!GoalIpt.openFile("cherryfruit.wrl")) exit(1);
	SoSeparator *GoalObj1 = SoDB::readAll(&GoalIpt);
	if (GoalObj1 == NULL) exit(1);

	//color
	SoMaterial *GoalColor = new SoMaterial;
	GoalColor->diffuseColor.setValue(1, 0, 0);

	//�ʒu���킹
	SoTranslation *GoalTranslation1 = new SoTranslation;
	//GoalTranslation1->translation.setValue(HANDGOAL[0], HANDGOAL[1] - 0.03, HANDGOAL[2]);
	GoalTranslation1->translation.setValue(cherry.mid[0], cherry.mid[1] - 0.03, cherry.mid[2]);

	SoTransform *GoalInitialrot = new SoTransform;	//Y

	GoalSep->addChild(GoalColor);
	GoalSep->addChild(GoalTranslation1);
	//GoalSep->addChild(GoalTransform);
	GoalSep->addChild(GoalObj1);
}

void PointObj(std::vector<double> position, SoSeparator *redPoint) {
	SoSphere *redP = new SoSphere;

	redP->radius = 0.01;

	SoMaterial *myMaterial = new SoMaterial;
	myMaterial->diffuseColor.setValue((rand() % 101) * 0.01, (rand() % 101) * 0.01, (rand() % 101) * 0.01);

	SoTranslation *trans = new SoTranslation;

	trans->translation.setValue(position[0], position[1], position[2]);

	redPoint->addChild(myMaterial);
	redPoint->addChild(trans);
	redPoint->addChild(redP);
}

void PointObj(SoSeparator *redPoint) {
	SoSphere *redP = new SoSphere;

	redP->radius = 0.01;

	SoMaterial *myMaterial = new SoMaterial;
	myMaterial->diffuseColor.setValue((rand() % 101) * 0.01, (rand() % 101) * 0.01, (rand() % 101) * 0.01);

	redPoint->addChild(myMaterial);
	redPoint->addChild(redP);
}

//�A���~�t���[���`��
void AlmiFlame1(SoSeparator *armFlame) {
	///////////////////////////////////////

	SoInput armFlameIpt;
	if (!armFlameIpt.openFile("hfsh8-8080-1000_vertical.wrl")) exit(1);
	SoSeparator *armFlameObj = SoDB::readAll(&armFlameIpt);
	if (armFlameObj == NULL) exit(1);

	//color
	SoMaterial *GoalColor = new SoMaterial;
	GoalColor->diffuseColor.setValue(0.7, 0.7, 0.7);

	//�ʒu���킹
	SoTranslation *GoalTranslation = new SoTranslation;
	GoalTranslation->translation.setValue(0.11, -0.916, -0.210);

	SoTransform *GoalInitialrot = new SoTransform;	//Y

	armFlame->addChild(GoalColor);
	armFlame->addChild(GoalTranslation);
	//GoalSep->addChild(GoalTransform);
	armFlame->addChild(armFlameObj);
}

void AlmiFlame2(SoSeparator *armFlame) {
	///////////////////////////////////////

	SoInput armFlameIpt;
	if (!armFlameIpt.openFile("hfsh8-8080-1000_vertical.wrl")) exit(1);
	SoSeparator *armFlameObj = SoDB::readAll(&armFlameIpt);
	if (armFlameObj == NULL) exit(1);

	//color
	SoMaterial *GoalColor = new SoMaterial;
	GoalColor->diffuseColor.setValue(0.7, 0.7, 0.7);

	//�ʒu���킹
	SoTranslation *GoalTranslation = new SoTranslation;
	GoalTranslation->translation.setValue(-0.11, -0.916, -0.210);

	SoTransform *GoalInitialrot = new SoTransform;	//Y

	armFlame->addChild(GoalColor);
	armFlame->addChild(GoalTranslation);
	//GoalSep->addChild(GoalTransform);
	armFlame->addChild(armFlameObj);
}
*/

/*
void JointLimit() {
if (currentJointRad[0] > PI) { currentJointRad[0] = PI; }
if (currentJointRad[0] < 0) { currentJointRad[0] = 0; }

if (currentJointRad[1] > PI / 2) { currentJointRad[1] = PI / 2; }
if (currentJointRad[1] < -PI / 2) { currentJointRad[1] = -PI / 2; }

if (currentJointRad[2] > PI / 2) { currentJointRad[2] = PI / 2; }
if (currentJointRad[2] < -PI / 2) { currentJointRad[2] = -PI / 2; }

if (currentJointRad[3] > PI / 2) { currentJointRad[3] = PI / 2; }
if (currentJointRad[3] < 0) { currentJointRad[3] = 0; }

if (currentJointRad[4] > PI) { currentJointRad[4] = PI; }
if (currentJointRad[4] < -PI) { currentJointRad[4] = -PI; }

if (currentJointRad[5] > PI / 2) { currentJointRad[5] = PI / 2; }
if (currentJointRad[5] < -PI / 2) { currentJointRad[5] = -PI / 2; }

if (currentJointRad[6] > PI) { currentJointRad[6] = PI; }
if (currentJointRad[6] < -PI) { currentJointRad[6] = -PI; }
}

*******************�r�����ʒu*******************************
baseTrans->translation.setValue(0.0, 0.0, 0.0);
arm1Trans->translation.setValue(0.0, 0.0, 0.01);
arm2Trans->translation.setValue(0.0, 0.0, 0.154);
arm3Trans->translation.setValue(0.0, 0.0, 0.0);
arm4Trans->translation.setValue(0.0, 0.0, 0.322);
arm5Trans->translation.setValue(0.0, -0.22, 0.0);
arm6Trans->translation.setValue(0.0, 0.0, 0.037);
arm7Trans->translation.setValue(0.0, -0.022, 0.0);
**********************************************************
*******************��Ώ�����!!!!!**************************
*******************�r�����ʒu REFINE*******************************

baseTrans->translation.setValue(0.0, 0.0, 0.0);
arm1Trans->translation.setValue(0.0, 0.0, 0.01);
arm2Trans->translation.setValue(0.0, 0.0, 0.158);
arm3Trans->translation.setValue(0.0, 0.007, 0.0);
arm4Trans->translation.setValue(0.0, 0.0, 0.329);
arm5Trans->translation.setValue(0.0, -0.238, 0.0);
arm6Trans->translation.setValue(0.0, 0.0, 0.020);
arm7Trans->translation.setValue(0.0, -0.007, 0.0);
*/

/*
//�A�[���̉�]
///////////////////////////////////////
SorotXYZ *baserot = new SorotXYZ;
SorotXYZ *arm1rot = new SorotXYZ;
SorotXYZ *arm2rot = new SorotXYZ;
SorotXYZ *arm3rot = new SorotXYZ;
SorotXYZ *arm4rot = new SorotXYZ;
SorotXYZ *arm5rot = new SorotXYZ;
SorotXYZ *arm6rot = new SorotXYZ;
SorotXYZ *arm7rot = new SorotXYZ;
///////////////////////////////////////

//��]�p
///////////////////////////////////////
currentJointRad1++;
if (currentJointRad1 == 180) { currentJointRad1 = 0; }
baserot->angle = -M_PI / 2;	//�����͉񂳂Ȃ�
arm1rot->angle = currentJointRad1*M_PI / 180;	//currentJointRad1;
arm2rot->angle = -M_PI / 4;	//currentJointRad2;
arm3rot->angle = M_PI / 3;		//currentJointRad3;
arm4rot->angle = -M_PI / 2;	//currentJointRad4;
arm5rot->angle = M_PI / 3;		//currentJointRad5;
arm6rot->angle = -M_PI / 4;	//currentJointRad6;
arm7rot->angle = M_PI / 3;		//currentJointRad7;
///////////////////////////////////////

//��]��
///////////////////////////////////////
baserot->axis = SorotXYZ::Y;
arm1rot->axis = SorotXYZ::Z;
arm2rot->axis = SorotXYZ::X;
arm3rot->axis = SorotXYZ::Y;
arm4rot->axis = SorotXYZ::X;
arm5rot->axis = SorotXYZ::Y;
arm6rot->axis = SorotXYZ::X;
arm7rot->axis = SorotXYZ::Y;
///////////////////////////////////////

//rot���I�u�W�F�N�g�ɒǉ�
///////////////////////////////////////
arms->addChild(baserot);
basesep->addChild(arm1rot);
arm1sep->addChild(arm2rot);
arm2sep->addChild(arm3rot);
arm3sep->addChild(arm4rot);
arm4sep->addChild(arm5rot);
arm5sep->addChild(arm6rot);
arm6sep->addChild(arm7rot);
///////////////////////////////////////
*/