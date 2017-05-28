//cone Simulator
// 2017/04/28
// created by eiki obara

#include "include\kine_quat.h"
#include "include\kine_vector.h"
#include "include\kine_debag.h"
#include "include\cone_simulator.h"
#include "include\3Dobjects.h"
#include "include\kine_target_point.h"
#include "include\kine_config.h"
#include "include\kine_convertor.h"
#include "include\kinematics.h"
#include "include\trapezoidal_interpolation.h"

#include <Inventor\Win\SoWin.h>
#include <Inventor\Win\viewers\SoWinExaminerViewer.h>
#include <Inventor\so.h>

//データ格納用配列
double outputEuler[4 * 100000] = {};
double outputAngular[4 * 100000] = {};

//動作ループカウンター
int counter = 0.0;

float quaternion[4] = {1,0,0,0};

Quat firstPostureQ(
	0.5,	//w
	0.5,	//x
	0.5,	//y
	0.5		//z
	);

Quat endPostureQ(
	0.5,	//w
	0.5,	//x
	0.5,	//y
	0.5		//z
	);

static void moveSensorCallback(void *b_data, SoSensor *){
	SoTransform *rot = (SoTransform *)b_data;
	rot->center.setValue(0, 0.0, 0);
	rot->rotation.setValue(quaternion);
}

void KeyPressCB(void *userData, SoEventCallback *eventCB) {
	//現在時間
	static double currentTime = 0;

	static double nowTime = 0;
	static double beforeTime = 0;
	static double constVelocity[3] = {};

	//初期姿勢クォータニオン

	TarPoints centerPoint;

	const SoEvent *event = eventCB->getEvent();

	if (SO_KEY_PRESS_EVENT(event, SPACE)) {
		int whileCounter = 0.0;

		while (1) {
			if (currentTime == 0.0) {
				double bufm[16] = {};
				double bufArray[4] = {};
				firstPostureQ.Quat2array(bufArray);

				for (int i = 0; i < 4; ++i) quaternion[i] = bufArray[i];

				DebagComment("first\n");
				firstPostureQ.display();
				firstPostureQ.quat2RotM(bufm);
				DisplayRegularMatrix(4, bufm);

				DebagComment("end\n");
				endPostureQ.display();
				endPostureQ.quat2RotM(bufm);
				DisplayRegularMatrix(4, bufm);

				beforeTime = 0.0;
				nowTime = 0.0;
			}
			else if (currentTime >= kine::TIME_SPAN && currentTime < kine::TIME_LENGTH) {
				printf("currentTime -> %3.9lf\n", currentTime);

				beforeTime = nowTime;
				nowTime += TrapeInterpolate(1, kine::TIME_LENGTH, currentTime);

				//printf("now time %lf\n", nowTime);
				//printf("before time %lf\n", beforeTime);

				Quat currentSlerpQ = firstPostureQ.slerp(nowTime, endPostureQ);
				Quat beforeSlerpQ = firstPostureQ.slerp(beforeTime, endPostureQ);
				
				double q[4] = {};

				currentSlerpQ.Quat2array(q);

				quaternion[0] = q[1];
				quaternion[1] = q[2];
				quaternion[2] = q[3];
				quaternion[3] = q[0];

				double bufEulerN[3] = {};
				double bufEulerB[3] = {};
				
				currentSlerpQ.quat2Euler(bufEulerN);
				beforeSlerpQ.quat2Euler(bufEulerB);

				double eulerVelocity[3] = {};

				if (currentTime < kine::ACCEL_TIME) {
					for (int i = 0; i < 3; ++i) {
						eulerVelocity[i] = bufEulerN[i] - bufEulerB[i];
						constVelocity[i] = eulerVelocity[i];
					}
				}
				else if (currentTime >= kine::ACCEL_TIME && currentTime <= (kine::TIME_LENGTH - kine::ACCEL_TIME)) {
					for (int i = 0; i < 3; ++i) eulerVelocity[i] = constVelocity[i];
				}
				else if (currentTime > (kine::TIME_LENGTH - kine::ACCEL_TIME) && currentTime < kine::TIME_LENGTH) {
					for (int i = 0; i < 3; ++i) eulerVelocity[i] = bufEulerN[i] - bufEulerB[i];
				}

				//DisplayVector(3, eulerVelocity);

				double bufAngular[3] = {};

				Euler2Angular(bufEulerN, eulerVelocity, bufAngular);

				//output parametor
				outputEuler[counter * 4] = currentTime;
				outputAngular[counter * 4] = currentTime;

				for (int i = 0; i < 3; ++i) outputEuler[counter * 4 + i + 1] = bufEulerN[i];
				for (int i = 0; i < 3; ++i) outputAngular[counter * 4 + i + 1] = bufAngular[i];

				if (currentTime > kine::TIME_LENGTH && currentTime <= kine::TIME_LENGTH + kine::TIME_SPAN) DebagComment("slerp conplete");

				//if ((whileCounter * TIME_SPAN) >= LOOP_SPAN) break;
			}
			else if (currentTime >= kine::TIME_LENGTH) {
				printf("finished\n");
				break;
			}

			currentTime += kine::TIME_SPAN;
			++counter;
			++whileCounter;

			if (whileCounter >= kine::TIME_LENGTH * kine::TIME_SPAN) {
				break;
			}
		}
	}

	if (SO_KEY_PRESS_EVENT(event, R)) {
		currentTime = 0.0;

		nowTime = 0;
		beforeTime = 0;
	}

	if (SO_KEY_PRESS_EVENT(event, W)) {
		OutputMatrixTxt(outputEuler, 4, counter, "SlerpTest_Euler");
		OutputMatrixTxt(outputAngular, 4, counter, "SlerpTest_anglaur");
	}

	if (SO_KEY_PRESS_EVENT(event, Q)) {
		exit(1);
	}
}

void coneObject(SoSeparator *sep) {
	SoSeparator *sep1 = new SoSeparator;
	SoSeparator *sep2 = new SoSeparator;
	SoSeparator *sep3 = new SoSeparator;

	//オブジェクトの生成
	SoCone *cone1Obj = new SoCone;
	SoCone *cone2Obj = new SoCone;
	SoCone *cone3Obj = new SoCone;

	//オブジェクトのパラメータ
	cone1Obj->bottomRadius = 0.01;
	cone1Obj->height = 0.1;

	cone2Obj->bottomRadius = 0.01;
	cone2Obj->height = 0.1;

	cone3Obj->bottomRadius = 0.01;
	cone3Obj->height = 0.1;

	//部品同士の位置決め
	SoTranslation *cone1Trans = new SoTranslation;
	SoTranslation *cone2Trans = new SoTranslation;
	SoTranslation *cone3Trans = new SoTranslation;

	cone1Trans->translation.setValue(0.0, 0.05, 0.0);
	cone2Trans->translation.setValue(0.0, 0.05, 0.0);
	cone3Trans->translation.setValue(0.0, 0.05, 0.0);

	SoTransform *cone1rot = new SoTransform;
	cone1rot->center.setValue(0, 0, 0.0);
	cone1rot->rotation.setValue(SbVec3f(1, 0, 0), M_PI / 2);

	SoTransform *cone2rot = new SoTransform;
	cone2rot->center.setValue(0, 0, 0.0);
	//cone2rot->rotation.setValue(SbVec3f(0, 1, 0), M_PI / 2);

	SoTransform *cone3rot = new SoTransform;
	cone3rot->center.setValue(0, 0, 0.0);
	cone3rot->rotation.setValue(SbVec3f(0, 0, 1), -M_PI / 2);

	//色（RGB）
	SoMaterial *red = new SoMaterial;
	red->diffuseColor.setValue(1.0, 0.0, 0.0);
	
	SoMaterial *blue = new SoMaterial;
	blue->diffuseColor.setValue(0.0, 0.0, 1.0);
	
	SoMaterial *green= new SoMaterial;
	green->diffuseColor.setValue(0.0, 1.0, 0.0);

	sep1->addChild(blue);	//x
	sep1->addChild(cone1rot);
	sep1->addChild(cone1Trans);
	sep1->addChild(cone1Obj);

	sep2->addChild(green);	//y
	sep2->addChild(cone2rot);
	sep2->addChild(cone2Trans);
	sep2->addChild(cone2Obj);

	sep3->addChild(red);	//z
	sep3->addChild(cone3rot);
	sep3->addChild(cone3Trans);
	sep3->addChild(cone3Obj);

	sep->addChild(sep1);
	sep->addChild(sep2);
	sep->addChild(sep3);
}

void ConeSimulator() {
	HWND coneWindow = SoWin::init("");

	SoSeparator *root = new SoSeparator;
	
	//event call back
	SoEventCallback *eventcallback = new SoEventCallback;
	eventcallback->addEventCallback(SoKeyboardEvent::getClassTypeId(), KeyPressCB, root);
	
	SoSeparator *coneSep = new SoSeparator;

	SoSphere *origin = new SoSphere;

	origin->radius = (0.01);

	//cone generate
	SoSeparator *coneObj = new SoSeparator;
	coneObject(coneObj);

	//rotation callback
	SoTransform *coneRot = new SoTransform;
	SoTimerSensor *coneMoveSensor = new SoTimerSensor(moveSensorCallback, coneRot);

	coneSep->addChild(coneRot);
	coneSep->addChild(coneObj);

	coneMoveSensor->setInterval(0.01);
	coneMoveSensor->schedule();

	SoSeparator *coord = new SoSeparator;
	CoordinateSystem(coord);

	//addchild
	root->ref();
	root->addChild(coord);
	root->addChild(origin);
	root->addChild(eventcallback);
	root->addChild(coneSep);
	
	//viewer configuration
	SoWinExaminerViewer *coneViewer = new SoWinExaminerViewer(coneWindow);

	coneViewer->setSceneGraph(root);
	coneViewer->setTitle("*** Cone Simulator ***");
	coneViewer->setBackgroundColor(SbColor(0.3, 0.3, 0.4));
	coneViewer->setSize(SbVec2s(720, 600));
	coneViewer->show();

	SoWin::show(coneWindow);
	SoWin::mainLoop();
}