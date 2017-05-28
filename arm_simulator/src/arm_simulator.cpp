// 2017/04/28
// created by eiki obara
//7自由度アームシミュレータの心臓部。
//主に3次元モデルの生成、動作に関するコールバック関数(callback)
//キーボード操作受けつけ関数(mykeypress)
//運動学計算(calcCore)はmyKeyPressの中で，様々な処理を受ける

///////////////////////////////////////////////////////////////////
//注意
//c:\coin3d\include\inventor\system\inttypes.h
//の
//#if !defined(HAVE_INT8_T) && defined(COIN_INT8_T)
//typedef COIN_INT8_T int8_t;
//#define HAVE_INT8_T 1
//#endif /* !HAVE_INT8_T && COIN_INT8_T */
//を定義していません
//他のプログラムでエラーが出る可能性あり．
/////////////////////////////////////////////////
//関数は先頭文字が大文字，変数は小文字，単語ごとに頭文字が大文字
//定数は全部大文字
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
//coin3Dと運動学で共通の変数．
//coin3Dの引数が変えられないからグローバルにするしか無いものをここに置く

//計算結果の描画を何回に一回にするか
const double LOOP_SPAN = kine::TIME_SPAN * 1;

//経由点を目標点の半径いくらに設定するか．単位はメートル
const double VIA_LENGTH = 0.1;

//さくらんぼの目標位置．単位はm(メートル)
static TarPoints cherryPos;

//関節の角度を格納する
static double currentJointRad[kine::MAXJOINT] = {};

//手先カメラの位置
static double handCameraCoordinate[3] = {};

//Recording array
double positionRec[3 * 100000] = {};
double velocityRec[7 * 100000] = {};

static int loopnum = 0;

///////////////////////////////////////////////////////////////////////////////
//ここから関数群
//目標座標設定
void pointInitialize() {
	//初期姿勢時の手先位置
	cherryPos.mid = { 0.601, -0.04, 0.0 };

	//搬送部の位置
	//cherryPos.mid = { 0.193, -0.435, -0.250 };

	//キャリブレーション治具から10cm引き抜いた所
	//cherryPos.mid = { -0.116, -0.589, 0.0 };

	cherryPos.top = { cherryPos.mid[0] + 0.2,	cherryPos.mid[1] + 0.0,	cherryPos.mid[2] + 0.0 };
	cherryPos.btm = { cherryPos.mid[0] - 0.0,	cherryPos.mid[1] - 0.2,	cherryPos.mid[2] - 0.0 };
}

//アーム各関節の初期値を与える関数．グローバル変数へのアクセスなので．
//プログラムのどこからでも起動できるから注意
void CurrentJointRadInit() {
	currentJointRad[0] = 60 * M_PI / 180 - M_PI / 2;//肩ロール
	currentJointRad[1] = 0 * M_PI / 180 + M_PI / 2;	//肩ピッチ
	currentJointRad[2] = 0 * M_PI / 180 + M_PI / 2;	//肩ヨー
	currentJointRad[3] = 70 * M_PI / 180;			//肘
	currentJointRad[4] = 0 * M_PI / 180;			//手首ロール
	currentJointRad[5] = -40 * M_PI / 180;			//手首ピッチ
	currentJointRad[6] = 0 * M_PI / 180;			//手先ロール
	currentJointRad[7] = 0;							//使わないパラメータ
}

void WriteOut() {
	printf("output velocity start\n");

	//OutputMatrixTxt(positionRec, 3, loopnum, "handPosi_");
	OutputMatrixTxt(velocityRec, 8, loopnum, "Velo_");

	printf("output velocity ended\n");
}

//スペースマウスから受け取る数値を格納する
//float spaceMouseInput[6];

//使い方の説明
void DisplayDescription() {
	DebagBar();
	printf("==============================================\n");
	printf("= 使用方法\n");
	printf("= 1.シミュレーション画面をクリックでアクティブにする\n");
	printf("= 2.Escキーでキーボード操作モードに切り替える\n");
	printf("= 3.シミュレーション開始可能になります\n\n");

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

//パラメータの表示
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

//逆運動学を計算
int CalcInverseKinematics(double *speed, kine::Kinematics *arm, bool selfMotion) {
	//エラーチェック
	int errorCheck = 0;

	//CalcIKから返ってくる計算値
	double nextRadVelocity[kine::MAXJOINT] = {};

	//セルフモーション発動
	if (selfMotion == 1) errorCheck = arm->CalcIK(currentJointRad, speed, nextRadVelocity);
	
	//擬似逆行列法
	//else if (selfMotion == 0) errorCheck = arm->CalcPIK(currentJointRad, speed, nextRadVelocity);
	
	//エラーチェック
	if (errorCheck > 0) return 1;

	//出力された関節角速度を現在の関節角度に足す．
	for (int i = 0; i < kine::MAXJOINT - 1; i++)	currentJointRad[i] += nextRadVelocity[i];
	
	return 0;
}

//指先移動軌道を計算
void PositionSpeed(double *currentTime, kine::Kinematics *arm, double *speed) {
	//手先移動速度の算出
	double firstPos[3] = {};
	double viaPos[3] = {};
	double endPos[3] = {};

	double positionBuf[3] = {};

	//手先位置
	arm->GetCoordinate(firstPos);
	for (int i = 0; i < 3; ++i) endPos[i] = cherryPos.mid[i];

	//経由点の計算
	CalcViaPos(cherryPos, VIA_LENGTH, viaPos);

	//直線軌道
	//CalcVelocityLinear(firstPos, endPos, *currentTime, positionBuf);

	//スプライン
	CalcVelocitySpline(firstPos, viaPos, endPos, *currentTime, positionBuf);

	//DebagComment("position buffer");	DisplayVector(3, positionBuf);

	for (int i = 0; i < 3; ++i) speed[i] = positionBuf[i];
}

//手先姿勢計算
void PostureSpeed(double *currentTime, double *speed) {
	//手先姿勢の速度算出/////////////////////////////////////////////////////////
	//speedは　3 -> roll, 4 -> yaw , 5 -> pitch
	double postureBuf[3] = {};

	CalcVelocityPosture(currentJointRad, &cherryPos, *currentTime, postureBuf);
	//DebagComment("posture buffer");	DisplayVector(3, postureBuf);

	//計算した指先軌道，手先姿勢動作速度の代入
	for (int i = 0; i < 3; ++i) speed[i + 3] = postureBuf[i];
	//speed[6] = 0;
}

//運動学計算関数を実行する関数．LOOP_SPANの回数ループさせる
void CalcCore(double *currentTime, kine::Kinematics *arm, bool selfMotion, double *speed) {
	//calcCoreのループタイマー
	double loopTimer = 0;

	//エラーチェック
	int errorCheck = 0;

	while (1) {
		//printf("currentTime -> %lf\n", *currentTime);

		//手先位置の算出
		arm->CalcFK(currentJointRad);

		//手先カメラの算出
		//CalcHandCameraPosition(currentJointRad, handCameraCoordinate);

		//軌道計算
		PositionSpeed(currentTime, arm, speed);

		//手先姿勢計算
		PostureSpeed(currentTime, speed);

		//ヤコビ行列を用いた逆運動学の計算
		errorCheck = CalcInverseKinematics(speed, arm, selfMotion);

		if (errorCheck == 1) {
			ErrComment("*** calculation error happened ***");
			//WriteOut();
			exit(1);
		}

		//書き出し用配列
		velocityRec[loopnum * 8 + 0] = *currentTime;	//デバッグ用
		for (int i = 1; i < 8; ++i) velocityRec[8 * loopnum + i] = speed[i - 1];

		//アウトプット用の書き出し回数カウンタ
		++loopnum;

		//現在時間の更新
		*currentTime += kine::TIME_SPAN;

		//シミュレータ上で描画する時間。
		//下の条件文を抜けるとグラフィクスを描画する。
		loopTimer += kine::TIME_SPAN;
		if (loopTimer >= LOOP_SPAN) break;
	}

	//DebagComment("calcCore : finished");
}

//目標座標の変更を行う関数
void ChengeGoalValue(kine::Kinematics *arm){
	printf("Started --- Input Amount of movement ---\n");

	//軸選択
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

//座標（ｘ，ｙ，ｚ）により操作するキーボードコールバック関数
void SimulatorKeyPressCB(void *userData, SoEventCallback *eventCB) {
	//clock_t start, end;
	//start = clock();

	//肘，手首，手先位置を格納する
	static kine::Kinematics arm;
	
	//手先速度格納配列
	double speed[7] = {};

	//エラーチェックフラグ
	int check = 0;

	//経過時間格納変数
	static double CurrentTime = 0;

	//計算機起動フラグ	FALSEならoff,TRUEならon
	bool calcSwitch = FALSE;
	//セルフモーションフラグ　True なら ON
	int selfMotionOn = TRUE;

	//関節変数の初期化フラグ
	static bool currentJointRadInitSwitch = TRUE;

	const SoEvent *event = eventCB->getEvent();

	//関節変数の初期化
	if (currentJointRadInitSwitch == TRUE) { 
		CurrentJointRadInit();
		currentJointRadInitSwitch = FALSE;
		pointInitialize();
	}

	//スペースマウスの値取得
	/*
	if (event->isOfType(SoMotion3Event::getClassTypeId())) {
		SoMotion3Event* tr = (SoMotion3Event*)event;
		spaceMouseInput[0] = SPACEMOUSE_TRANSLATION_GAIN * tr->getTranslation().getValue()[0];	//x軸
		spaceMouseInput[1] = SPACEMOUSE_TRANSLATION_GAIN * tr->getTranslation().getValue()[1];	//y軸
		spaceMouseInput[2] = SPACEMOUSE_TRANSLATION_GAIN * tr->getTranslation().getValue()[2];	//z軸
		spaceMouseInput[3] = SPACEMOUSE_ROTATION_GAIN * tr->getRotation().getValue()[0];		//α
		spaceMouseInput[4] = SPACEMOUSE_ROTATION_GAIN * tr->getRotation().getValue()[1];		//y軸回り回転
		spaceMouseInput[5] = SPACEMOUSE_ROTATION_GAIN * tr->getRotation().getValue()[2];

		//speed[3] = -spaceMouseInput[4];	//ロール
		speed[4] = -spaceMouseInput[5];		//ヨー
		speed[5] = spaceMouseInput[3];		//ピッチ

		calcSwitch = 1;
	}
	*/

	if (SO_KEY_PRESS_EVENT(event, T)) {
		ChengeGoalValue(&arm);
		CurrentTime = 0;	//Reset VelocityRegulation
	}

	//必要な計算等すべてを行うキー　A (all)
	if (SO_KEY_PRESS_EVENT(event, A)) {
		calcSwitch = TRUE;
		selfMotionOn = 1;
		speed[6] = 0.0;
	}

	//手先位置を初期姿勢に戻すキー　R (restart)
	if (SO_KEY_PRESS_EVENT(event, R)) {
		system("cls");
		currentJointRadInitSwitch = TRUE;
		CurrentTime = 0;
		DisplayDescription();
	}

	//セルフモーション
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

	//手先操作
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

	//パラメータ表示
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

	//計算プログラム開始
	if (calcSwitch == TRUE) { 
		CalcCore(&CurrentTime, &arm, selfMotionOn, speed);
	}
	
	if (SO_KEY_PRESS_EVENT(event, W)) {
		WriteOut();		
	}

	//プログラム終了
	if (SO_KEY_PRESS_EVENT(event, Q)) { 
		//DebagCom("program finished");
		exit(1);
	}

	//SpeedIntegration(speed);

	if (check > 0) {
		ErrComment("\n\n\n*** fatal error ***\n*** please enter 'q' for finish this program ***\n");
		exit(1);
	}

	//end = clock();	printf("処理時間:%lf[ms]\n",(double)(end - start)/CLOCKS_PER_SEC);
	//DebagComment("call back finished\n");
}

/*SoWinSpaceball* SpaceMouseSet() {
	float f;
	if (SoWinSpaceball::exists()) {
		printf("スペースマウスを検出できません\n");
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




//各関節の回転コールバック
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
	//説明文の表示
	DisplayDescription();

	pointInitialize();

	//画面の初期化
	HWND myWindow = SoWin::init("");

	//プログラムツリーの始まり
	SoSeparator *root = new SoSeparator;
	SoSeparator *arms = new SoSeparator;
	
	//各オブジェクトツリーのセパレータの作成
	SoSeparator *basesep = new SoSeparator;	SoSeparator *arm1sep = new SoSeparator;
	SoSeparator *arm2sep = new SoSeparator;	SoSeparator *arm3sep = new SoSeparator;
	SoSeparator *arm4sep = new SoSeparator;	SoSeparator *arm5sep = new SoSeparator;
	SoSeparator *arm6sep = new SoSeparator;	SoSeparator *arm7sep = new SoSeparator;

	//スペースマウスの確認
	//if (SoWinSpaceball::exists()) { printf("spaceball No Exists\n");	exit(1); }
	//spacemouse callback
	//SoEventCallback *spaceMouseCB = new SoEventCallback;
	//spaceMouseCB->addEventCallback(SoMotion3Event::getClassTypeId(), MyKeyPressCB, root);

	//コールバック
	SoEventCallback *eventcallback = new SoEventCallback;
	eventcallback->addEventCallback(SoKeyboardEvent::getClassTypeId(), SimulatorKeyPressCB, root);

	//目標描画
	SoSeparator *cherryPosSep = new SoSeparator;
	GoalCherry(cherryPos.mid, cherryPosSep);

	//カメラ位置確認用赤玉
	SoSeparator *redPointSep = new SoSeparator;

	SoTranslation *redPointTrans = new SoTranslation;
	SoTimerSensor *redPointTransMoveSensor = new SoTimerSensor(RedPointSensorCallback, redPointTrans);
	redPointTransMoveSensor->setInterval(0.01);
	redPointTransMoveSensor->schedule();
	
	redPointSep->addChild(redPointTrans);
	PointObj(redPointSep);

	//////////////////////////////////////////
	//目標座標三点確認用
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

	//支柱のアルミフレーム
	SoSeparator *almiFlame1 = new SoSeparator;
	SoSeparator *almiFlame2 = new SoSeparator;

	//std::vector<double> alm1Position = { 0.11, -0.916, -0.210 };
	std::vector<double> alm1Position = { 0.0, -0.916, -0.210 };
	std::vector<double> alm2Position = { -0.08, -0.916, -0.210 };

	AlmiFlame(alm1Position, almiFlame1);
	AlmiFlame(alm2Position, almiFlame2);

	//座標系表示
	SoSeparator *coordinateSystem = new SoSeparator;
	CoordinateSystem(coordinateSystem);

	//座標変換用変数 arm(x)trans xは対応する関節番号
	SoTranslation *baseTrans = new SoTranslation;	SoTranslation *arm1Trans = new SoTranslation;
	SoTranslation *arm2Trans = new SoTranslation;	SoTranslation *arm3Trans = new SoTranslation;
	SoTranslation *arm4Trans = new SoTranslation;	SoTranslation *arm5Trans = new SoTranslation;
	SoTranslation *arm6Trans = new SoTranslation;	SoTranslation *arm7Trans = new SoTranslation;

	//部品読み込み
	SoInput baseIpt;	SoInput arm1Ipt;	SoInput arm2Ipt;	SoInput arm3Ipt;
	SoInput arm4Ipt;	SoInput arm5Ipt;	SoInput arm6Ipt;	SoInput arm7Ipt;

	if (argc == 0) {
		//超簡易版アーム/////////////////////////////////////////////////
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
		//カメラ付きアーム版//////////////////////////////////////////////
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

	//オブジェクトをセパレータに格納////////////////////////////////////////////////////////////////////////
	SoSeparator *baseObj = SoDB::readAll(&baseIpt);	SoSeparator *arm1Obj = SoDB::readAll(&arm1Ipt);
	SoSeparator *arm2Obj = SoDB::readAll(&arm2Ipt);	SoSeparator *arm3Obj = SoDB::readAll(&arm3Ipt);
	SoSeparator *arm4Obj = SoDB::readAll(&arm4Ipt);	SoSeparator *arm5Obj = SoDB::readAll(&arm5Ipt);
	SoSeparator *arm6Obj = SoDB::readAll(&arm6Ipt);	SoSeparator *arm7Obj = SoDB::readAll(&arm7Ipt);
	//オブジェクトをセパレータに格納////////////////////////////////////////////////////////////////////////

	//.wrlの読み込みエラー判定////////////////////////////////////////////////
	if (baseObj == NULL) exit(1);	if (arm1Obj == NULL) exit(1);
	if (arm2Obj == NULL) exit(1);	if (arm3Obj == NULL) exit(1);
	if (arm4Obj == NULL) exit(1);	if (arm5Obj == NULL) exit(1);
	if (arm6Obj == NULL) exit(1);	if (arm7Obj == NULL) exit(1);
	//.wrlの読み込みエラー判定////////////////////////////////////////////////

	//関節回転コールバック/////////////////////////////////////////////////////////////////////////////////////////
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
	//関節回転コールバック/////////////////////////////////////////////////////////////////////////////////////////

	
	//部品ごとの初期設定///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	SoTransform *arm1InitialRot = new SoTransform;	arm1InitialRot->rotation.setValue(SbVec3f(1, 0, 0), 0);
	SoTransform *arm2InitialRot = new SoTransform;	arm2InitialRot->rotation.setValue(SbVec3f(1, 0, 0), -M_PI / 2);
	SoTransform *arm3InitialRot = new SoTransform;	arm3InitialRot->rotation.setValue(SbVec3f(1, 0, 0), M_PI / 2);
	SoTransform *arm4InitialRot = new SoTransform;	arm4InitialRot->rotation.setValue(SbVec3f(1, 0, 0), -M_PI / 2);
	SoTransform *arm5InitialRot = new SoTransform;	arm5InitialRot->rotation.setValue(SbVec3f(1, 0, 0), M_PI / 2);
	SoTransform *arm6InitialRot = new SoTransform;	arm6InitialRot->rotation.setValue(SbVec3f(1, 0, 0), -M_PI / 2);
	SoTransform *arm7InitialRot = new SoTransform;	arm7InitialRot->rotation.setValue(SbVec3f(1, 0, 0), M_PI / 2);
	//部品ごとの初期設定///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//セパレータツリー作成///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

	//セパレータツリーの作成///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//部品組み立て部分///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
	//部品組み立て部分///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//ビュワーの設定///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	SoWinExaminerViewer *myViewer = new SoWinExaminerViewer(myWindow);
	//myViewer->registerDevice(SpaceMouseSet());
	myViewer->setSceneGraph(root);
	myViewer->setTitle("simulator 7dof arm");
	myViewer->setBackgroundColor(SbColor(0.3, 0.3, 0.4));
	myViewer->setSize(SbVec2s(720, 600));
	myViewer->show();

	//ビュワーの設定///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	SoWin::show(myWindow);
	SoWin::mainLoop();
}

//*******************絶対消すな!!!!!**************************
/*
//座標点確認用サクランボ描画
void GoalCherry(SoSeparator *GoalSep) {
	///////////////////////////////////////

	SoInput GoalIpt;
	if (!GoalIpt.openFile("cherryfruit.wrl")) exit(1);
	SoSeparator *GoalObj1 = SoDB::readAll(&GoalIpt);
	if (GoalObj1 == NULL) exit(1);

	//color
	SoMaterial *GoalColor = new SoMaterial;
	GoalColor->diffuseColor.setValue(1, 0, 0);

	//位置合わせ
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

//アルミフレーム描画
void AlmiFlame1(SoSeparator *armFlame) {
	///////////////////////////////////////

	SoInput armFlameIpt;
	if (!armFlameIpt.openFile("hfsh8-8080-1000_vertical.wrl")) exit(1);
	SoSeparator *armFlameObj = SoDB::readAll(&armFlameIpt);
	if (armFlameObj == NULL) exit(1);

	//color
	SoMaterial *GoalColor = new SoMaterial;
	GoalColor->diffuseColor.setValue(0.7, 0.7, 0.7);

	//位置合わせ
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

	//位置合わせ
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

*******************腕初期位置*******************************
baseTrans->translation.setValue(0.0, 0.0, 0.0);
arm1Trans->translation.setValue(0.0, 0.0, 0.01);
arm2Trans->translation.setValue(0.0, 0.0, 0.154);
arm3Trans->translation.setValue(0.0, 0.0, 0.0);
arm4Trans->translation.setValue(0.0, 0.0, 0.322);
arm5Trans->translation.setValue(0.0, -0.22, 0.0);
arm6Trans->translation.setValue(0.0, 0.0, 0.037);
arm7Trans->translation.setValue(0.0, -0.022, 0.0);
**********************************************************
*******************絶対消すな!!!!!**************************
*******************腕初期位置 REFINE*******************************

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
//アームの回転
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

//回転角
///////////////////////////////////////
currentJointRad1++;
if (currentJointRad1 == 180) { currentJointRad1 = 0; }
baserot->angle = -M_PI / 2;	//ここは回さない
arm1rot->angle = currentJointRad1*M_PI / 180;	//currentJointRad1;
arm2rot->angle = -M_PI / 4;	//currentJointRad2;
arm3rot->angle = M_PI / 3;		//currentJointRad3;
arm4rot->angle = -M_PI / 2;	//currentJointRad4;
arm5rot->angle = M_PI / 3;		//currentJointRad5;
arm6rot->angle = -M_PI / 4;	//currentJointRad6;
arm7rot->angle = M_PI / 3;		//currentJointRad7;
///////////////////////////////////////

//回転軸
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

//rotをオブジェクトに追加
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