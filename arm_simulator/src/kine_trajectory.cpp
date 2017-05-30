#include "include\kine_trajectory.h"

#include "include\kine_config.h"
#include "include\kine_spline.h"
#include "include\trapezoidal_interpolation.h"
#include "include\kine_target_point.h"
#include "include\kine_quat.h"
#include "include\kine_convertor.h"
#include "include\kinematics.h"
#include "include\kine_debag.h"
#include "include\kine_vector.h"

//三点の情報を与えると，経由点を持つスプライン曲線を生成する．
void CalcVelocitySpline(double *firstPos3, double *viaPos3, double *endPos3, double currentTime, double *moveSpeed3) {
	static double nowT = 0;
	static double beforeT = 0;

	//DebagComment("CalcVelocitySpline : started");
	
	//DisplayVector(3, firstPos3);
	//DisplayVector(3, viaPos3);
	//DisplayVector(3, endPos3);

	double pointX[3] = { firstPos3[0], viaPos3[0], endPos3[0] };
	double pointY[3] = { firstPos3[1], viaPos3[1], endPos3[1] };
	double pointZ[3] = { firstPos3[2], viaPos3[2], endPos3[2] };

	static Spline routeX;
	static Spline routeY;
	static Spline routeZ;

	if (currentTime == 0) {//初期化
						   //DebagComment("move velocity initialize");
		routeX.initPoint(pointX, 3);
		routeY.initPoint(pointY, 3);
		routeZ.initPoint(pointZ, 3);

		nowT = 0;
		beforeT = 0;

		for (int i = 0; i < 3; ++i) moveSpeed3[i] = 0.0;
	}
	else if (currentTime > 0) {//現在時間が0より大きければ
		//DebagComment("move velocity calculation");
		beforeT = nowT;
		//動作は節点数ではなくリンクの数
		nowT += TrapeInterpolate(kine::ROUTE_LINK, kine::TIME_LENGTH, currentTime);

		//printf("beforeT = %lf\n", beforeT);
		//printf("nowT = %lf\n", nowT);
		
		moveSpeed3[0] = (routeX.calc(nowT) - routeX.calc(beforeT));
		moveSpeed3[1] = (routeY.calc(nowT) - routeY.calc(beforeT));
		moveSpeed3[2] = (routeZ.calc(nowT) - routeZ.calc(beforeT));

		//DebagComment("calc velocity spline : moveSpeed3");	DisplayVector(3, moveSpeed3);
	}
	else if (currentTime > kine::TIME_LENGTH) {//現在時間が動作時間を超したら。
		for (int i = 0; i < 3; ++i) moveSpeed3[i] = 0.0;
	}

	//DebagComment("move speed");	DisplayVector(3, moveSpeed);
}

void CalcVelocityLinear(double *firstPos3, double *endPos3, double currentTime, double *moveSpeed3) {
	int i;
	static double p2pLength[3];

	//DisplayCoordinate();

	//さくらんぼの茎に対して直接アプローチするとぶつかるので，
	//茎の手前に一度移動し，把持面と平行にアプローチする．
	//通過点(茎手前)
	if (currentTime < kine::TIME_SPAN) {
		for (int i = 0; i < 3; ++i)	p2pLength[i] = endPos3[i] - firstPos3[i];
	}

	//DebagComment("p2p length");
	//DisplayVector(3, p2pLength);

	//台形補間の速度計算
	for (int i = 0; i < 3; ++i) {
		moveSpeed3[i] = TrapeInterpolate(p2pLength[i], kine::TIME_LENGTH, currentTime);
	}
}

//velocity regulation posture関数内で使用するクォータニオン生成関数(初期姿勢)
Quat StartPosture(double *currentJointRadian) {
	kine::Kinematics posture;
	
	//DebagComment("initilize first and last posture");
	double handPostureRotMat[16] = {};

	posture.GetHandHTM(currentJointRadian, handPostureRotMat);

	Quat currentPostureQ;

	//double firstEuler[3];
	//DebagComment("first posture matrix");	DisplayRegularMatrix(4, handPostureRotMat);

	RotMat2Quat(handPostureRotMat, currentPostureQ);
	//DebagComment("first posture quat");	currentPostureQ.display();

	return currentPostureQ;
}

//velocity regulation posture関数内で使用するクォータニオン生成関数(目標姿勢)
Quat EndPosture(TarPoints *tarCoord) {
	double graspRotMat[16] = {};
	Quat targetPostureQ;
	bool graspDirectionFlag = true;
	//目標把持方向算出(direction Z)
	std::vector<double> graspV(3, 0);
	graspV = tarCoord->graspDirection();
	//DebagComment("grasp Vector");	DisplayVector(graspV);

	//目標姿勢算出(direction X)
	std::vector<double> directionX(3, 0);
	for (int i = 0; i < 3; ++i) directionX[i] = tarCoord->top[i] - tarCoord->btm[i];
	//DebagComment("direction x");	DisplayVector(directionX);

	//direction X, Zを使って回転行列を求める。
	//その時、外積を使ってYは求めます。
	int checkDirectX = 0;

	//directionXのエラー判定
	for (int i = 0; i < 3; ++i) {
		if (directionX[i] < kine::COMPARE_ZERO) ++checkDirectX;
	}

	if (checkDirectX < 3) {	//エラーじゃないなら姿勢計算する
		//回転行列を二つのベクトルから出す
		//DebagComment("function : EndPosture\ncheckDirectX < 3");
		DirectVector2RotMat(directionX, graspV, graspRotMat);
		//DebagComment("target posture matrix");	DisplayRegularMatrix(4, graspRotMat);

		//回転行列→クォータニオン
		RotMat2Quat(graspRotMat, targetPostureQ);
		//DebagComment("target posture Quat");	targetPostureQ.display();
	}
	else if (checkDirectX >= 3) {//エラーなら姿勢はロボットの正面を向くようにする。
		DebagComment("function : EndPosture\ncheckDirectX == 3");
		targetPostureQ.assign(0.5, 0.5, 0.5, 0.5);
	}

	return targetPostureQ;
}

void CalcVelocityPosture(double *curJointRad, TarPoints *targetCoord, double currentTime, double *postureSpeed) {
	//初期姿勢クォータニオン
	static Quat initPostureQ;
	//目標姿勢クォータニオン
	static Quat targetPostureQ;
	//現在のクォータニオン
	static Quat nowSlerpQ;

	Quat beforeSlerpQ;

	//速度生成監視フラグ
	static bool generateSpeedFlag = true;

	//初期姿勢などを求める
	//生成された姿勢情報を元に速度生成を行うか決定する
	if (currentTime < kine::TIME_SPAN) {
		Quat checkQ;
		double checkA[4] = {};
		int arrayNum = 0;

		//初期姿勢算出(quaternion)
		initPostureQ = StartPosture(curJointRad);
		//initPostureQ.display();

		//目標姿勢算出(Quaternion)
		targetPostureQ = EndPosture(targetCoord);
		//targetPostureQ.display();

		//同値のクォータニオンが生成されたかチェック
		checkQ = initPostureQ.sub(targetPostureQ);

		//クォータニオンを配列に格納
		checkQ.Quat2array(checkA);

		for (int i = 0; i < 4; ++i) if (checkA[i] < kine::COMPARE_ZERO) ++arrayNum;

		//クォータニオンの中身が全てゼロなら．		
		if (arrayNum == 4) {
			//DebagComment("function : velocityRegulationPosture\n quat -> same");
			generateSpeedFlag = false;
		}
		else {//そうじゃなければ
			//DebagComment("function : velocityRegulationPosture\n quat -> diff");
			generateSpeedFlag = true;
		}
	}

	//監視フラグによる分岐(false)
	if (generateSpeedFlag == false) {
		for (int i = 0; i < 3; ++i) postureSpeed[i] = 0.0;
	}
	//監視フラグによる分岐(true)
	else if (generateSpeedFlag == true) {
		static double beforeValue = 0.0;
		static double nowValue = 0.0;
		static double constVelocity[3] = {};

		//初動は速度ゼロ
		if (currentTime < kine::TIME_SPAN) {
			for (int i = 0; i < 3; ++i) postureSpeed[i] = 0.0;
			beforeValue = 0.0;
			nowValue = 0.0;
			nowSlerpQ.assign(initPostureQ);
		}
		else if (currentTime >= kine::TIME_SPAN && currentTime < kine::TIME_LENGTH) {
			//現在時間
			//printf("currentTime-> %lf\n", currentTime);

			//Slerpの０～１までの補間値
			nowValue += TrapeInterpolate(1, kine::TIME_LENGTH, currentTime);

			//DebagComment("now value");	printf("-> %3.9lf\n", nowValue);

			beforeSlerpQ.assign(nowSlerpQ);
			nowSlerpQ = initPostureQ.slerp(nowValue, targetPostureQ);

			//DebagComment("before slerp Quat");	beforeSlerpQ.display();
			//DebagComment("now slerp Quat");	nowSlerpQ.display();

			//double beforeMat[16] = {};
			//double nowMat[16] = {};

			//beforeSlerpQ.quat2RotM(beforeMat);
			//nowSlerpQ.quat2RotM(nowMat);

			double beforeSlerpEuler[3] = {};
			double nowSlerpEuler[3] = {};

			//RotMatrixToEuler(beforeMat, beforeSlerpEuler);
			//RotMatrixToEuler(nowMat, nowSlerpEuler);

			beforeSlerpQ.quat2Euler(beforeSlerpEuler);
			nowSlerpQ.quat2Euler(nowSlerpEuler);

			//////////////////////////////////////

			//DebagComment("before slerp euler");	DisplayVector(3, beforeSlerpEuler);
			//DebagComment("now slerp euler");	DisplayVector(3, nowSlerpEuler);

			double eulerVelocity[3] = {};

			for (int i = 0; i < 3; ++i) {
				double subBuf = 0.0;

				eulerVelocity[i] = (nowSlerpEuler[i] - beforeSlerpEuler[i]);
				if (fabs(eulerVelocity[i]) > 0.1) 	eulerVelocity[i] = 0.0;
			}

			//DebagComment(" euler velocity ");	DisplayVector(3, eulerVelocity);

			double postureSpeedBuf[3] = {};

			Euler2Angular(nowSlerpEuler, eulerVelocity, postureSpeedBuf);

			postureSpeed[0] = -postureSpeedBuf[1];//1	//roll
			postureSpeed[1] = -postureSpeedBuf[0];//0	//yaw
			postureSpeed[2] = -postureSpeedBuf[2];//2	//pitch

			//DebagComment(" posture speed ");	DisplayVector(3, postureSpeed);
		}
		else if (currentTime >= kine::TIME_LENGTH) for (int i = 0; i < 3; ++i) {
			postureSpeed[i] = 0.0;
		}
	}

	//DebagComment("posture speed");	DisplayVector(3, postureSpeed);

	//DebagComment("calc velocity posture : finished\n");
}

void CalcViaPos(TarPoints targetPoints, double via2endLength, double *returnPos3) {
	std::vector<double> graspV;
	std::vector<double> modifyV;

	graspV = targetPoints.graspDirection();

	modifyV = mulVector(graspV, via2endLength);

	for (int i = 0; i < 3; ++i) {
		returnPos3[i] = targetPoints.mid[i] - modifyV[i];
	}
}
