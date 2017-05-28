#pragma once
// 2017/04/28
// created by eiki obara

#ifndef __MY_CONFIG_H__
#define __MY_CONFIG_H__

namespace kine {
	//通過点の設定値(初期位置と最終位置を含む点の数)
	const int ROUTE_POINTS = 3;

	//通過するルートのリンク数(通過点をつなぐ線のこと)
	const int ROUTE_LINK = ROUTE_POINTS - 1;

	//自由度の設定
	const int MAXJOINT = 8; //7 axis and hand tip DoF

	//速度計算1ループごとに進む時間
	const double TIME_SPAN = 0.01;

	//台形補間の計算総時間
	const double TIME_LENGTH = 2.0;

	//台形補間の加減速時間
	const double ACCEL_TIME = TIME_LENGTH / 4;

	//手先位置収束しきい値
	const double FINGER_THRESHOLD = 0.001;

	//リンク長さ
	//肩つけ根(base)
	const double B_ARM_LENGTH = 0.164;
	//上腕(upper)
	const double U_ARM_LENGTH = 0.322;
	//前腕(forward)
	const double F_ARM_LENGTH = 0.257;
	//手先(hand)
	//const double H_ARM_LENGTH = 0.157;	//light simulator
	const double H_ARM_LENGTH = 0.135;	//real simulator

	//手首からカメラ設置位置の距離
	const double WRIST2CAMERA_LENGTH = 0.0405;	

	const double CAMERA_POSITION = 0.10;	//カメラ設置位置からカメラまでの距離

	const double CAMERA_OFFSET = -0.006;	//手先とカメラの中心までのズレ



	//スペースマウスの値強度
	//const double SPACEMOUSE_TRANSLATION_GAIN = 0.0f;
	//const double SPACEMOUSE_ROTATION_GAIN = 0.0005f;

	//NULL分のメモリ確保用
	const int NR_END = 1;

	//数値の比較用定数
	const double COMPARE_ZERO = 1.0e-6;
}
#endif // !__MY_CONFIG_H__
