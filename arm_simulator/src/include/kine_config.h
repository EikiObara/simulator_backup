#pragma once
// 2017/04/28
// created by eiki obara

//もし初期姿勢や搬送ユニットの座標が間違っているならば，
// src/kine_trajectory.cpp の selectTarget 関数を変更してください．


#ifndef __MY_CONFIG_H__
#define __MY_CONFIG_H__

namespace kine {
	enum targetType {
		CALIB_IN	= 0,	//７．キャリブレーション治具上方10cmへ
		CALIB_OUT	= 1,	//１．キャリブレーション冶具から上方10㎝
		CALIB_RIGHT = 2,
		INIT_POS	= 3,	//２．キャリブレーション治具上方10cmから初期姿勢
		PICK_POS	= 4,	//３．初期姿勢から把持姿勢
		PICKING		= 5,	//４．把持姿勢から摘み取り動作を行う
		CONVEY		= 6,	//５．摘み取り動作後収穫搬送機構へ
		KEYBOARD	= 7		//６．キーボードから受取
	};

	//通過点の設定値(初期位置と最終位置を含む点の数)
	const int ROUTE_POINTS = 3;

	//通過するルートのリンク数(通過点をつなぐ線のこと)
	const int ROUTE_LINK = ROUTE_POINTS - 1;

	//自由度の設定
	const int MAXJOINT = 8; //7 axis and hand tip DoF

	//速度計算1ループごとに進む時間
	const double TIME_SPAN = 0.001;

	//総移動時間
	const double TIME_LENGTH = 1.0;

	//指先軌道の計算総時間
	const double POSITION_CHANGE_TIME = TIME_LENGTH;

	//手先姿勢の計算総時間
	const double POSTURE_CHANGE_TIME = TIME_LENGTH * 1;

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
	const double H_ARM_LENGTH = 0.1407;	//real simulator

	//経由点を目標点の半径いくらに設定するか．単位はメートル
	const double VIA_LENGTH = 0.08;

	//スペースマウスの値強度
	//const double SPACEMOUSE_TRANSLATION_GAIN = 0.0f;
	//const double SPACEMOUSE_ROTATION_GAIN = 0.0005f;

	//NULL分のメモリ確保用
	const int NR_END = 1;

	//数値の比較用定数
	const double COMPARE_ZERO = 1.0e-6;
}
#endif // !__MY_CONFIG_H__