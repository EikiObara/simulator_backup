#include <stdio.h>
#include <iostream>
#include <stddef.h>
#include <stdlib.h>
#include <iomanip>
#include <time.h>
#include "include\kinematics.h"
#include "include\lu.h"
#include "include\kinemaDebag.h"

static double B_ARM_LENGTH = 0.164;
static double U_ARM_LENGTH = 0.322;
static double F_ARM_LENGTH = 0.257;
static double H_ARM_LENGTH = 0.157;	//old hand
//static double H_ARM_LENGTH = 0.149;	//new hand

//デバッグ用横線
void Dbar() {
	printf("-----------------------------------\n");
}

//プロトタイプ宣言
static void InitOM(double *currentRadian, Matrix *OM);
static void CalcJacob(Matrix *HTMMat, Matrix *Jacob, int selfmotion);
static int PIM(Matrix *mat, Matrix *PIMat);

//ludcmp用の一時的な計算用変数
double *vv = SetVector(MAXJOINT);

//行列用構造体のメモリ確保
Matrix *CreateDiMatrix(const int row, const int column){
	 Matrix *temp;

	 temp = new Matrix;
	 temp->m = new double[row*column];

	 if (!temp->m) {
		 Ercom("***Create Matrix Error***");
	 }

	 temp->row = row;
	 temp->column = column;

	 return temp;
}

Matrix *CreateTriMatrix(const int row, const int column, const int numMat) {
	Matrix *temp;

	temp = new Matrix;
	temp->m = new double[row*column*numMat];

	if (!temp->m) {
		Ercom("***Create Matrix Error***");
	}

	temp->row = row;
	temp->column = column;

	return temp;
}

//行列用構造体のメモリ解放
void FreeMatrix(Matrix* matrix) {
	delete[] matrix;
}

double *SetVector(int nl) {
	//cout << "vector malloc started" << endl;
	double *v = new double[nl + NR_END];

	if (!v) {
		std::cout << "malloc failure in vector()" << std::endl;
	}

	return v;
}

void FreeVector(double *v) {
	//cout << "free_vector started" << endl;
	delete[] v;
}

static int InverseMatrix(Matrix *OriginMat, Matrix *InverseMat) {
	//std::cout << "InverseMatrix started\n";
	double d, col[MAXJOINT] = {};
	int indx[MAXJOINT + NR_END] = {};
	int check = 0;

	check = Ludcmp(OriginMat->m, OriginMat->row, indx, &d);

	if (check > 0) {
		return check;
	}

	for (int j = 0; j < OriginMat->row; j++) {
		for (int i = 0; i < OriginMat->row; i++)col[i] = 0.0;
		col[j] = 1.0;
		Lubksb(OriginMat->m, OriginMat->row, indx, col);
		for (int i = 0; i < OriginMat->row; i++)
		{
			InverseMat->m[OriginMat->row * i + j] = col[i];
		}
	}
	return check;
}

static void InitOM(double *inRad, Matrix *OM) {
	//cout << "OMInit started" << endl;

	int row, jnt;
	double theta[MAXJOINT];
	double alpha[MAXJOINT];
	double alength[MAXJOINT];
	double dlength[MAXJOINT];

	/*Dbar();　for (row = 0; row < MAXJOINT; row++) {	cout << inRad[row] << endl;	}*/

	//a(i-1) リンク間の距離
	for (row = 0; row < MAXJOINT; row++) { alength[row] = 0; }

	//alpha(i-1) 関節のねじりの位置
	alpha[0] = 0;		alpha[1] = -M_PI / 2;
	alpha[2] = M_PI / 2;	alpha[3] = -M_PI / 2;
	alpha[4] = M_PI / 2;	alpha[5] = -M_PI / 2;
	alpha[6] = M_PI / 2;	alpha[7] = 0;

	//d(i) リンク長さ
	dlength[0] = 0;				dlength[1] = 0;
	dlength[2] = U_ARM_LENGTH;	dlength[3] = 0;
	dlength[4] = F_ARM_LENGTH;	dlength[5] = 0;
	dlength[6] = 0;				dlength[7] = H_ARM_LENGTH;

	//theta(i) 関節角度
	theta[0] = inRad[0];	theta[1] = inRad[1];
	theta[2] = inRad[2];	theta[3] = inRad[3];
	theta[4] = inRad[4];	theta[5] = inRad[5];
	theta[6] = inRad[6];

	//八番目の関節はないので
	theta[7] = 0.0;

	double cos_t,sin_t,cos_a,sin_a;

	for (jnt = 0; jnt < MAXJOINT; jnt++) {
		cos_t = cos(theta[jnt]);
		sin_t = sin(theta[jnt]);
		cos_a = cos(alpha[jnt]);
		sin_a = sin(alpha[jnt]);
		
		OM->m[OM->column * OM->row * jnt + 4 * 0 + 0] = cos_t;
		OM->m[OM->column * OM->row * jnt + 4 * 0 + 1] = -sin_t;
		OM->m[OM->column * OM->row * jnt + 4 * 0 + 2] = 0.0;
		OM->m[OM->column * OM->row * jnt + 4 * 0 + 3] = alength[jnt];

		OM->m[OM->column * OM->row * jnt + 4 * 1 + 0] = cos_a*sin_t;
		OM->m[OM->column * OM->row * jnt + 4 * 1 + 1] = cos_a*cos_t;
		OM->m[OM->column * OM->row * jnt + 4 * 1 + 2] = -sin_a;
		OM->m[OM->column * OM->row * jnt + 4 * 1 + 3] = -sin_a*dlength[jnt];

		OM->m[OM->column * OM->row * jnt + 4 * 2 + 0] = sin_a*sin_t;
		OM->m[OM->column * OM->row * jnt + 4 * 2 + 1] = sin_a*cos_t;
		OM->m[OM->column * OM->row * jnt + 4 * 2 + 2] = cos_a;
		OM->m[OM->column * OM->row * jnt + 4 * 2 + 3] = cos_a*dlength[jnt];

		OM->m[OM->column * OM->row * jnt + 4 * 3 + 0] = 0;
		OM->m[OM->column * OM->row * jnt + 4 * 3 + 1] = 0;
		OM->m[OM->column * OM->row * jnt + 4 * 3 + 2] = 0;
		OM->m[OM->column * OM->row * jnt + 4 * 3 + 3] = 1;
	}

	//cos()にｔる浮動小数点誤差を０にする
	for (row = 0; row < (MAXJOINT) * OM->column * OM->column; row++) { if (fabs(OM->m[row]) < COMPARE_ZERO) { OM->m[row] = 0; } }

	//-0を0にする
	for (row = 0; row < (MAXJOINT) * OM->column * OM->column; row++) {
		if (OM->m[row] == -0) { OM->m[row] = 0; }
	}

	//debag
	//Dbar();
	//DisplayTriMatrix(OM, MAXJOINT);
}

void CalcHTM(Matrix *OM, Matrix *HTMMat) {
	//cout << "HTMCalc started" << endl;
	Matrix *tempMat;
	tempMat = CreateDiMatrix(4, 4);
	Matrix *sum;
	sum = CreateDiMatrix(4, 4);

	//cout <<"HTM Function Started" <<endl;
	//cout <<"initializing tempMatrix" <<endl;

	for (int row = 0; row < tempMat->row; row++) {
		for (int column = 0; column < tempMat->column; column++) {
			if (row == column) {
				tempMat->m[4 * row + column] = 1.0;
			}
			else {
				tempMat->m[4 * row + column] = 0.0;
			}
		}
	}

	//同次変換行列の計算（心臓部）//////////////
	//cout << "HTM calculation Started" << endl;

	for (int joint = 0; joint < MAXJOINT; joint++) {
		for (int row = 0; row < sum->row; row++) {
			for (int column = 0; column < sum->column; column++) {
				sum->m[4 * row + column] =
					tempMat->m[tempMat->column * row + 0] * OM->m[OM->row * OM->column * joint + OM->column * 0 + column]
					+ tempMat->m[tempMat->column * row + 1] * OM->m[OM->row * OM->column * joint + OM->column * 1 + column]
					+ tempMat->m[tempMat->column * row + 2] * OM->m[OM->row * OM->column * joint + OM->column * 2 + column]
					+ tempMat->m[tempMat->column * row + 3] * OM->m[OM->row * OM->column * joint + OM->column * 3 + column];
			}
		}

		for (int row = 0; row < sum->row; row++) {
			for (int column = 0; column < sum->column; column++) {
				tempMat->m[tempMat->column * row + column] = sum->m[sum->column * row + column];
				HTMMat->m[HTMMat->row * HTMMat->column * joint + HTMMat->column * row + column] = sum->m[sum->column * row + column];
			}
		}
	}
	////////////(心臓部)////////////////////////
	FreeMatrix(tempMat);
	FreeMatrix(sum);
}

static void CalcJacob(Matrix *HTMMat, Matrix *Jacob, int selfmotion) {
	//cout << "JacobCalc started" << endl;
	int row,joint;
	double ArmPosition[3] = {};
	double PosiVector[3] = {};

	for (row = 0; row < 3; row++) {
		ArmPosition[row] = HTMMat->m[HTMMat->row * (MAXJOINT-1) + 4 * row + 3];
		//cout << ArmPosition[i] <<endl;
	}

	for (joint = 0; joint < (MAXJOINT-1); joint++) {

		//0PE,n -> x
		PosiVector[0] = ArmPosition[0] - HTMMat->m[HTMMat->column * HTMMat->row * joint + 4 * 0 + 3];
		PosiVector[1] = ArmPosition[1] - HTMMat->m[HTMMat->column * HTMMat->row * joint + 4 * 1 + 3];
		PosiVector[2] = ArmPosition[2] - HTMMat->m[HTMMat->column * HTMMat->row * joint + 4 * 2 + 3];

		//Jacobianの外積計算
		//J(0,joint)
		Jacob->m[Jacob->column * 0 + joint]
			= HTMMat->m[HTMMat->column * HTMMat->row * joint + 4 * 1 + 2] * PosiVector[2]
			- HTMMat->m[HTMMat->column * HTMMat->row * joint + 4 * 2 + 2] * PosiVector[1];
		//J(1,joint)
		Jacob->m[Jacob->column * 1 + joint]
			= HTMMat->m[HTMMat->column * HTMMat->row * joint + 4 * 2 + 2] * PosiVector[0]
			- HTMMat->m[HTMMat->column * HTMMat->row * joint + 4 * 0 + 2] * PosiVector[2];
		//J(2,joint)
		Jacob->m[Jacob->column * 2 + joint]
			= HTMMat->m[HTMMat->column * HTMMat->row * joint + 4 * 0 + 2] * PosiVector[1]
			- HTMMat->m[HTMMat->column * HTMMat->row * joint + 4 * 1 + 2] * PosiVector[0];
		//J(3,joint)
		Jacob->m[Jacob->column * 3 + joint] = HTMMat->m[HTMMat->column * HTMMat->row * joint + 4 * 0 + 2];
		//J(4,joint)
		Jacob->m[Jacob->column * 4 + joint] = HTMMat->m[HTMMat->column * HTMMat->row * joint + 4 * 1 + 2];
		//J(5,joint)
		Jacob->m[Jacob->column * 5 + joint] = HTMMat->m[HTMMat->column * HTMMat->row * joint + 4 * 2 + 2];
		
		if (selfmotion) {
			//J(6,joint)
			Jacob->m[Jacob->column * 6 + joint] = 0.0;
			if (joint == 2) {	//ひじ関節が第三関節だから(プログラムでは0から連番される)
				Jacob->m[Jacob->column * 6 + joint] = 1;
			}
		}
	}
}

//pseudo inverse matrixを計算するプログラム
static int PIM(Matrix *mat, Matrix *PIMat) {
	Matrix *TMat;
	TMat = CreateDiMatrix(mat->column, mat->row);
	Matrix *sqMat;
	sqMat = CreateDiMatrix(mat->row, mat->row);
	Matrix *detMat;
	detMat = CreateDiMatrix(mat->row, mat->row);
	Matrix *IsqMat;
	IsqMat = CreateDiMatrix(mat->row, mat->row);

	double col[MAXJOINT + NR_END] = {};
	double d = 0;
	int index[MAXJOINT + NR_END] = {};
	int row, column, check=0;

	//cout << "PIM started" << endl;

	//Dbar();	std::cout << "original matrix\n";	displayDiMatrix(mat);

	//転置行列を作る(A^T)
	for (row = 0; row < mat->row; row++) {
		for (column = 0; column < mat->column; column++) {
			TMat->m[mat->row * column + row] = mat->m[mat->column * row + column];
		}
	}
	/////////////////////転置行列作った

	//Dbar();	std::cout << "Transform Matrix\n";	displayDiMatrix(TMat);

	//mat*TMatの計算をする(A･A^T)
	for (row = 0; row < mat->row; row++) {
		for (column = 0; column < mat->column; column++) {
			sqMat->m[mat->row * row + column]
				= mat->m[mat->column * row + 0] * TMat->m[mat->row * 0 + column]		//ここはなぜかfor(int k = 0; k < mat->column; k++)
				+ mat->m[mat->column * row + 1] * TMat->m[mat->row * 1 + column]		//のように省略すると
				+ mat->m[mat->column * row + 2] * TMat->m[mat->row * 2 + column]		//エラーは出ないけれども
				+ mat->m[mat->column * row + 3] * TMat->m[mat->row * 3 + column]		//値が狂ってしまうから
				+ mat->m[mat->column * row + 4] * TMat->m[mat->row * 4 + column]		//このままにして置かなければいけない
				+ mat->m[mat->column * row + 5] * TMat->m[mat->row * 5 + column]
				+ mat->m[mat->column * row + 6] * TMat->m[mat->row * 6 + column];
			detMat->m[mat->row * row + column] = sqMat->m[mat->row * row + column];
		}
	}

	////////////////////////mat*Tmatのおわり
	
	//Dbar();	std::cout << "[Original * Transform] matrix\n";	displayDiMatrix(sqMat);
		
	//行列式による判定
	check = Ludcmp(detMat->m, mat->row, index, &d);
	for (int i = 0; i < mat->row; i++) {	d *= detMat->m[mat->row * i + i];}
	if (d == 0) {
		Dbar();		printf(".....determinant = 0.....\n.....Can not calculate.....\n");	Dbar();
		return 1;
	}

	//正方逆行列を作る((A･A^T)^-1)
	//cout << "Creating Square Inverse Matrix started" << endl;
	check = Ludcmp(sqMat->m, mat->row, index, &d);

	if (check > 0) {
		return check;
	}

	for (int j = 0; j < mat->row; j++) {
		for (int i = 0; i < mat->row; i++)col[i] = 0.0;
		col[j] = 1.0;
		Lubksb(sqMat->m, mat->row, index, col);
		for (int i = 0; i < mat->row; i++){
			IsqMat->m[mat->row * i + j] = col[i];
		}
	}

	if (check > 0) {
		printf(".....LU function error.....\n.....Can not calculate.....\n");
		return 1;
	}

	//cout << "Creating Square Inverse Matrix finished" << endl;
	//Dbar();	std::cout << "square inverse matrix\n";	DisplayDiMatrix(IsqMat);

	for (row = 0; row < PIMat->row; row++) {
		for (column = 0; column < PIMat->column; column++) {
			PIMat->m[PIMat->column * row + column]
				= TMat->m[PIMat->column * row + 0] * IsqMat->m[PIMat->column * 0 + column]
				+ TMat->m[PIMat->column * row + 1] * IsqMat->m[PIMat->column * 1 + column]
				+ TMat->m[PIMat->column * row + 2] * IsqMat->m[PIMat->column * 2 + column]
				+ TMat->m[PIMat->column * row + 3] * IsqMat->m[PIMat->column * 3 + column]
				+ TMat->m[PIMat->column * row + 4] * IsqMat->m[PIMat->column * 4 + column]
				+ TMat->m[PIMat->column * row + 5] * IsqMat->m[PIMat->column * 5 + column];
		}
	}

	//Dbar();	std::cout << "Pseudo inverse matrix\n";	displayDiMatrix(PIMat);

	for (row = 0; row < mat->row * mat->column; row++) {
		if (fabs(PIMat->m[row]) < COMPARE_ZERO) {
			PIMat->m[row] = 0.0;
		}
	}

	//cout << "PIM finished" << endl;

	FreeMatrix(TMat);
	FreeMatrix(sqMat);
	FreeMatrix(detMat);
	FreeMatrix(IsqMat);

	return check;
}

//class HandP の定義
kinematics::kinematics(){
	eX = 0;	eY = 0; eZ = 0;
	wX = 0;	wY = 0; wZ = 0;
	X = 0;	Y = 0; Z = 0;
}

kinematics::~kinematics() {
}


//メンバ変数の出力
void kinematics::DisplayCoordinate() {
	printf("elbow(x,y,z)\t->\t(%lf\t%lf\t%lf)\n", eX, eZ);
	printf("wrists(x,y,z)\t->\t(%lf\t%lf\t%lf)\n", wX, wY, wZ);
	printf("fingers(x,y,z)\t->\t(%lf\t%lf\t%lf)\n", X, Y, Z);
}

//肘座標を取得
void kinematics::GetElbowCoordinate(double *ec) {
	ec[0] = eX;
	ec[1] = eY;
	ec[2] = eZ;
}

//手首座標を取得
void kinematics::GetwristCoordinate(double *wc) {
	wc[0] = wX;
	wc[1] = wY;
	wc[2] = wZ;
}

//手先座標を取得
void kinematics::GetCoordinate(double *c) {
	c[0] = X;
	c[1] = Y;
	c[2] = Z;
}

//順運動学計算
void kinematics::CalcFK(double *currentRadian, double *currentCoordinate) {
	//std::cout << "FKkinematicsosition started\n";

	Matrix *om;
	om = CreateTriMatrix(4, 4, MAXJOINT);
	Matrix *htm;
	htm = CreateTriMatrix(4, 4, MAXJOINT);

	//(1)同次変換行列を作る
	InitOM(currentRadian, om);
	//displayTriMatrix(om);

	//(2)同次変換行列を作る
	CalcHTM(om, htm);
	//displayTriMatrix(htm);

	//(3)同次変換行列の手先位置に関する情報を書き出す．
	
	//elbow
	eX = *(htm->m + (htm->column * htm->row * 2 + 4 * 0 + 3));
	eY = *(htm->m + (htm->column * htm->row * 2 + 4 * 1 + 3));
	eZ = *(htm->m + (htm->column * htm->row * 2 + 4 * 2 + 3));

	//wrist
	wX = *(htm->m + (htm->column * htm->row * 4 + 4 * 0 + 3));
	wY = *(htm->m + (htm->column * htm->row * 4 + 4 * 1 + 3));
	wZ = *(htm->m + (htm->column * htm->row * 4 + 4 * 2 + 3));

	//finger
	currentCoordinate[0] = X = *(htm->m + (htm->column * htm->row * 7 + 4 * 0 + 3));
	currentCoordinate[1] = Y = *(htm->m + (htm->column * htm->row * 7 + 4 * 1 + 3));
	currentCoordinate[2] = Z = *(htm->m + (htm->column * htm->row * 7 + 4 * 2 + 3));
	
	FreeMatrix(om);
	FreeMatrix(htm);
}

//calc inverse kinematics
int kinematics::CalcIK(double *currentRadian, double *handVelocity, double *nextRadVerocity, bool selfMotion) {
	//std::cout << "self motion calculation started\n";
	Matrix *om_m;	//original matrix
	om_m = CreateTriMatrix(4,4, MAXJOINT);
	Matrix *htm_m;	//homogeneous translate matrix
	htm_m = CreateTriMatrix(4, 4, MAXJOINT);
	Matrix *jacob_m;	//jacobian
	jacob_m = CreateDiMatrix(7, 7);
	Matrix *iJacob_m;	//inverse jacobian
	iJacob_m = CreateDiMatrix(7, 7);
	int check = 0;

	//同次変換行列の作成
	InitOM(currentRadian, om_m);
	//Dbar();	std::cout << "OM\n";	displayTriMatrix(OM->m, 4, 4);

	//同次変換行列の積の計算
	CalcHTM(om_m, htm_m);
	//Dbar();	std::cout << "HTM\n";	displayTriMatrix(HTM->m, 4, 4);

	//ヤコビ行列の計算
	CalcJacob(htm_m, jacob_m, selfMotion);
	//Dbar();	std::cout << "Jacob\n";	displayDiMatrix(Jacob->m, Jacob->row, Jacob->column);

	//擬似逆行列を作成
	InverseMatrix(jacob_m, iJacob_m);
	//Dbar();	std::cout << "Inverse Jacob\n";	displayDiMatrix(IJacob->m, 7, 7);

	for (int i = 0; i < (MAXJOINT - 1); i++) {
		nextRadVerocity[i] =
			iJacob_m->m[7 * i + 0] * handVelocity[0]
			+ iJacob_m->m[7 * i + 1] * handVelocity[1]
			+ iJacob_m->m[7 * i + 2] * handVelocity[2]
			+ iJacob_m->m[7 * i + 3] * handVelocity[3]
			+ iJacob_m->m[7 * i + 4] * handVelocity[4]
			+ iJacob_m->m[7 * i + 5] * handVelocity[5]
			+ iJacob_m->m[7 * i + 6] * handVelocity[6];
		//if (fabs(outRadVelocity[i]) < COMPARE_ZERO) {
		//	outRadVelocity[i] = 0.0;
		//}
	}
	//Dbar();

	FreeMatrix(om_m);
	FreeMatrix(htm_m);
	FreeMatrix(jacob_m);
	FreeMatrix(iJacob_m);

	return check;
}

void kinematics::VelocityRegulation(const double *targetCoordinate, double *velocity, double *currentTime) {
	int i;
	static double p2pLength[3];

	//さくらんぼの茎に対して直接アプローチするとぶつかるので，茎の手前に一度移動し，把持面と平行にアプローチする．
	//通過点(茎手前)
	if (*currentTime < TIME_SPAN) {
		p2pLength[0] = targetCoordinate[0] - X;
		p2pLength[1] = targetCoordinate[1] - Y;
		p2pLength[2] = targetCoordinate[2] - Z;
	}

	/////////////////////////////////
	//台形補間の速度計算
	if (*currentTime < ACCEL_TIME) {
		for (i = 0; i < 3; i++) {
			velocity[i] = TIME_SPAN * p2pLength[i] * (*currentTime) / (ACCEL_TIME * (TIME_LENGTH - ACCEL_TIME));
		}
	}
	else if (*currentTime >= ACCEL_TIME && *currentTime <= (TIME_LENGTH - ACCEL_TIME)) {
		for (i = 0; i < 3; i++) {
			velocity[i] = TIME_SPAN * p2pLength[i] / (TIME_LENGTH - ACCEL_TIME);
		}
	}
	else if (*currentTime >(TIME_LENGTH - ACCEL_TIME) && *currentTime <= TIME_LENGTH) {
		for (i = 0; i < 3; i++) {
			velocity[i] = TIME_SPAN * p2pLength[i] * (TIME_LENGTH - (*currentTime)) / (ACCEL_TIME * (TIME_LENGTH - ACCEL_TIME));
		}
	}
	else if (*currentTime > TIME_LENGTH) {
		if (fabs(X - targetCoordinate[0]) < FINGER_THRESHOLD) {
			if (fabs(Y - targetCoordinate[1]) < FINGER_THRESHOLD) {
				if (fabs(Z - targetCoordinate[2]) < FINGER_THRESHOLD) {
					velocity[0] = velocity[1] = velocity[2] = 0.0;
				}
			}
		}
		else{
			velocity[0] = TIME_SPAN * (targetCoordinate[0] - X);
			velocity[1] = TIME_SPAN * (targetCoordinate[1] - Y);
			velocity[2] = TIME_SPAN * (targetCoordinate[2] - Z);
		}
	}

	//printf(" time Counter-> %lf",&CurrentTime);
	*currentTime += TIME_SPAN;
}
//jointvelocity = JV
/*
//擬似逆行列で動かすやつ，いらなかったくそっぉぉぉぉぉおぉぉぉぉぉぉ！！！
int MainCalc(double *inRadian, double *speed, double *outRadVelocity) {
	//std::cout << "mainCalc" << std::endl;

	Matrix *OM;
	OM = Create_Matrix(16, MAXJOINT);
	Matrix *HTM;
	HTM = Create_Matrix(16, MAXJOINT);
	Matrix *Jacob;
	Jacob = Create_Matrix(6, 7);
	Matrix *IJacob;
	IJacob= Create_Matrix(7, 6);

	int check = 0;
	int selfmotion = 0;

	//関数外からの入力値の表示
	//入力角度
	Dbar();	
	for (int i = 0; i < MAXJOINT; i++) {
		printf("%d inrad-> %3.3lf", i, inRadian[i]);
		printf("/speed->%3.3lf\n", speed[i]);
	}
	//同次変換行列の作成
	OMInit(inRadian, OM);

	//同次変換行列の表示
	//Dbar();	std::cout << "OM\n";	displayTriMatrix(OM->m, 4, 4);

	//同次変換行列の積の計算
	HTMCalc(OM, HTM);

	//同次変換行列の表示
	//Dbar();	std::cout << "HTM\n";	displayTriMatrix(HTM->m, 4, 4);

	//ヤコビ行列の計算
	JacobCalc(HTM, Jacob, selfmotion);

	//ヤコビ行列の表示
	//Dbar();	std::cout << "Jacob\n";	displayDiMatrix(Jacob->m, Jacob->row, Jacob->column);

	//擬似逆行列を作成
	check = PIM(Jacob, IJacob);

	//擬似逆行列の表示
	//Dbar();	std::cout << "Inverse Jacob\n";	displayDiMatrix(IJacob->m, IJacob->row, IJacob->column);

	//擬似逆行列が本当に正しいか元の行列と掛けあわせて確認(単位正方行列が出ればOK)
	//Dbar();	std::cout << "PIMConfirmation\n";	PIMConfirmation(Jacob,IJacob);

	for (int i = 0; i < (MAXJOINT - 1); i++) {
		outRadVelocity[i] =
			IJacob->m[6 * i + 0] * speed[0]
			+ IJacob->m[6 * i + 1] * speed[1]
			+ IJacob->m[6 * i + 2] * speed[2]
			+ IJacob->m[6 * i + 3] * speed[3]
			+ IJacob->m[6 * i + 4] * speed[4]
			+ IJacob->m[6 * i + 5] * speed[5];
		//if (fabs(outRadVelocity[i]) < COMPARE_ZERO) {
		//	outRadVelocity[i] = 0.0;
		//}
		//std::cout << outRadVelocity[i] << std::endl;
	}
	//Dbar();
	//std::cout << "mainCalc finished\n";
	Free_Matrix(OM);
	Free_Matrix(HTM);
	Free_Matrix(Jacob);
	Free_Matrix(IJacob);

	return check;
}
*/
