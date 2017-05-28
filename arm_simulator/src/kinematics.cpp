// 2017/04/28
// created by eiki obara

#define _USE_MATH_DEFINES

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "include\kine_debag.h"
#include "include\kinematics.h"
#include "include\kine_target_point.h"
#include "include\kine_matrix.h"
#include "include\kine_quat.h"
#include "include\kine_vector.h"
#include "include\trapezoidal_interpolation.h"
#include "include\kine_spline.h"

#include "include\kine_config.h"
#include "include\kine_convertor.h"

namespace kine {

	void InitOM(double *inRad, Matrix &OM) {
		//std::cout << "OMInit started" << std::endl;

		int row, jnt;
		double theta[MAXJOINT];
		double alpha[MAXJOINT];
		double alength[MAXJOINT];
		double dlength[MAXJOINT];

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

		//八番目の関節はないので決め打ちで
		theta[7] = 0.0;

		double cos_t, sin_t, cos_a, sin_a;

		for (jnt = 0; jnt < MAXJOINT; jnt++) {
			cos_t = cos(theta[jnt]);
			sin_t = sin(theta[jnt]);
			cos_a = cos(alpha[jnt]);
			sin_a = sin(alpha[jnt]);

			OM.Mat3D(jnt, 0, 0, cos_t);
			OM.Mat3D(jnt, 0, 1, -sin_t);
			OM.Mat3D(jnt, 0, 2, 0.0);
			OM.Mat3D(jnt, 0, 3, alength[jnt]);

			OM.Mat3D(jnt, 1, 0, cos_a*sin_t);
			OM.Mat3D(jnt, 1, 1, cos_a*cos_t);
			OM.Mat3D(jnt, 1, 2, -sin_a);
			OM.Mat3D(jnt, 1, 3, -sin_a*dlength[jnt]);

			OM.Mat3D(jnt, 2, 0, sin_a*sin_t);
			OM.Mat3D(jnt, 2, 1, sin_a*cos_t);
			OM.Mat3D(jnt, 2, 2, cos_a);
			OM.Mat3D(jnt, 2, 3, cos_a*dlength[jnt]);

			OM.Mat3D(jnt, 3, 0, 0);
			OM.Mat3D(jnt, 3, 1, 0);
			OM.Mat3D(jnt, 3, 2, 0);
			OM.Mat3D(jnt, 3, 3, 1);

			/*
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
			*/
		}

		for (int i = 0; i < MAXJOINT; ++i) {
			for (int j = 0; j < OM.Row(); ++j) {
				for (int k = 0; k < OM.Column(); ++k) {
					//cos()による浮動小数点誤差を０にする
					if (fabs(OM.Mat3D(i, j, k)) < COMPARE_ZERO) {
						OM.Mat3D(i, j, k, 0.0);
					}
				}
			}
		}

		//cos()による浮動小数点誤差を０にする
		//for (row = 0; row < (MAXJOINT) * OM->column * OM->row; row++) { if (fabs(OM->m[row]) < COMPARE_ZERO) { OM->m[row] = 0; } }

		//-0を0にする
		//for (row = 0; row < (MAXJOINT) * OM->column * OM->row; row++) {	if (OM->m[row] == -0) { OM->m[row] = 0; }}

		//debag
		//OM.Display();
	}

	/*
	void InitOM(double *inRad, std::vector<std::vector<std::vector<double>>> &OM) {
		//cout << "OMInit started" << endl;

		int row, jnt;
		double theta[MAXJOINT];
		double alpha[MAXJOINT];
		double alength[MAXJOINT];
		double dlength[MAXJOINT];


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

		//八番目の関節はないので決め打ちで
		theta[7] = 0.0;

		double cos_t, sin_t, cos_a, sin_a;

		for (jnt = 0; jnt < MAXJOINT; jnt++) {
			cos_t = cos(theta[jnt]);
			sin_t = sin(theta[jnt]);
			cos_a = cos(alpha[jnt]);
			sin_a = sin(alpha[jnt]);

			OM[jnt][0][0] = cos_t;
			OM[jnt][0][1] = -sin_t;
			OM[jnt][0][2] = 0.0;
			OM[jnt][0][3] = alength[jnt];

			OM[jnt][1][0] = cos_a*sin_t;
			OM[jnt][1][1] = cos_a*cos_t;
			OM[jnt][1][2] = -sin_a;
			OM[jnt][1][3] = -sin_a*dlength[jnt];

			OM[jnt][2][0] = sin_a*sin_t;
			OM[jnt][2][1] = sin_a*cos_t;
			OM[jnt][2][2] = cos_a;
			OM[jnt][2][3] = cos_a*dlength[jnt];

			OM[jnt][3][0] = 0;
			OM[jnt][3][1] = 0;
			OM[jnt][3][2] = 0;
			OM[jnt][3][3] = 1;

			//for (row = 0; row < OM.size(); row++) { if (fabs(OM[jnt][i][j]) < COMPARE_ZERO) { OM[jnt][i][j] = 0; } }
		}

		//cos()にｔる浮動小数点誤差を０にする

		//-0を0にする
		//for (row = 0; row < (MAXJOINT)* OM->column * OM->row; row++) { if (OM->m[row] == -0) { OM->m[row] = 0; } }

		//debag
		//DisplayTriMatrix(OM, 4, 4);
	}
	*/

	void CalcHTM(Matrix OM, Matrix &HTM) {
		//cout << "HTMCalc started" << endl;
		int jnt, row, column;

		Matrix temp;
		Matrix sum;

		temp.CreateDiMatrix(4, 4);
		sum.CreateDiMatrix(4, 4);

		//DebagComment("OM");
		//OM.Display();

		//DebagComment("HTM");
		//HTM.Display();

		//cout <<"HTM Function Started" <<endl;
		//cout <<"initializing temprix" <<endl;

		for (row = 0; row < temp.Row(); ++row) {
			for (column = 0; column < temp.Column(); ++column) {
				if (row == column) {
					temp.Mat2D(row, column, 1.0);
				}
				else {
					temp.Mat2D(row, column, 0.0);
				}
			}
		}

		//temp.Display();

		//同次変換行列の計算（心臓部）//////////////

		for (jnt = 0; jnt < MAXJOINT; ++jnt) {
			for (row = 0; row < sum.Row(); ++row) {
				for (column = 0; column < sum.Column(); ++column) {
					double buf = 0.0;
					for (int k = 0; k < sum.Column(); ++k) {
						buf += temp.Mat2D(row, k) * OM.Mat3D(jnt, k, column);
					}

					sum.Mat2D(row, column, buf);

					if (fabs(sum.Mat2D(row, column)) < COMPARE_ZERO) {
						sum.Mat2D(row, column, 0.0);
					}
				}
			}

			//sum.Display();

			for (row = 0; row < sum.Row(); ++row) {
				for (column = 0; column < sum.Column(); ++column) {
					temp.Mat2D(row, column, sum.Mat2D(row, column));
					HTM.Mat3D(jnt, row, column, sum.Mat2D(row, column));
				}
			}
		}

		//HTM.Display();

		//負のゼロを正のゼロに変換
		for (int i = 0; i < HTM.MatSize(); ++i) {
			for (int j = 0; j < HTM.Row(); ++j) {
				for (int k = 0; k < HTM.Column(); ++k) {
					if (HTM.Mat3D(i, j, k) == (0.0)) {
						//printf("%d, %d, %d, ", i, j, k);
						//printf("%lf \t", HTM.Mat3D(i, j, k));
						HTM.Mat3D(i, j, k, 0.0);
						//printf("-> %lf \n", HTM.Mat3D(i, j, k));
					}
				}
			}
		}
		//HTM.Display();

		////////////(心臓部)////////////////////////
		temp.~Matrix();
		sum.~Matrix();
	}

	void CalcJacob(Matrix HTM, int selfmotion, Matrix &JACOB) {
		//cout << "JacobCalc started" << endl;
		int row, jnt;
		double ArmPosition[3] = {};
		double PosiVector[3] = {};

		for (row = 0; row < 3; row++) {
			ArmPosition[row] = HTM.Mat3D((MAXJOINT - 1), row, 3);
			//[HTM.column * HTM.row * (MAXJOINT - 1) + 4 * row + 3];
		}

		for (jnt = 0; jnt < (MAXJOINT - 1); jnt++) {
			//printf("jnt -> %d\n", jnt);

			//0PE,n -> x
			for (int i = 0; i < 3; ++i) {
				PosiVector[i] = ArmPosition[i] - HTM.Mat3D(jnt, i, 3);
			}

			//Jacobianの外積計算

			double jacobBuf[MAXJOINT] = {};

			jacobBuf[0] = HTM.Mat3D(jnt, 1, 2) * PosiVector[2] - HTM.Mat3D(jnt, 2, 2) * PosiVector[1];
			jacobBuf[1] = HTM.Mat3D(jnt, 2, 2) * PosiVector[0] - HTM.Mat3D(jnt, 0, 2) * PosiVector[2];
			jacobBuf[2] = HTM.Mat3D(jnt, 0, 2) * PosiVector[1] - HTM.Mat3D(jnt, 1, 2) * PosiVector[0];

			jacobBuf[3] = HTM.Mat3D(jnt, 0, 2);
			jacobBuf[4] = HTM.Mat3D(jnt, 1, 2);
			jacobBuf[5] = HTM.Mat3D(jnt, 2, 2);

			for (int i = 0; i < 6; ++i) {
				JACOB.Mat2D(i, jnt, jacobBuf[i]);
			}

			if (selfmotion) {
				//J(6,jnt)
				JACOB.Mat2D(6, jnt, 0.0);
				if (jnt == 2) {
					JACOB.Mat2D(6, jnt, 1.0);
				}
			}
		}
	}

	//pseudo inverse matrixを計算するプログラム
	int PIM(Matrix MAT, Matrix &PIMAT) {
		Matrix TMat;
		TMat.CreateDiMatrix(MAT.Column(), MAT.Row());
		Matrix sqMat;
		sqMat.CreateDiMatrix(MAT.Row(), MAT.Row());
		//Matrix *detMat;
		//detMat = CreateDiMatrix(mat->row, mat->row);
		Matrix IsqMat;
		IsqMat.CreateDiMatrix(MAT.Row(), MAT.Row());

		int row, column;
		bool check = true;
		//cout << "PIM started" << endl;


		//転置行列を作る(A^T)
		for (row = 0; row < MAT.Row(); row++) {
			for (column = 0; column < MAT.Column(); column++) {
				TMat.Mat2D(column, row, MAT.Mat2D(row, column));
			}
		}
		/////////////////////転置行列作った

		//mat*TMatの計算をする(A･A^T)
		for (row = 0; row < MAT.Row(); row++) {
			for (column = 0; column < MAT.Column(); column++) {
				double sqBuf = 0.0;

				for (int k = 0; k < MAXJOINT - 1; ++k) {
					sqBuf += MAT.Mat2D(row, k) * TMat.Mat2D(k, column);
				}

				sqMat.Mat2D(row, column, sqBuf);

				//sqmat[sqmat->column * row + column]	
				//	= mat->m[mat->column * row + 0] * TMat->m[mat->row * 0 + column]		//ここはなぜかfor(int k = 0; k < mat->column; k++)
				//	+ mat->m[mat->column * row + 1] * TMat->m[mat->row * 1 + column]		//のように省略すると
				//	+ mat->m[mat->column * row + 2] * TMat->m[mat->row * 2 + column]		//エラーは出ないけれども
				//	+ mat->m[mat->column * row + 3] * TMat->m[mat->row * 3 + column]		//値が狂ってしまうから
				//	+ mat->m[mat->column * row + 4] * TMat->m[mat->row * 4 + column]		//このままにして置かなければいけない
				//	+ mat->m[mat->column * row + 5] * TMat->m[mat->row * 5 + column]
				//	+ mat->m[mat->column * row + 6] * TMat->m[mat->row * 6 + column];
				//detMat->m[mat->row * row + column] = sqMat->m[mat->row * row + column];
			}
		}

		////////////////////////mat*Tmatのおわり

		check = sqMat.InverseMatrix(IsqMat);

		//行列式による判定
		//check = Ludcmp(detMat->m, mat->row, index, &d);
		//for (int i = 0; i < mat->row; i++) { d *= detMat->m[mat->row * i + i]; }

		//正方逆行列を作る((A･A^T)^-1)
		//cout << "Creating Square Inverse Matrix started" << endl;
		//check = Ludcmp(sqMat->m, mat->row, index, &d);

		/*
		for (int j = 0; j < mat->row; j++) {
			for (int i = 0; i < mat->row; i++)col[i] = 0.0;
			col[j] = 1.0;
			Lubksb(sqMat->m, mat->row, index, col);
			for (int i = 0; i < mat->row; i++) IsqMat->m[mat->row * i + j] = col[i];
		}
		*/

		if (check > 0) {
			printf(".....LU function error.....\n.....Can not calculate.....\n");
			return check;
		}

		//cout << "Creating Square Inverse Matrix finished" << endl;
		//DebagBar();	std::cout << "square inverse matrix\n";	DisplayDiMatrix(IsqMat);

		for (row = 0; row < PIMAT.Row(); ++row) {
			for (column = 0; column < PIMAT.Column(); ++column) {
				double piBuf = 0.0;

				for (int k = 0; k < PIMAT.Column(); ++k) {
					piBuf += TMat.Mat2D(row, k) * IsqMat.Mat2D(k, column);
				}

				PIMAT.Mat2D(row, column, piBuf);

				/*
				PIMat->m[PIMat->column * row + column]
					= TMat->m[PIMat->column * row + 0] * IsqMat->m[PIMat->column * 0 + column]
					+ TMat->m[PIMat->column * row + 1] * IsqMat->m[PIMat->column * 1 + column]
					+ TMat->m[PIMat->column * row + 2] * IsqMat->m[PIMat->column * 2 + column]
					+ TMat->m[PIMat->column * row + 3] * IsqMat->m[PIMat->column * 3 + column]
					+ TMat->m[PIMat->column * row + 4] * IsqMat->m[PIMat->column * 4 + column]
					+ TMat->m[PIMat->column * row + 5] * IsqMat->m[PIMat->column * 5 + column];
					*/
			}
		}

		//DebagBar();	std::cout << "Pseudo inverse matrix\n";	displayDiMatrix(PIMat);

		//物凄く小さい値なら0に近似する
		for (int i = 0; i < MAT.Row(); ++i) {
			for (int j = 0; j < MAT.Column(); ++j) {
				if (fabs(PIMAT.Mat2D(i, j) < COMPARE_ZERO)) {
					PIMAT.Mat2D(i, j, 0.0);
				}
			}
		}

		//cout << "PIM finished" << endl;

		TMat.~Matrix();
		sqMat.~Matrix();
		IsqMat.~Matrix();

		return check;
	}

	//class HandP の定義
	Kinematics::Kinematics() {
		//ch = new TarPoints;

		eX = 0;	eY = 0; eZ = 0;
		wX = 0;	wY = 0; wZ = 0;
		X = 0;	Y = 0; Z = 0;
	}

	Kinematics::~Kinematics() {
		om.~Matrix();
		htm.~Matrix();
		jacob.~Matrix();
		iJacob.~Matrix();
	}

	//メンバ変数の出力
	void Kinematics::DisplayCoordinate() {
		printf("elbow(x,y,z)\t->\t(%3.8lf\t%3.8lf\t%3.8lf)\n", eX, eY, eZ);
		printf("wrists(x,y,z)\t->\t(%3.8lf\t%3.8lf\t%3.8lf)\n", wX, wY, wZ);
		printf("fingers(x,y,z)\t->\t(%3.8lf\t%3.8lf\t%3.8lf)\n", X, Y, Z);
	}

	//肘座標を取得
	void Kinematics::GetElbowCoordinate(double *ec) {
		ec[0] = eX;
		ec[1] = eY;
		ec[2] = eZ;
	}

	//手首座標を取得
	void Kinematics::GetwristCoordinate(double *wc) {
		wc[0] = wX;
		wc[1] = wY;
		wc[2] = wZ;
	}

	//手先座標を取得
	void Kinematics::GetCoordinate(double *c) {
		c[0] = X;
		c[1] = Y;
		c[2] = Z;
	}

	//手先の同次変換行列を求める
	void Kinematics::GetHandHTM(double *currentRadian, double *handPostureMat) {

		om.CreateTriMatrix(4, 4, MAXJOINT);
		InitOM(currentRadian, om);

		htm.CreateTriMatrix(4, 4, MAXJOINT);
		CalcHTM(om, htm);

		for (int i = 0; i < htm.Row(); ++i) {
			for (int j = 0; j < htm.Column(); ++j) {
				handPostureMat[htm.Column() * i + j] = htm.Mat3D(7, i, j);
			}
		}

		//DebagComment("GetHandHTM : handPostureMat");	DisplayRegularMatrix(4, handPostureMat);
	}

	//順運動学計算
	void Kinematics::CalcFK(double *currentJointRad) {
		//DebagComment("FK Kinematics started");

		//DebagComment("CalcFK : current Joint Rad");
		//DisplayVector(8, currentJointRad);

		om.CreateTriMatrix(4, 4, MAXJOINT);

		//printf("calc FK htm");
		htm.CreateTriMatrix(4, 4, MAXJOINT);

		//(1)同次変換行列を作る
		InitOM(currentJointRad, om);
		//DisplayTriMatrix(om, 7);

		//(2)同次変換行列を作る
		CalcHTM(om, htm);
		//displayTriMatrix(htm);

		//(3)同次変換行列の手先位置に関する情報を書き出す．

		eX = htm.Mat3D(2, 0, 3);
		eY = htm.Mat3D(2, 1, 3);
		eZ = htm.Mat3D(2, 2, 3);

		wX = htm.Mat3D(4, 0, 3);
		wY = htm.Mat3D(4, 1, 3);
		wZ = htm.Mat3D(4, 2, 3);

		X = htm.Mat3D(7, 0, 3);
		Y = htm.Mat3D(7, 1, 3);
		Z = htm.Mat3D(7, 2, 3);

		//DebagComment("CalcFK : DisplayCoordinate");
		//DisplayCoordinate();
	}

	int Kinematics::CalcIK(double *currentRadian, double *handVelocity, double *nextRadVelocity) {
		//std::cout << "self motion calculation started\n";

		om.CreateTriMatrix(4, 4, MAXJOINT);
		htm.CreateTriMatrix(4, 4, MAXJOINT);
		jacob.CreateDiMatrix(7, 7);
		iJacob.CreateDiMatrix(7, 7);

		int check = 0;
		int selfMotion = 1;

		//同次変換行列の作成
		InitOM(currentRadian, om);
		//DebagComment("origin matrix"); om.Display();

		//同次変換行列の積の計算
		CalcHTM(om, htm);
		//DebagComment("htm"); htm.Display();

		//ヤコビ行列の計算
		CalcJacob(htm, selfMotion, jacob);
		//DebagComment("jacobian"); jacob.Display();

		//逆行列を作成
		check = jacob.InverseMatrix(iJacob);
		//DebagComment("inverse jacobian");	iJacob.Display();

		for (int i = 0; i < (MAXJOINT - 1); ++i) {
			double sumBuf = 0;
			for (int j = 0; j < (MAXJOINT - 1); ++j) {
				sumBuf += iJacob.Mat2D(i, j) * handVelocity[j];
			}
			nextRadVelocity[i] = sumBuf;
		}
		return check;
	}

	//calc inverse Kinematics
	int Kinematics::CalcPIK(double *currentRadian, double *handVelocity, double *nextRadVelocity) {
		//std::cout << "self motion calculation started\n";
		om.CreateTriMatrix(4, 4, MAXJOINT);
		htm.CreateTriMatrix(4, 4, MAXJOINT);
		jacob.CreateDiMatrix(6, 7);
		iJacob.CreateDiMatrix(7, 6);

		int check = 0;
		bool selfMotion = false;

		//同次変換行列の作成
		InitOM(currentRadian, om);
		//DisplayTriMatrix(om_m, MAXJOINT);

		//同次変換行列の積の計算
		CalcHTM(om, htm);
		//DisplayTriMatrix(htm_m, MAXJOINT);

		//ヤコビ行列の計算
		CalcJacob(htm, selfMotion, jacob);

		//逆行列を作成
		check = PIM(jacob, iJacob);

		for (int i = 0; i < (MAXJOINT - 1); ++i) {
			double sumBuf = 0;
			for (int j = 0; j < MAXJOINT - 2; ++j) {
				sumBuf += iJacob.Mat2D(i, j) * handVelocity[j];
			}
			nextRadVelocity[i] = sumBuf;
		}

		return check;
	}

	//ピック動作//参考程度に書いてます．
	void Pick(double *curJointRad, TarPoints tarCoord, double *currentTime, double *nextRadVelocity) {
		//１．手先のベクトルを確認
		double finger[3] = {};
		double wrist[3] = {};

		//GetCoordinate(finger);
		//GetwristCoordinate(wrist);

		std::vector<double> handVec(3, 0);
		//for (int i = 0; i < 3; ++i) handVec[i] = finger[i] - wrist[i];


		//２．さくらんぼのなり方を確認
		std::vector<double> targetVec(3, 0);
		for (int i = 0; i < 3; ++i) targetVec[i] = tarCoord.top[i] - tarCoord.mid[i];

		VectorNormalize(targetVec);

		//３．とる方向を決める
		//今は簡易的にさくらんぼのベクトルを見て
		//Zの

		double direction[3] = {};

		if (targetVec[2] > 0) {	//サクランボがZ方向に正なら
			direction[2] = -0.05;	//ｚ負方向に5㎝動かす
		}
		else if (targetVec[2] < 0) {//サクランボZ方向に負なら
			direction[2] = 0.05;	//ｚ正方向に5㎝動かす
		}
		else if (targetVec[2] == 0) {//正面を向いていたら、
			direction[0] = -0.05;	//x方向に5cm引く。
		}

		//４．現在の手先位置からとる方向に○cm動くように
		for (int i = 0; i < 3; ++i) {
			nextRadVelocity[i] = TrapeInterpolate(direction[i], TIME_LENGTH, *currentTime);
		}
	}

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
std::cout << i << " inrad->" << std::setw(10) << inRadian[i];
std::cout << "/speed->" << std::setw(10) << speed[i] << std::endl;
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
//DebagBar();	std::cout << "Jacob\n";	displayDiMatrix(Jacob->m, Jacob->row, Jacob->column);

//擬似逆行列を作成
check = PIM(Jacob, IJacob);

//擬似逆行列の表示
//DebagBar();	std::cout << "Inverse Jacob\n";	displayDiMatrix(IJacob->m, IJacob->row, IJacob->column);

//擬似逆行列が本当に正しいか元の行列と掛けあわせて確認(単位正方行列が出ればOK)
//DebagBar();	std::cout << "PIMConfirmation\n";	PIMConfirmation(Jacob,IJacob);

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
//DebagBar();
//std::cout << "mainCalc finished\n";
Free_Matrix(OM);
Free_Matrix(HTM);
Free_Matrix(Jacob);
Free_Matrix(IJacob);

return check;
}
*/


//資源置き場
/*
void Kinematics::VelocityRegulationPosture(TarPoints tarCoord, double *curJointRad, double *postureSpeed, double currentTime) {
	//同次変換行列
	static double handPostureRotMat[16] = {};
	static double graspRotMat[16] = {};

	//クォータニオンの入れ物
	static Quat currentPostureQ;
	static Quat targetPostureQ;

	//初期姿勢の算出
	if (currentTime < TIME_SPAN) {
		//このif文の中は合ってるくさい
		DebagComment("initilize first and last posture");

		Matrix *om = CreateTriMatrix(4, 4, MAXJOINT);
		InitOM(curJointRad, om);

		Matrix *htm = CreateTriMatrix(4, 4, MAXJOINT);
		CalcHTM(om, htm);

		for (int i = 0; i < htm->row; ++i) {
			for (int j = 0; j < htm->column; ++j) {
				handPostureRotMat[htm->row * i + j] = htm->m[(MAXJOINT - 1) * htm->row * htm->column + htm->column * i + j];
			}
		}

		//DebagComment("hand Posture rotation matrix");
		//DisplayRegularMatrix(4, handPostureRotMat);

		//目標姿勢算出
		std::vector<double> graspV(3, 0);
		tarCoord.graspDirection(graspV);
		//DisplayVector(graspV);

		std::vector<double> directionX(3, 0);

		for (int i = 0; i < 3; ++i) {
			directionX[i] = tarCoord.top[i] - tarCoord.btm[i];
		}

		DebagComment("direction x");
		DisplayVector(directionX);

		DebagComment("grasp Vector");
		DisplayVector(graspV);


		DirectVectorToRotMatrix(directionX, graspV, graspRotMat);

		//DebagComment("hand Posture rotation matrix");
		//DisplayRegularMatrix(4, graspRotMat);

		RotMatrixToQuat(handPostureRotMat, currentPostureQ);
		//DebagComment("current Posture quat");
		//currentPostureQ.display();

		//DebagComment("current Posture rotation mat");
		//DisplayRegularMatrix(4, handPostureRotMat);

		RotMatrixToQuat(graspRotMat, targetPostureQ);
		//DebagComment("target Posture quat");
		//targetPostureQ.display();

		//DebagComment("target Posture rotation mat");
		//DisplayRegularMatrix(4, graspRotMat);

	}

	if (currentTime < TIME_SPAN) {
		//DebagComment("currentTime = 0");
		for (int i = 0; i < 3; ++i) {
			postureSpeed[i] = 0.0;
		}
	}
	else if (currentTime > TIME_SPAN && currentTime < TIME_LENGTH)
	{
		//DebagComment("currenttime > 0");
		Quat beforeSlerpQ = currentPostureQ.slerp((currentTime / TIME_LENGTH) - TIME_SPAN, targetPostureQ);

		//DebagComment("before Slerp quat");
		//beforeSlerpQ.display();

		double beforeSlerpMat[16] = {};
		double beforeEulerV[3] = {};

		QuatToRotMatrix(beforeSlerpQ, beforeSlerpMat);

		RotMatrixToEuler(beforeSlerpMat, beforeEulerV);

		//DisplayVector(3, beforeEulerV);

		Quat nowSlerpQ = currentPostureQ.slerp((currentTime / TIME_LENGTH), targetPostureQ);

		//DebagComment("now Slerp quat");
		//nowSlerpQ.display();

		double nowSlerpMat[16] = {};
		double nowEulerV[3] = {};

		QuatToRotMatrix(nowSlerpQ, nowSlerpMat);

		RotMatrixToEuler(nowSlerpMat, nowEulerV);

		//DisplayVector(3, nowEulerV);

		//ここの速度算出の計算が違うっぽい

		double beforeAngluar[3] = {};
		double nowAngluar[3] = {};

		EulerToAngluar(beforeEulerV, beforeAngluar);
		EulerToAngluar(nowEulerV, nowAngluar);

		for (int i = 0; i < 3; ++i) postureSpeed[i] = (nowAngluar[i] - beforeAngluar[i]) * 1.0;
	}
	else if ((currentTime) >= TIME_LENGTH) {
		//DebagComment("(*currentTime) >= TIME_LENGTH");
		postureSpeed[0] = 0.0;
		postureSpeed[1] = 0.0;
		postureSpeed[2] = 0.0;
	}

	//DebagComment("posture speed");
	//DisplayVector(3,postureSpeed);

	//DisplayRegularMatrix(4, handPostureRotMat);
	//DisplayRegularMatrix(4, graspRotMat);
	//DebagComment("velocity posture finished");
}

*/

/*
//calc inverse Kinematics
int Kinematics::CalcIK(double *currentRadian, double *handVelocity, double *nextRadVelocity) {
//std::cout << "self motion calculation started\n";

std::vector<std::vector<std::vector<double>>> om_m;	//original matrix
std::vector<std::vector<double>> htm_m;	//homogeneous translate matrix
std::vector<std::vector<double>> jacob_m;	//jacobian
std::vector<std::vector<double>> iJacob_m;	//inverse jacobian
int check = 0;
int selfMotion = 1;

//double bufRotMat[16] = {};

//同次変換行列の作成
InitOM(currentRadian, om_m);
//DisplayTriMatrix(om_m, MAXJOINT);

//手先に関する操作をするとしたらここ
//ch->CalcHandPosture(currentRadian, bufRotMat);
//DisplayRegularMatrix(4, bufRotMat);

//同次変換行列の積の計算
CalcHTM(om_m, htm_m);
//DisplayTriMatrix(htm_m, MAXJOINT);

//クォータニオンによる姿勢制御の変換式
htm_m->m[htm_m->column * htm_m->row * 6 + htm_m->column * 0 + 0] = bufRotMat[4 * 0 + 1];
htm_m->m[htm_m->column * htm_m->row * 6 + htm_m->column * 1 + 0] = bufRotMat[4 * 1 + 1];
htm_m->m[htm_m->column * htm_m->row * 6 + htm_m->column * 2 + 0] = bufRotMat[4 * 2 + 1];

htm_m->m[htm_m->column * htm_m->row * 6 + htm_m->column * 0 + 1] = bufRotMat[4 * 0 + 2];
htm_m->m[htm_m->column * htm_m->row * 6 + htm_m->column * 1 + 1] = bufRotMat[4 * 1 + 2];
htm_m->m[htm_m->column * htm_m->row * 6 + htm_m->column * 2 + 1] = bufRotMat[4 * 2 + 2];

htm_m->m[htm_m->column * htm_m->row * 6 + htm_m->column * 0 + 2] = bufRotMat[4 * 0 + 0];
htm_m->m[htm_m->column * htm_m->row * 6 + htm_m->column * 1 + 2] = bufRotMat[4 * 1 + 0];
htm_m->m[htm_m->column * htm_m->row * 6 + htm_m->column * 2 + 2] = bufRotMat[4 * 2 + 0];
//

//DisplayTriMatrix(htm_m, MAXJOINT);

//ヤコビ行列の計算
CalcJacob(htm_m, jacob_m, selfMotion);
//DisplayDiMatrix(jacob_m);

//逆行列を作成
check = InverseMatrix(jacob_m, iJacob_m);
//DisplayDiMatrix(iJacob_m);

//DisplayVector(7, handVelocity);

for (int i = 0; i < (MAXJOINT - 1); i++) {
nextRadVelocity[i] =
iJacob_m->m[7 * i + 0] * handVelocity[0]
+ iJacob_m->m[7 * i + 1] * handVelocity[1]
+ iJacob_m->m[7 * i + 2] * handVelocity[2]
+ iJacob_m->m[7 * i + 3] * handVelocity[3]
+ iJacob_m->m[7 * i + 4] * handVelocity[4]
+ iJacob_m->m[7 * i + 5] * handVelocity[5]
+ iJacob_m->m[7 * i + 6] * handVelocity[6];
}

FreeMatrix(om_m);
FreeMatrix(htm_m);
FreeMatrix(jacob_m);
FreeMatrix(iJacob_m);

return check;
}
*/

/*
void Kinematics::VelocityRegulationPosture(TarPoints tarCoord, double *curJointRad, double currentTime, double *postureSpeed) {
static double kPostureSpeed[3] = {};

static double beforeTime = 0.0;

//クォータニオンの入れ物
static Quat currentPostureQ;
static Quat targetPostureQ;

if (currentTime < TIME_SPAN) {
currentPostureQ = StartPosture(curJointRad);
targetPostureQ = EndPosture(tarCoord);

//DebagComment("before Slerp quat");
//beforeSlerpQ.display();

double beforeSlerpMat[16] = {};
double beforeEulerV[3] = {};

QuatToRotMatrix(beforeSlerpQ, beforeSlerpMat);

RotMatrixToEuler(beforeSlerpMat, beforeEulerV);

//DisplayVector(3, beforeEulerV);

Quat nowSlerpQ = currentPostureQ.slerp((currentTime / TIME_LENGTH), targetPostureQ);

//DebagComment("now Slerp quat");
//nowSlerpQ.display();

double nowSlerpMat[16] = {};
double nowEulerV[3] = {};

QuatToRotMatrix(nowSlerpQ, nowSlerpMat);

RotMatrixToEuler(nowSlerpMat, nowEulerV);

//DisplayVector(3, nowEulerV);

//ここの速度算出の計算が違うっぽい

double beforeAngluar[3] = {};
double nowAngluar[3] = {};

EulerToAngluar

//improvement required!!
//ロール軸回転(kPostureSpeed[0])の場合分けが必要。一つの方向にしか回らない。

//if(directionX[0] > )

kPostureSpeed[0] = (nowAngluar[0] - beforeAngluar[0]) / TIME_SPAN;
kPostureSpeed[1] = (nowAngluar[1] - beforeAngluar[1]) / TIME_SPAN;
kPostureSpeed[2] = (nowAngluar[2] - beforeAngluar[2]) / TIME_SPAN;

DebagComment("kposture speed");
DisplayVector(3, kPostureSpeed);
}

if (currentTime < TIME_SPAN) {
//DebagComment("currentTime = 0");
for (int i = 0; i < 3; ++i) {
postureSpeed[i] = 0.0;
}
}
else if (currentTime > TIME_SPAN && currentTime < TIME_LENGTH) {
//DebagComment("currentTime > 0");

double nowTime = TrapeInterpolate(1, TIME_LENGTH, nowTime);
Quat beforeSlerpQ = currentPostureQ.slerp(, targetPostureQ);

for (int i = 0; i < 3; ++i) {
postureSpeed[i] = TrapeInterpolate(kPostureSpeed[i], TIME_LENGTH, currentTime);
}

//DisplayVector(3, postureSpeed);
}
else if ((currentTime) >= TIME_LENGTH) {
//DebagComment("(*currentTime) >= TIME_LENGTH");
postureSpeed[0] = 0.0;
postureSpeed[1] = 0.0;
postureSpeed[2] = 0.0;
}

//DebagComment("posture speed");
//DisplayVector(3,postureSpeed);

//DisplayRegularMatrix(4, handPostureRotMat);
//DisplayRegularMatrix(4, graspRotMat);
//DebagComment("velocity posture finished");
}
*/
