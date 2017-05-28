// 2017/04/28
// created by eiki obara

#include "include\kine_matrix.h"
#include "include\kine_debag.h"
#include "include\lu.h"
#include "include\kine_config.h"

//#include <stdlib.h>
#include <stdio.h>

Matrix::Matrix() {
	mRow = mCol = mNum = 0;
	m.resize(1);
	mType = 0;
}

Matrix::Matrix(int myRow, int myColumn) {
	mRow = myRow;
	mCol = myColumn;
	mNum = 1;
	m.resize(myRow * myColumn);
	mType = twoD;
}

Matrix::Matrix(int myRow, int myColumn, int matrixSize) {
	mRow = myRow;
	mCol = myColumn;
	mNum = matrixSize;
	m.resize(myRow * myColumn * matrixSize);
	mType = threeD;
}

/*
//行列用構造体のメモリ解放
void FreeMatrix(Matrix *matrix) {
	delete[] matrix->m;
}
*/

Matrix::~Matrix() {
	m.~vector();
}

void Matrix::Display() {
	for (int jnt = 0; jnt < MatSize(); ++jnt) {
		for (int i = 0; i < mRow; ++i) {
			for (int j = 0; j < mCol; ++j) {
				if (j == 0) { printf("|"); }
				if (mType == twoD) {
					printf("%lf\t", Mat2D(i, j));
				}
				if (mType == threeD) {
					printf("%lf\t", Mat3D(jnt, i, j));
				}
			}
			printf("|\n");
		}
		printf("\n");
	}
	printf("\n");
}

void Matrix::Mat2D(int myRow, int myColumn, double val){
	m[mCol * myRow + myColumn] = val;
}

double Matrix::Mat2D(int myRow, int myColumn){
	double buf = m[mCol * myRow + myColumn];

	return buf;
}

void Matrix::Mat3D(int matNum, int myRow, int myColumn,  double val) {
	m[mRow * mCol * matNum + mCol * myRow + myColumn] = val;
}

double Matrix::Mat3D(int matNum, int myRow, int myColumn) {
	double buf = m[mCol * mRow * matNum + mCol * myRow + myColumn];

	return buf;
}

int Matrix::Row(void) {
	return mRow;
}

int Matrix::Column(void) {
	return mCol;
}

int Matrix::MatSize(void) {
	return mNum;
}

//明示的にマトリックスのメンバ変数を変更
void Matrix::CreateDiMatrix(const int myRow, const int myColumn) {
	//printf("create di matrix : ");

	if (myRow == mRow && myColumn == mCol) {
		//printf("no change\n");
		//どちらも変わらないならば更新しない
	}
	else {
		//printf("change \n");
		//どちらか変わるなら更新する
		m.resize(myRow * myColumn);

		mRow = myRow;
		mCol = myColumn;
		mNum = 1;
		mType = twoD;
	}
}

void Matrix::CreateTriMatrix(int myRow, int myColumn, int myMatSize) {
	//printf("create tri matrix : ");

	if (myRow == mRow && myColumn == mCol) {
		//printf("no change\n");
		//どちらも変わらないならば更新しない
	}
	else {
		//printf("change\n");
		//どちらかでも変わるなら更新する
		m.resize(myRow * myColumn * myMatSize);

		mRow = myRow;
		mCol = myColumn;
		mNum = myMatSize;
		mType = threeD;
	}

	//printf("create tri matrix finished\n");
}

bool Matrix::InverseMatrix(Matrix &returnMat) {
	//std::cout << "InverseMatrix started\n";
	double d = 0;
	double col[10] = {};

	int indx[10] = {};
	int check = 0;

	Matrix detMat;
	detMat.CreateDiMatrix(mRow, mRow);

	for (int i = 0; i < mRow; ++i) {
		for (int j = 0; j < mCol; ++j) {
			detMat.m[detMat.Column() * i + j] = m[mCol * i + j];
		}
	}

	check = Ludcmp(detMat.m, detMat.Row(), indx, &d);
	
	for (int i = 0; i < detMat.Row(); i++) { d *= detMat.m[detMat.Column() * i + i]; }
	
	if (d == 0) {
		ErrComment("\n.....inverse matrix determinant = 0.....\n\
			.....Can not calculate.....\n");
		return 1;
	}

	if (check > 0) {
		return check;
	}

	check = Ludcmp(m, mRow, indx, &d);

	for (int j = 0; j < mRow; ++j) {
		for (int i = 0; i < mRow; ++i) col[i] = 0.0;

		col[j] = 1.0;
		Lubksb(m, mRow, indx, col);

		for (int i = 0; i < mRow; ++i){
			returnMat.Mat2D(i,j, col[i]);
		}
	}
	return check;
}
