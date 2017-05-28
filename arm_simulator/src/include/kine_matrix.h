// 2017/04/28
// created by eiki obara

#pragma once

#ifndef __KINE_MATRIX_H__
#define __KINE_MATRIX_H__

#include <vector>

/*
typedef struct {
	int row;
	int column;
	int numMat;
	double *m;
}Matrix;

Matrix *CreateDiMatrix(int row, int column);
Matrix *CreateTriMatrix(int row, int column, int numMat);
void FreeMatrix(Matrix *matrix);
int InverseMatrix(Matrix *OriginMat, Matrix *InverseMat);
*/

class Matrix {
public:
	Matrix();
	Matrix(int myRow, int myColumn);
	Matrix(int myRow, int myColumn, int matrixSize);
	~Matrix();

	void Display();

	int Row();
	int Column();
	int MatSize();

	void Mat2D(int myRow, int myColumn, double val);
	double Mat2D(int myRow, int myColumn);

	void Mat3D(int matNum, int myRow, int myColumn, double val);
	double Mat3D(int matNum, int myRow, int myColumn );

	void CreateDiMatrix(const int myRow, const int myColumn);
	void CreateTriMatrix(const int myRow, const int myColumn, const int myMatSize);

	bool InverseMatrix(Matrix &returnMat);

private:
	enum{twoD = 1, threeD = 2};

	int mRow;
	int mCol;
	int mNum;
	int mType;
	std::vector<double> m;	//êîíläiî[
};

#endif //__KINE_MATRIX_H__
