// 2017/04/28
// created by eiki obara

#pragma once

#ifndef __KINEMA_DEBAG_H__
#define __KINEMA_DEBAG_H__

#include "include\kine_matrix.h"

//display
void kinemaDebagON(void);
void kinemaDebagOFF(void);
void DebagBar(void);
void DebagComment(char *c);
void DebagCommentEnt(char *c);
void ErrComment(char *c);
void DisplayVector(const int n, const double *v);
void DisplayVector(const int n, const float *v);
void DisplayRegularMatrix(const int n, double *mat);
void OutputMatrixTxt(double *m, const int column, double loopnum, char *filename);
void DisplayDiMatrix(Matrix *m);
void DisplayTriMatrix(Matrix *m, int dispNum);
void PushEnter(void);

#endif // !__KINEMA_DEBAG_H__
