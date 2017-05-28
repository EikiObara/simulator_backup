// 2017/04/28
// created by eiki obara

#pragma once

#ifndef __LU_H__
#define __LU_H__

double *Dvector(long nh);
void Free_dvector(double *v);
int *Ivector(long nh);
void Free_ivector(int *v);

int Ludcmp(std::vector<double> &m, int n, int *index, double *d);
void Lubksb(std::vector<double> &m, int matSize, int *index, double *b);

#endif // !__LU_H__
