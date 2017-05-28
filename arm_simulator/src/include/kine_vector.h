// 2017/04/28
// created by eiki obara

#pragma once

#ifndef __CALC_VECTOR_H__
#define __CALC_VECTOR_H__

#include <vector>

//
void DisplayVector(const std::vector<double> v);

std::vector<double> AddVector(const std::vector<double> reftV, const std::vector<double> rightV);

std::vector<double> SubVector(const std::vector<double> reftV, const std::vector<double> rightV);

std::vector<double> mulVector(const std::vector<double> reftV, const double mulValue);

std::vector<double> ArrayToVect(double *originArray);

//ì‡êœ
double InnerVectorSqr(const std::vector<double> v1, const std::vector<double> v2);

double InnerVector(const std::vector<double> v1, const std::vector<double> v2);

double InnerVector(const double reftV[3], const double rightV[3]);

//äOêœ
std::vector<double> CrossVector(const std::vector<double> &reftV, const std::vector<double> &rightV);

void CrossVector(const double reftV[3], const double rightV[3], double *returnV);

//ê≥ãKâª
void VectorNormalize(std::vector<double> &returnV);

//âÒì]äpéZèo
double GetRotValue(std::vector<double> &curAxis, std::vector<double> &tarAxis);


#endif //__CALC_VECTOR_H__