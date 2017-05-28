// 2017/04/28
// created by eiki obara

#pragma once

#ifndef __PLANE_H__
#define __PLANE_H__

#include <vector>

void AngleOfPlane(double p1[4], double p2[4], double &angle);

void Plane(std::vector<double> p1, std::vector<double> p2, std::vector<double> p3, std::vector<double> plane);

#endif // !__PLANE_H__
