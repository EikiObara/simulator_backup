// 2017/04/28
// created by eiki obara

#pragma once

#ifndef __3D_OBJECTS_H__
#define __3D_OBJECTS_H__

#include <vector>
#include <Inventor\Win\SoWin.h>
#include <Inventor\So.h>

void GoalCherry(std::vector<double> position, SoSeparator *GoalSep);

void PointObj(std::vector<double> position, SoSeparator *redPoint);

void PointObj(SoSeparator *redPoint);

void AlmiFlame(std::vector<double> position, SoSeparator *armFlame);

void CoordinateSystem(SoSeparator *coordinateSystem);

void ConeObj(SoSeparator *cone);

#endif //__3D_OBJECTS_H__