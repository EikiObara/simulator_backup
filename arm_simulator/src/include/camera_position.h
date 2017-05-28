// 2017/04/28
// created by eiki obara

#pragma once
#ifndef __CAMERA_POSITION_H__
#define __CAMERA_POSITION_H__

void ShortCameraOffset(const double *originMat, double *convertMat);
void LongCameraOffset(const double *originMat, double *convertMat);
void CalcHandCameraPosition(double *curRad, double *cameraCoord);

#endif //__CAMERA_POSITION_H__