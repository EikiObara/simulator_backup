#include "include\cameraOffset.h"


//近距離カメラのオフセット。
//originMatにカメラから実際に得た3次元座標を渡すと
//オフセットしてconvertMatに返す。
void ShortCameraOffset(const double *originMat, double *convertMat) {
	convertMat[0] = originMat[2] - 109;
	convertMat[1] = originMat[1] - 2;
	convertMat[2] = originMat[0] + 100;
}

//遠距離カメラのオフセット。
//originMatにカメラから実際に得た3次元座標を渡すと
//オフセットしてconvertMatに返す。
void LongCameraOffset(const double *originMat, double *convertMat) {
	convertMat[0] = originMat[2] + 143.0;
	convertMat[1] = originMat[1] + 208.0;
	convertMat[2] = originMat[0] - 314.0;
}