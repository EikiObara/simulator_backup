// 2017/04/28
// created by eiki obara

#include "include\plane.h"
#include "include\kine_vector.h"
#include "include\kine_debag.h"

//二つの平面方程式のパラメータから角度を得る
void AngleOfPlane(double p1[4], double p2[4], double &angle) {
	double numerator = 0;
	double denominator = 0;

	numerator = fabs(p1[0] * p2[0] + p1[1] * p2[1] + p1[2] * p2[2]);
	denominator = pow(p1[0] * p1[0] + p1[1] * p1[1] + p1[2] * p1[2], 0.5) * pow(p2[0] * p2[0] + p2[1] * p2[1] + p2[2] * p2[2], 0.5);

	if (denominator == 0) {
		ErrComment(" angle of plane function : ERROR \n denominator = 0 ");
		angle = 0.0f;
		return;
	}
	else {
		angle = acos(numerator / denominator);
	}
}

//空間中の3点から平面方程式のパラメータを求める
void Plane(std::vector<double> p1, std::vector<double> p2, std::vector<double> p3, std::vector<double> plane) {
	double v1[3] = { p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2] };
	double v2[3] = { p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2] };
	double crossV[3] = {};

	CrossVector(v1, v2, crossV);

	plane[0] = crossV[0];
	plane[1] = crossV[1];
	plane[2] = crossV[2];
	plane[3] = -1 * (crossV[0] * p1[0] + crossV[1] * p1[1] + crossV[2] * p1[2]);

	for (int i = 0; i < 4; ++i) {
		if (plane[i] == -0.0) {
			plane[i] = 0.0;
		}
	}
}

