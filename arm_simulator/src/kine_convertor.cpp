// 2017/04/28
// created by eiki obara

#define _USE_MATH_DEFINES

#include "include\kine_vector.h"
#include "include\kine_quat.h"
#include "include\kine_config.h"
#include "include\kine_debag.h"

#include <math.h>

void Euler2Angular(const double nowEuler[3], const double eulerVelocity[3], double *angleVelocity) {
	//euler[0] = ƒÓ, euler[1] = ƒÆ, euler[2] = ƒÕ‚É‘Î‰ž‚µ‚Ä‚¢‚é(‚Í‚¸)
	
	//DebagComment("now euler vector");
	//DisplayVector(3, nowEuler);

	//DebagComment("euler velocity");
	//DisplayVector(3, eulerVelocity);

	double euler2angularM[9] = {};
	
	//phi, theta, psi
	euler2angularM[0] = 0;
	euler2angularM[1] = -sin(nowEuler[0]);
	euler2angularM[2] = cos(nowEuler[0]) * sin(nowEuler[1]);

	euler2angularM[3] = 0;
	euler2angularM[4] = cos(nowEuler[0]);
	euler2angularM[5] = sin(nowEuler[0]) * sin(nowEuler[1]);

	euler2angularM[6] = 1;
	euler2angularM[7] = 0;
	euler2angularM[8] = cos(nowEuler[1]);

	//DebagComment("transform matrix");
	//DisplayRegularMatrix(3, euler2angularM);

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			angleVelocity[i] += (euler2angularM[3 * i + j] * eulerVelocity[j]);
		}
	}

	//DebagComment("angle velocity");	DisplayVector(3, angleVelocity);

	//printf("euler to angular finished\n");
}

inline float SIGN(float x) {
	if (x >= 0.0f) {
		return 1.0f;
	}
	else {
		return -1.0f;
	}
}

void RotMat2Quat(const double *rotMat, Quat &quat) {
	//DebagComment("rotation matrix to quaternion");
	//http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/

	double trace = rotMat[4 * 0 + 0] + rotMat[4 * 1 + 1] + rotMat[4 * 2 + 2];
	double q[4] = {};

	if (trace > 0) {
		double s = 0.5 / powf(trace + 1.0f, 0.5);
		q[0] = 0.25f / s;
		q[1] = (rotMat[4 * 2 + 1] - rotMat[4 * 1 + 2]) * s;
		q[2] = (rotMat[4 * 0 + 2] - rotMat[4 * 2 + 0]) * s;
		q[3] = (rotMat[4 * 1 + 0] - rotMat[4 * 0 + 1]) * s;
		quat.assign(q);
	}
	else {
		if (rotMat[4 * 0 + 0] > rotMat[4 * 1 + 1] && rotMat[4 * 0 + 0] > rotMat[4 * 2 + 2]) {
			double s = 2.0f * powf(1.0f + rotMat[4 * 0 + 0] - rotMat[4 * 1 + 1] - rotMat[4 * 2 + 2], 0.5);

			q[0] = (rotMat[4 * 2 + 1] - rotMat[4 * 1 + 2]) / s;
			q[1] = 0.25f * s;
			q[2] = (rotMat[4 * 0 + 1] + rotMat[4 * 1 + 0]) / s;
			q[3] = (rotMat[4 * 1 + 2] + rotMat[4 * 2 + 1]) / s;
		}
		else if (rotMat[4 * 1 + 1] > rotMat[4 * 2 + 2]) {
			double s = 2.0f * powf(1.0f - rotMat[4 * 0 + 0] + rotMat[4 * 1 + 1] - rotMat[4 * 2 + 2], 0.5);

			q[0] = (rotMat[4 * 0 + 2] - rotMat[4 * 2 + 0]) / s;
			q[1] = (rotMat[4 * 0 + 1] + rotMat[4 * 1 + 0]) / s;
			q[2] = 0.25f * s;
			q[3] = (rotMat[4 * 1 + 2] + rotMat[4 * 2 + 1]) / s;
		}
		else {
			double s = 2.0f * powf(1.0f - rotMat[4 * 0 + 0] - rotMat[4 * 1 + 1] + rotMat[4 * 2 + 2], 0.5);

			q[0] = (rotMat[4 * 1 + 0] - rotMat[4 * 0 + 1]) / s;
			q[1] = (rotMat[4 * 0 + 2] + rotMat[4 * 2 + 0]) / s;
			q[2] = (rotMat[4 * 1 + 2] + rotMat[4 * 2 + 1]) / s;
			q[3] = 0.25f * s;
		}
	}

	for (int i = 0; i < 4; ++i) {
		if (fabs(q[i]) < kine::COMPARE_ZERO) {
			q[i] = 0.0;
		}
	}

	quat.assign(q);
}

void DirectVector2RotMat(std::vector<double> directionX, std::vector<double> directionZ, double *matrix) {
	//DebagComment("vector to rotation matrix");

	VectorNormalize(directionX);
	VectorNormalize(directionZ);

	//printf("x\n");DisplayVector(directionX);
	//printf("z\n");	DisplayVector(directionZ);

	//DebagComment("cross product vector z axis ");
	std::vector<double> directionY(3, 0);
	directionY = CrossVector(directionZ, directionX);

	//DebagComment("normalize vector x axis ");
	VectorNormalize(directionY);

	//DebagComment("direction x");	DisplayVector(directionX);
	//DebagComment("direction y");	DisplayVector(directionY);
	//DebagComment("direction z");	DisplayVector(directionZ);

	//DebagComment("initializing rotation matrix in VecToRotMat");
	matrix[0] = directionX[0];
	matrix[4] = directionX[1];
	matrix[8] = directionX[2];
	matrix[12] = 0;

	matrix[1] = directionY[0];
	matrix[5] = directionY[1];
	matrix[9] = directionY[2];
	matrix[13] = 0;

	matrix[2] = directionZ[0];
	matrix[6] = directionZ[1];
	matrix[10] = directionZ[2];
	matrix[14] = 0;

	matrix[3] = 0;
	matrix[7] = 0;
	matrix[11] = 0;
	matrix[15] = 1;

	//DisplayRegularMatrix(4, matrix);

	for (int i = 0; i < 16; ++i) {
		if (fabs(matrix[i]) < kine::COMPARE_ZERO) {
			matrix[i] = 0.0;
		}
		if (matrix[i] == -0.0) {
			matrix[i] == 0.0;
		}
	}

	//DebagComment("vector to rotation matrix finished");
}
