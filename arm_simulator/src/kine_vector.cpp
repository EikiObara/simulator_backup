// 2017/04/28
// created by eiki obara

#define _USE_MATH_DEFINES

//#include <iostream>
//#include <math.h>

#include "include\kine_debag.h"
#include "include\kine_vector.h"
#include "include\kine_config.h"

std::vector<double> AddVector(const std::vector<double> reftV, const std::vector<double> rightV) {
	std::vector<double> returnV(reftV.size(),0);
	for (unsigned int i = 0; i < reftV.size(); ++i) {
		returnV[i] = reftV[i] + rightV[i];
	}
	return returnV;
}

std::vector<double> SubVector(const std::vector<double> originV, const std::vector<double> minusV) {
	std::vector<double> returnV(originV.size(), 0);
	for (unsigned int i = 0; i < originV.size(); ++i) {
		returnV[i] = originV[i] - minusV[i];
	}
	return returnV;
}

//é¿êîÇÃä|ÇØéZ
std::vector<double> mulVector(const std::vector<double> originV, const double mulValue) {
	std::vector<double> returnV(3, 0);

	for (int i = 0; i < originV.size(); ++i) {
		returnV[i] = originV[i] * mulValue;
	}

	return returnV;
}

std::vector<double> ArrayToVect(double *originArray) {
	std::vector<double> returnVector;
	returnVector.assign(&originArray[0], &originArray[3]);
	return returnVector;
}

double InnerVectorSqr(std::vector<double> v1, std::vector<double> v2) {
	double buf = 0;

	//DebagComment("inner vector square");

	buf = v1[0] * v2[0] + v1[1] * v2[1] +v1[2] * v2[2];

	if (buf < 0) {
		//DebagCom("calculation result : minus value");
		buf *= -1;	//translate plus 
	}

	//DebagComment("inner vector square finished");

	return buf;
}

double InnerVector(std::vector<double> v1, std::vector<double> v2) {
	double buf = 0;

	//DebagComment("inner vector");

	buf = sqrt(InnerVectorSqr(v1, v2));

	//DebagComment("inner vector finished");

	return buf;
}

double InnerVector(const double reftV[3], const double rightV[3]) {
	double buf = 0;

	for (int i = 0; i < 3; ++i) {
		buf += reftV[i] * rightV[i];
	}

	buf = pow(buf, 0.5);

	return buf;
}



//#include <vector> version
std::vector<double> CrossVector(const std::vector<double> &reftV, const std::vector<double> &rightV) {
	//DebagComment("cross vector");
	std::vector<double> returnV(3,0);

	returnV[0] = reftV[1] * rightV[2] - reftV[2] * rightV[1];
	returnV[1] = reftV[2] * rightV[0] - reftV[0] * rightV[2];
	returnV[2] = reftV[0] * rightV[1] - reftV[1] * rightV[0];

	//ïâÇÃÉ[ÉçÇê≥ÇÃÉ[ÉçÇ…ïœä∑
	for (int i = 0; i < 3; ++i) {
		if (returnV[i] == -0.0) {
			returnV[i] = 0.0;
		}
	}
	//DebagComment("return : cross vector");
	//DisplayVector(returnV);

	return returnV;
}

//array version
void CrossVector(const double reftV[3], const double rightV[3], double *returnV) {
	returnV[0] = reftV[1] * rightV[2] - reftV[2] * rightV[1];
	returnV[1] = reftV[2] * rightV[0] - reftV[0] * rightV[2];
	returnV[2] = reftV[0] * rightV[1] - reftV[1] * rightV[0];
}

void VectorNormalize(std::vector<double> &returnV) {
	double normBuf;

	normBuf = InnerVector(returnV, returnV);

	for (int i = 0; i < 3; ++i) {
		returnV[i] /= normBuf;
	}

}

//calculate rotate value between 2 vector;
double GetRotValue(std::vector<double> &curAxis, std::vector<double> &tarAxis) {
	double cur = 0;
	double tar = 0;
	double cosValue;

	VectorNormalize(curAxis);
	VectorNormalize(tarAxis);

	cur = InnerVector(curAxis, curAxis);
	//printf("currentAxisNorm -> %lf\n", cur);

	tar = InnerVector(tarAxis, tarAxis);
	//printf("targetAxisNorm -> %lf\n", tar);

	cosValue = InnerVector(curAxis, tarAxis) / (cur * tar);
	//printf("cosValue -> %lf\n", cosValue);

	//printf("%lf\n", theta);

	//printf("rotValue -> %3.9lf\n", fabs(acos(cosValue)));

	return fabs(acos(cosValue));
}

/*
double *SetVector(int nl) {
	//cout << "vector malloc started" << endl;
	double *v = new double[nl + NR_END];

	if (!v) {
		std::cout << "memory failure in vector()" << std::endl;
	}

	return v;
}

void FreeVector(double *v) {
	//cout << "free_vector started" << endl;
	delete[] v;
}
*/

void DisplayVector(std::vector<double> v) {
	//printf("display Vector\n");
	for (int i = 0; i < v.size(); ++i) {
		printf("v[%d]-> %3.8lf\n", i, v[i]);
	}
}