#define _USE_MATH_DEFINES

#include <math.h>
#include <stdio.h>
#include <fstream>

static double OBSTACLE_WEIGHT = 0.1;
static double GOAL_WEIGHT = 0.1;


class obstacle {
private:
	int obstNumber;
	double *obstCoord;	//è·äQï®ÇÃç¿ïWäiî[
public:
	void getObstCoord(double *p);
	void setObstCoord(double *p);
};

//ptn = potencial
void PotencialMethod(const double *obstCoord, const double *goalCoord, double *curCoord){
	double goalPtn;
	double obstPtn;
	double sumPtn;

	std::ofstream fout;

	double bufPow[3];

	for (int i = 0; i < 3; ++i) {
		bufPow[i] = pow(curCoord[i] - goalCoord[i], 2);
	}

	goalPtn += -1/sqrt(bufPow[0] + bufPow[1] + bufPow[2]);

	for (int i = 0; i < 3; ++i) {
		bufPow[i] = pow(curCoord[i] - obstCoord[i], 2);
	}
	
}