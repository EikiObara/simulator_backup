// 2017/04/28
// created by eiki obara
#define _USE_MATH_DEFINES
#include <stdio.h>
#include <math.h>

#include "include\kine_quat.h"
#include "include\kine_debag.h"
#include "include\kine_config.h"
#include "include\kine_convertor.h"
#include "include\kine_vector.h"

#define QUAT_ELEM 4 //Quatenion Elements

void NormalizeQuatMatrix(double *m);

//constractor
Quat::Quat() {
	x = y = z = 0.0;
	w = 1.0;
}
Quat::Quat(double wValue, double xValue, double yValue, double zValue) {
	w = wValue;
	x = xValue;
	y = yValue;
	z = zValue;
}
Quat::Quat(double wValue, std::vector<double> vec3) {
	w = wValue;
	x = vec3[0];
	y = vec3[1];
	z = vec3[2];
}
Quat::Quat(std::vector<double> vec4) {
	w = vec4[0];
	x = vec4[1];
	y = vec4[2];
	z = vec4[3];
}
Quat::Quat(double *vec4) {
	w = vec4[0];
	x = vec4[1];
	y = vec4[2];
	z = vec4[3];
}

Quat::~Quat(){}

void Quat::Quat2array(double array4[4]) {
	array4[0] = w;
	array4[1] = x;
	array4[2] = y;
	array4[3] = z;
}

void Quat::assign() {
	x = y = z = 0.0;
	w = 1.0;

}
void Quat::assign(double wValue, double xValue, double yValue, double zValue) {
	w = wValue;
	x = xValue;
	y = yValue;
	z = zValue;
}
void Quat::assign(double wValue, std::vector<double> vec3) {
	w = wValue;
	x = vec3[0];
	y = vec3[1];
	z = vec3[2];
}
void Quat::assign(std::vector<double> vec4) {
	w = vec4[0];
	x = vec4[1];
	y = vec4[2];
	z = vec4[3];
}
void Quat::assign(double *vec4) {
	w = vec4[0];
	x = vec4[1];
	y = vec4[2];
	z = vec4[3];
}
void Quat::assign(Quat originQ) {
	double buf[4] = {};
	originQ.Quat2array(buf);

	w = buf[0];
	x = buf[1];
	y = buf[2];
	z = buf[3];
}

//calculation
Quat Quat::add(Quat c) {
	Quat returnQ;

	double bufC[4] = {};
	c.Quat2array(bufC);

	double bufA[4] = {};
	Quat2array(bufA);


	double addBuf[4] = {};
	for (int i = 0; i < 4; ++i)	addBuf[i] = bufA[i] + bufC[i];

	returnQ.assign(addBuf);

	return returnQ;
}

Quat Quat::sub(Quat c) {
	Quat returnQ;

	double bufC[4] = {};
	c.Quat2array(bufC);

	double bufA[4] = {};
	Quat2array(bufA);

	double subBuf[4] = {};

	for (int i = 0; i < 4; ++i) subBuf[i] = bufA[i] - bufC[i];

	returnQ.assign(subBuf);

	return returnQ;
}

Quat Quat::mul(const Quat q2) {
	Quat returnQuat;
	returnQuat.w = w * q2.w - (x * q2.x + y * q2.y + z * q2.z);
	returnQuat.x = x * q2.w + w * q2.x - z * q2.y + y * q2.z;
	returnQuat.y = y * q2.w + z * q2.x + w * q2.y - x * q2.z;
	returnQuat.z = z * q2.w - y * q2.x + x * q2.y + w * q2.z;
	return returnQuat;
}

Quat Quat::mulReal(const double s) {
	Quat returnQ;

	double bufA[4] = {};
	Quat2array(bufA);

	double mulBuf[4] = {};

	for (int i = 0; i < 4; ++i) mulBuf[i] = bufA[i] * s;

	returnQ.assign(mulBuf);

	return returnQ;
}

Quat Quat::divReal(const double s) {
	Quat returnQ;

	double bufA[4] = {};
	Quat2array(bufA);

	double divBuf[4] = {};

	for (int i = 0; i < 4; ++i) divBuf[i] = bufA[i] / s;

	returnQ.assign(divBuf);

	return returnQ;
}

//共役
Quat Quat::conjugate() {
	Quat returnQuat;
	returnQuat.w = w;
	returnQuat.x = x * (-1);
	returnQuat.y = y * (-1);
	returnQuat.z = z * (-1);
	return returnQuat;
}

//normalize(正規化)
void Quat::normalize() {
	double n = norm();

	w /= n;
	x /= n;
	y /= n;
	z /= n;
}

//ノルム
double Quat::norm() {
	double xx = x * x;
	double yy = y * y;
	double zz = z * z;
	double ww = w * w;
	return pow(ww + xx + yy + zz, 0.5);
}

//内積
double Quat::dot(Quat dotQ) {
	double bufA[4] = {};
	double bufB[4] = {};

	Quat2array(bufA);
	dotQ.Quat2array(bufB);

	double bufDot = 0.0;

	for (int i = 0; i < 4; ++i) bufDot += bufA[i] * bufB[i];

	return bufDot;
}

//外積
std::vector<double> Quat::cross(Quat crossQ) {
	std::vector<double> returnV;

	double aQ[4] = {};
	double bQ[4] = {};

	Quat2array(aQ);
	crossQ.Quat2array(bQ);

	returnV[0] = aQ[yq] * bQ[zq] - aQ[zq] * bQ[yq];
	returnV[1] = aQ[zq] * bQ[xq] - aQ[xq] * bQ[zq];
	returnV[2] = aQ[xq] * bQ[yq] - aQ[yq] * bQ[xq];

	return returnV;
}

double clamp(double target, double min, double max) {
	double buf = 0;

	if (target < min) {
		buf = target;
	}
	else {
		buf = min;
	}

	if (buf < max) {
		return max;
	}
	else {
		return buf;
	}
}

Quat Quat::slerp(double t, Quat tarQuat) {
	//回転角算出
	/*
	normalize();
	tarQuat.normalize();

	double dotResult = dot(tarQuat);

	const double DOT_THRESHOLD = 0.9995;

	if (fabs(dotResult) > DOT_THRESHOLD) {
		Quat buf1 = sub(tarQuat);
		Quat buf2 = buf1.mulReal(t);
		return sub(buf2);
	}

	Quat minusTarQuat;

	if (dotResult < 0.0f) {
		minusTarQuat = tarQuat.mulReal(-1);
		dotResult = -1 * dotResult;
	}

	clamp(dotResult, -1, 1);
	
	double theta_0 = acos(dotResult);
	double theta = theta_0 * t;

	Quat altQuat = tarQuat.sub(mulReal(dotResult));
	altQuat.normalize();

	Quat returnQ = mulReal(cos(theta));

	return returnQ.add(altQuat.mulReal(sin(theta)));
	*/
	double curNorm = norm();
	double tarNorm = tarQuat.norm();
	double rotValBuf = dot(tarQuat) / (curNorm * tarNorm);
	double rotValue = acosf(rotValBuf);

	//補間
	double sin_rV = sinf(rotValue);

	double kx = sin((1 - t) * rotValue) / sin_rV;
	double ky = sin(t * rotValue) / sin_rV;

	//補間値　Z　計算
	Quat kxX = mulReal(kx);
	Quat kyY = tarQuat.mulReal(ky);

	return kxX.add(kyY);
}

//display
void Quat::display() {
	printf("w -> %3.6lf\n", w);
	printf("x -> %3.6lf\n", x);
	printf("y -> %3.6lf\n", y);
	printf("z -> %3.6lf\n", z);
	DebagBar();
}

//クォータニオンから回転行列を返す
void Quat::quat2RotM(double *rotMat) {
	//display();

	normalize();
	
	//display();

	double xx = x * x;
	double yy = y * y;
	double zz = z * z;
	double ww = w * w;
	double xy = x * y;
	double yz = y * z;
	double zx = z * x;
	double xw = x * w;
	double yw = y * w;
	double zw = z * w;

	double invs = 1 / (xx + yy + zz + ww);

	rotMat[4 * 0 + 0] = 1.0 - 2 * (yy + zz) * invs;
	rotMat[4 * 0 + 1] = 2.0 * (xy - zw) * invs;
	rotMat[4 * 0 + 2] = 2.0 * (zx + yw);
	rotMat[4 * 0 + 3] = 0;

	rotMat[4 * 1 + 0] = 2.0 * (xy + zw);
	rotMat[4 * 1 + 1] = 1.0 - 2 * (xx + zz) * invs;
	rotMat[4 * 1 + 2] = 2.0 * (yz - xw);
	rotMat[4 * 1 + 3] = 0;

	rotMat[4 * 2 + 0] = 2.0 * (zx - yw);
	rotMat[4 * 2 + 1] = 2.0 * (yz + xw);
	rotMat[4 * 2 + 2] = 1.0 - 2 * (xx + yy) * invs;
	rotMat[4 * 2 + 3] = 0;

	rotMat[4 * 3 + 0] = 0;
	rotMat[4 * 3 + 1] = 0;
	rotMat[4 * 3 + 2] = 0;
	rotMat[4 * 3 + 3] = 1.0;


	/*
	rotMat[4 * 0 + 0] = (ww + xx - yy - zz);
	rotMat[4 * 0 + 1] = 2.0 * (xy - zw);
	rotMat[4 * 0 + 2] = 2.0 * (zx + yw);
	rotMat[4 * 0 + 3] = 0;

	rotMat[4 * 1 + 0] = 2.0 * (xy + zw);
	rotMat[4 * 1 + 1] = (ww - xx + yy - zz);
	rotMat[4 * 1 + 2] = 2.0 * (yz - xw);
	rotMat[4 * 1 + 3] = 0;

	rotMat[4 * 2 + 0] = 2.0 * (zx - yw);
	rotMat[4 * 2 + 1] = 2.0 * (yz + xw);
	rotMat[4 * 2 + 2] = (ww - xx - yy + zz);
	rotMat[4 * 2 + 3] = 0;

	rotMat[4 * 3 + 0] = 0;
	rotMat[4 * 3 + 1] = 0;
	rotMat[4 * 3 + 2] = 0;
	rotMat[4 * 3 + 3] = 1.0;
	*/
}

/*
void Quat::quat2Euler(double *euler) {
	//下のサイトを参考に，運動学計算を行うに当たって
	//楽なように書き直してます．
	//euler角をφ，θ，ψの順に書き出したいのです．
	//jacobian の計算のときに与える速度vが,v = {x,y,z,φ，θ，ψ}だから
	//http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
	//https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

	double xsqr = x * x;
	double ysqr = y * y;
	double zsqr = z * z;
	double wsqr = w * w;

	// roll (x-axis rotation)
	double t0 = 2.0 * (w * x + y * z);
	double t1 = +1.0 - 2.0 * (xsqr + ysqr);

	euler[0] = std::atan2(t0, t1);	//roll

	// pitch (y-axis rotation)
	double t2 = +2.0 * (w * y - z * x);
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	euler[1] = std::asin(t2);	//pitch

	// yaw (z-axis rotation)
	double t3 = +2.0 * (w * z + x * y);
	double t4 = +1.0 - 2.0 * (ysqr + zsqr);
	euler[2] = std::atan2(t3, t4);	//yaw
}
*/

bool isRotationMatrix(const double *m) {
	//https://www.learnopencv.com/rotation-matrix-to-euler-angles/ 

	double rotMat[9] = {};
	double rotMatT[9] = {};

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			rotMat[3 * i + j] = m[4 * i + j];
		}
	}

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			rotMatT[3 * i + j] = m[4 * j + i];
		}
	}

	double shouldBeIdentity[9] = {};

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			for (int k = 0; k < 3; ++k) {
				shouldBeIdentity[3 * i + j] += rotMatT[3 * i + k] * rotMat[3 * k + j];
			}
		}
	}

	double normBuf = 0.0f;

	for (int i = 0; i < 3; ++i) {
		normBuf += pow(shouldBeIdentity[3 * i + i], 2);
	}

	double norm = 0.0f;

	norm = pow(normBuf, 0.5);

	if (fabs(norm) > 1e-6) {
		return true;
	}
	else {
		return false;
	}
}

//同次変換行列を渡してオイラー角を得る
void RotMat2Euler(const double *rotMat, double *euler) {
	//https://www.learnopencv.com/rotation-matrix-to-euler-angles/
	if (isRotationMatrix(rotMat) == true) {
		//printf(" valid matrix \n");
	}
	else {
		printf("ERROR : RotMatrixToEuler\n");
		printf("*** invalid matrix ***\n");
	}

	double sy = pow(pow(rotMat[4 * 0 + 0], 2) + pow(rotMat[4 * 1 + 0], 2), 0.5);

	if (sy < 1e-6) {
		euler[0] = atan2(-rotMat[4 * 1 + 2], rotMat[4 * 1 + 1]);
		euler[1] = atan2(rotMat[4 * 2 + 0], sy);
		euler[2] = 0;
	}
	else {
		euler[0] = atan2(rotMat[4 * 2 + 1], rotMat[4 * 2 + 2]);
		euler[1] = atan2(-rotMat[4 * 2 + 0], sy);
		euler[2] = atan2(rotMat[4 * 1 + 0], rotMat[4 * 0 + 0]);
	}
}

void RotMat2EulerZXZ(double *m, double *euler) {
	double buf = pow(m[4 * 0 + 2], 2) + pow(m[4 * 1 + 2], 2);

	euler[0] = atan2(m[4 * 0 + 2], m[4 * 1 + 2]);
	euler[1] = acos(m[4 * 2 + 2]);
	euler[2] = atan2(m[4 * 2 + 0], -m[4 * 2 + 1]);
}

//zyx
void RotMat2EulerZYX(const double *rotMat, double *euler) {

	if (rotMat[4 * 0 + 2] < -0.998) {
		euler[0] = M_PI / 2;
		euler[1] = 0;
		euler[2] = euler[0] + atan2(rotMat[4 * 1 + 0], rotMat[4 * 2 + 0]);
		return;
	}
	else if (rotMat[4 * 0 + 2] > 0.998) {
		euler[0] = M_PI / 2;
		euler[1] = 0;
		euler[2] = -euler[0] + atan2(-1 * rotMat[4 * 1 + 0], -1 * rotMat[4 * 2 + 0]);
		return;
	}
	else {
		double x1 = -asin(rotMat[4 * 0 + 2]);
		double x2 = M_PI - x1;

		double y1 = atan2(rotMat[4 * 1 + 2] / cos(x1), rotMat[4 * 2 + 2] / cos(x1));
		double y2 = atan2(rotMat[4 * 1 + 2] / cos(x2), rotMat[4 * 2 + 2] / cos(x2));

		double z1 = atan2(rotMat[4 * 0 + 1] / cos(x1), rotMat[4 * 0 + 0] / cos(x1));
		double z2 = atan2(rotMat[4 * 0 + 1] / cos(x2), rotMat[4 * 0 + 0] / cos(x2));

		if (fabs(x1) + fabs(y1) + fabs(z1) <= fabs(x2) + fabs(y2) + fabs(z2)) {
			euler[0] = x1;
			euler[1] = y1;
			euler[2] = z1;
		}
		else {
			euler[0] = x2;
			euler[1] = y2;
			euler[2] = z2;
		}
	}
	//euler[0] = atan2(rotMat[4 * 1 + 2], rotMat[4 * 2 + 2]);
	//euler[1] = asin(-1 * rotMat[4 * 0 + 2]);
	//euler[2] = atan2(rotMat[4 * 0 + 1], rotMat[4 * 0 + 0]);
}

//zxzのオイラー角変換zxz
void Quat::quat2Euler(double *euler) {
	double bufMat[16] = {};
	
	quat2RotM(bufMat);

	//DebagComment("quat2rotMat");	DisplayRegularMatrix(4, bufMat);

	double bufEuler[3] = {};

	RotMat2EulerZXZ(bufMat, bufEuler);

	//RotMat2EulerZYX(bufMat, bufEuler);

	//DebagComment("rotMat2euler");	DisplayVector(3, bufEuler);

	euler[0] = bufEuler[0];
	euler[1] = bufEuler[1];
	euler[2] = bufEuler[2];

	//zyxのとき
	//euler[0] = bufEuler[0];//1	z
	//euler[1] = bufEuler[1];//2	x
	//euler[2] = bufEuler[2];//0	z
}

/*
//xyz
void Quat::quat2Euler(double *euler) {
	//回転行列を得る。
	double m[16] = {};
	quat2RotM(m);
	RotMatrixToEuler(m, euler);
}
*/

void NormalizeQuatMatrix(double *m) {
	double a[3] = { m[0],m[4],m[8] };
	double b[3] = { m[1],m[5],m[9] };
	double c[3] = { m[2],m[6],m[10] };

	double bufc = 0.0;

	bufc = InnerVector(c, c);
	
	for (int i = 0; i < 3; ++i) {
		c[i] = c[i] / bufc;
	}

	double bufa[3] = {};



	



}

//quatanion rotatation
/*
Quat QuatRotate(const std::vector<double> &curPosi, const std::vector<double> &rotAxis,double rotValue) {
	int count = 0;
	
	for (int i = 0; i < 3; ++i) if (curPosi[i] == rotAxis[i]) ++count;

	if(count == 3){
		printf("===========quat zero start===========\n");
		Quat temp(0, 0, 0, 0);
		return temp;
	}
	else {
		printf("===========rotate start===========\n");
		double cos_q, sin_q;

		cos_q = cos(rotValue / 2);
		sin_q = sin(rotValue / 2);

		//printf("cos -> %lf\tsin -> %lf\n", cos_q, sin_q);

		Quat currentQuat(0, curPosi[0], curPosi[1], curPosi[2]);
		Quat rotQuat(cos_q, rotAxis[0] * sin_q, rotAxis[1] * sin_q, rotAxis[2] * sin_q);
		Quat invRotQuat;
		Quat tempQuat;

		rotQuat.normalize();

		invRotQuat = rotQuat.conjugate();

		tempQuat = invRotQuat.mul(currentQuat);

		//printf("rotQuat\n");
		//DisplayQuat(rotQuat);

		//printf("invRotQuat\n");
		//DisplayQuat(invRotQuat);
		return tempQuat.mul(rotQuat);
	}
}
*/


/*
//sample program
void QuatRotateSample() {
std::vector<double> currentPosition{ 10, 0, 0 };
std::vector<double> rotateAxis{ 0, 0, 1 };
double rotateValue = M_PI / 2;
Quat rotateQuatenion;

rotateQuatenion = QuatRotate(currentPosition, rotateAxis, rotateValue);

printf("rotate Quatanion\n");
DisplayQuat(rotateQuatenion);

printf("finished sample program\n");
}
*/