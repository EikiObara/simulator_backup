// 2017/04/28
// created by eiki obara
// reference : numerical recicpes in c

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "include\kine_debag.h"

const int PLUS_NULL = 1;

double *Dvector(long nh) {

	double *v = (double *)malloc(nh*sizeof(double));

	if (!v)	printf("malloc failure in vector()\n");

	return v + PLUS_NULL;
}

int *Ivector(long nh) {

	int *v = (int *)malloc(nh*sizeof(int));

	if (!v)	printf("malloc failure in vector()\n");

	return v + PLUS_NULL;
}

void Free_dvector(double *v) {
	//printf("free_vector started\n");
	free(v);
}

void Free_ivector(int *v) {
	//printf("free_vector started\n");
	free(v);
}

int Ludcmp(std::vector<double> &mat, int matSize, int *index, double *d) {
	int i, j, jmax = 0, k;
	double big, dum, sum, temp;
	double *vv;
	double TINY = 1.0e-20;

	vv = Dvector(matSize);
	*d = 1.0;

	///スケーリング情報
	for (i = 0; i < matSize; i++) {
		big = 0.0;
		for (j = 0; j < matSize; j++) {
			if ((temp = fabs(mat[matSize*i + j]))>big) { big = temp; }
		}
		if (big == 0.0) {
			ErrComment("Singular matrix in routine ludcmp");
			return 1;
		}
		vv[i] = 1.0 / big;
	}
	///////////////////////////////////////////


	//Crout方を用いる．列についてのループ
	for (i = 0; i < matSize; i++) {
		for (j = 0; j < i; j++) {
			sum = mat[matSize * j + i];
			for (k = 0; k < j; k++) {
				sum -= mat[matSize * j + k] * mat[matSize * k + i];
			}
			mat[j * matSize + i] = sum;
		}
		big = 0.0;

		for (j = i; j < matSize; j++) {
			sum = mat[matSize*j + i];
			for (k = 0; k < i; k++) {
				sum -= mat[matSize*j + k] * mat[matSize * k + i];
			}

			mat[matSize * j + i] = sum;
			dum = vv[j] * fabs(sum);

			if (dum >= big) {
				big = dum;
				jmax = j;
			}
		}
		if (i != jmax) {
			for (j = 0; j < matSize; j++) {
				dum = mat[matSize * jmax + j];
				mat[matSize * jmax + j] = mat[matSize * i + j];
				mat[matSize * i + j] = dum;
			}
			*d = -(*d);
			vv[jmax] = vv[i];
		}

		index[i] = jmax;

		if (mat[matSize * i + i] == 0.0) {
			mat[matSize * i + i] = TINY;
			ErrComment("Matrix:: ludcmp : tiny");
			exit(1);
		}

		if (i != (matSize - 1)) {
			dum = 1.0 / (mat[matSize*i + i]);
			for (j = i + 1; j < matSize; j++) {
				mat[matSize * j + i] *= dum;
			}
		}
	}
	
	//free(vv);

	return 0;

}

void Lubksb(std::vector<double> &mat, int matSize, int *index, double *b) {
	int i, ii = -1, ip, j;
	double sum;

	for (i = 0; i < matSize; i++) {
		ip = index[i];
		sum = b[ip];
		b[ip] = b[i];
		if (ii != -1) {
			for (j = ii; j < i; j++) {
				sum -= mat[matSize*i + j] * b[j];
			}
		}
		else if (sum != 0.0) {
			ii = i;
		}
		b[i] = sum;
	}

	for (i = (matSize - 1); i >= 0; i--) {
		sum = b[i];
		for (j = i + 1; j < matSize; j++) {
			sum -= mat[i*matSize + j] * b[j];
		}
		b[i] = sum / mat[i*matSize + i];
	}

}
