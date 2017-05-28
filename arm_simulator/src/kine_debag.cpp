// 2017/04/28
// created by eiki obara

#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <time.h>
#include <string.h>
#include <fstream>
#include "include\kinematics.h"
#include "include\kine_debag.h"
#include "include\kine_vector.h"
#include "include\kine_config.h"

int DEBAG = 0;

void kinemaDebagON(void) {
	DEBAG = 1;
	DebagComment("kinemaDebag On");
}

void kinemaDebagOFF(void) {
	DebagComment("kinemaDebag OFF");
	DEBAG = 0;
}

//デバッグ用横線
void DebagBar(void) {
	printf("\n-----------------------------------------------------\n");
}

// comment for debag
void DebagComment(char *c) {
	if (DEBAG == 1) {
		DebagBar();
		printf("%s\n", c);
	}
}

void DebagCommentEnt(char *c) {
	if (DEBAG == 1) {
		DebagBar();
		printf("%s : push ENTER\n", c);
		DebagBar();
		PushEnter();
	}
}

void ErrComment(char *c) {
	DebagBar();
	printf("Error Comment\n%s\n", c);
	printf(" : push ENTER : \n");
	DebagBar();
	PushEnter();
}

void DisplayVector(const int n, const double *v) {
	if (DEBAG == 1) {
	//printf("display Vector\n");
	for (int i = 0; i < n; ++i) {
		printf("v[%d]-> %3.8lf\n", i, v[i]);
	}
	printf("\n");
	}
}

void DisplayVector(const int n, const float *v) {
	if (DEBAG == 1) {
		//printf("display Vector\n");
		for (int i = 0; i < n; ++i) {
			printf("v[%d]-> %3.8lf\n", i, v[i]);
		}
		printf("\n");
	}
}

void DisplayRegularMatrix(const int n, double *mat) {
	if (DEBAG == 1) {
		//printf("display Matrix \n");
		for (int i = 0; i < n; ++i) {
			for (int j = 0; j < n; ++j) {
				printf("%3.6lf\t", mat[n * i + j]);
			}
			printf("\n");
		}
		printf("\n");
	}
}

void OutputMatrixTxt(double *m, const int column, double loopnum, char *filename) {
	std::ofstream fout;
	time_t 	t = time(NULL);
	struct tm *pnow = localtime(&t);

	char name[200] = "";
	char timename[100] = "";
	
	sprintf(timename, "%04d%02d%02d%02d%02d%02d",
		pnow->tm_year + 1900, pnow->tm_mon + 1, pnow->tm_mday,
		pnow->tm_hour, pnow->tm_min, pnow->tm_sec);

	strcat(name, filename);
	strcat(name, timename);
	strcat(name, ".txt");

	printf("\n output to - %s - file \n", name);

	fout.open(name);
	if (!fout) {
		printf("%s\n", filename);
		ErrComment("file open error");
		fout.close();
		PushEnter();
		exit(1);
	}

	for (int i = 0; i < loopnum; ++i) {
		if (i < kine::TIME_LENGTH / kine::TIME_SPAN) {
			for (int j = 0; j < column; ++j) {
				fout << m[column * i + j] << "\t";
			}
			fout << "\n";
		}
	}
	fout.close();
	printf("output finished\n");
}

void PushEnter(void) {
	while (getchar() != '\n') continue;
}

