#include <stdio.h>
#include <windows.h>
#include "calculation_win.h"

#define THREAD_NUM 2
#define DATA_NUM 100
#define SPLIT_DATA_NUM (DATA_NUM / THREAD_NUM)
typedef struct _thread_arg {
	int thread_no;
	int *data;
}thread_arg_t;

void thread_func(void *arg) {
	thread_arg_t* targ = (thread_arg_t *)arg;
	int i;
	/*�Q�e�X���b�h�ɕ�������A�^�f�[�^�̏���*/
	/*�����f�[�^�̒l�Ƃ��̒l�ɂP���������l���o��*/
	for (i = 0; i < SPLIT_DATA_NUM; i++) {
		printf("thread%d : %d + 1 = %d\n", targ->thread_no, targ->data[i], targ->data[i] + 1);
	}
}

int main() {
	HANDLE handle[THREAD_NUM];
	thread_arg_t tArg[THREAD_NUM];
	int data[DATA_NUM];
	int i;

	/*�f�[�^�̏�����*/
	for (i = 0; i < DATA_NUM; i++) {
		data[i] = i;
	}

	for (i = 0; i < THREAD_NUM; i++) {
		tArg[i].thread_no = i;
		/*�P�e�X���b�h�ւ̃f�[�^����*/
		tArg[i].data = &data[SPLIT_DATA_NUM * i];
		handle[i] = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)thread_func, (void *)&tArg[i], 0, NULL);
	}
	/*�X���b�h�̏I����҂�*/
	WaitForMultipleObjects(THREAD_NUM, handle, true, INFINITE);

	return 0;
}