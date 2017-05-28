#include <iostream>
#include <Eigen\Dense>

#define _TCHAR char
#define _tmain main

int _tmain(int argc, _TCHAR* argv[])
{
	Eigen::MatrixXd A(9, 9);
	std::cout << A;
	return 0;
}
