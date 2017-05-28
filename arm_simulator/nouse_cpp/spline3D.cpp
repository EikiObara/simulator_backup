
const int MAX_SPLINE_SIZE = 100;
const int PLUS_NULL = 1;

class Spline {
	int num;
	double a[MAX_SPLINE_SIZE + PLUS_NULL];
	double b[MAX_SPLINE_SIZE + PLUS_NULL];
	double c[MAX_SPLINE_SIZE + PLUS_NULL];
	double d[MAX_SPLINE_SIZE + PLUS_NULL];
public:
	Spline() { num = 0; }
	void init(double *sp, int num);
	double culc(double t);
};

