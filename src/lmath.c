#include "lmath.h"

void matrix_multiply3(double a[3][3], double b[3][3], double m[3][3])
{
	double op[3];

	for(int i=0; i<3; ++i) {
		for(int j=0; j<3; ++j) {
			for(int k=0; k<3; ++k) {
				op[k] = a[i][k] * b[k][j];
			}

			m[i][j] = op[0] + op[1] + op[2];
		}
	}
}

void matrix_add3(double a[3][3], double b[3][3], double m[3][3])
{
	for(int i=0; i<3; ++i) {
		for(int j=0; j<3; ++j) {
			m[i][j] = a[i][j] + b[i][j];
		}
	}
}

double vector_dot_product3(double a[3], double b[3])
{
	double m = 0;

	for(int i = 0; i<3; ++i) {
		m+= a[i] * b[i];
	}

	return m;
}

void vector_cross_product3(double a[3], double b[3], double m[3])
{
	m[0] = (a[1] * b[2]) - (a[2] * b[1]);
	m[1] = (a[2] * b[0]) - (a[0] * b[2]);
	m[2] = (a[0] * b[1]) - (a[1] * b[0]);
}

void vector_scale3(double a[3], double s, double m[3])
{
	for(int i = 0; i<3; ++i) {
		m[i] = a[i] * s;
	}
}

void vector_add3(double a[3], double b[3], double m[3])
{
	for(int i = 0; i<3; ++i) {
		m[i] = a[i] + b[i];
	}
}

void vector_subtract3(double a[3], double b[3], double m[3])
{
	for(int i = 0; i<3; ++i) {
		m[i] = a[i] - b[i];
	}
}

void vector_multiply3(double a[3], double b[3], double m[3])
{
	for(int i = 0; i<3; ++i) {
		m[i] = a[i] * b[i];
	}
}
