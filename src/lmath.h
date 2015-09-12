#ifndef _LMATH_H
#define _LMATH_H

void matrix_multiply3(double a[3][3], double b[3][3], double m[3][3]);
double vector_dot_product3(double v1[3], double v2[3]);
void vector_cross_product3(double a[3], double b[3], double m[3]);
void vector_scale3(double a[3], double s, double m[3]);
void vector_add3(double a[3], double b[3], double m[3]);

#endif
