#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "dcm.h"
#include "lmath.h"
#include "constants.h"

double dcm_matrix[3][3] = {
	{1, 0, 0},
	{0, 1, 0},
	{0, 0, 1}
};

double out_matrix[3][3] = {
	{0, 1, 2},
	{3, 4, 5},
	{6, 7, 8}
};

double temp[3][3] = {{0}};

double acc_vector[3] = {0};
double gyro_vector[3] = {0};
double omega_vector[3] = {0};
double omega_i[3] = {0};
double omega_p[3] = {0};
double omega[3] = {0};
double roll_pitch_error[3] = {0};
double yaw_error[3] = {0};
double course_error = 180.0;

void print_matrix(double m[3][3], char *title) {
	int i = 0;

	printf("%s\n", title);

	for(;i<3;++i) {
		printf("%7.3f %7.3f %7.3f\n\n", m[i][0], m[i][1], m[i][2]);
	}
}

void print_vector(double m[3], char *title) {
	printf("%s\n", title);
	printf("%7.3f %7.3f %7.3f\n\n", m[0], m[1], m[2]);
}

int renorm(double *in) {
	if(*in < 1.5625 && *in > 0.64) { /* TODO: WTF are these magic numbers?
					  */
		*in = 0.5 * (3 - *in); /* TODO: Again, WTF!? */
	} else if(*in < 100.0 && *in > 0.01) {
		*in = 1.0 / sqrt(*in);
	} else {
		return -1;
	}

	return 0;
}

void normalize(void) {
	double error, normal;
	int reset = 0;
	int i = 0;

	error = -vector_dot_product3(&dcm_matrix[0][0], &dcm_matrix[1][0]) * 0.5;

	vector_scale3(&dcm_matrix[1][0], error, &temp[0][0]);
	vector_scale3(&dcm_matrix[0][0], error, &temp[1][0]);

	vector_add3(&dcm_matrix[0][0], &temp[0][0], &temp[0][0]);
	vector_add3(&dcm_matrix[1][0], &temp[1][0], &temp[1][0]);

	vector_cross_product3(&temp[0][0], &temp[1][0], &temp[2][0]);


	for(; i < 3; ++i) {
		normal = vector_dot_product3(&temp[i][0], &temp[i][0]);

		if(renorm(&normal) < 0) {
			reset = 1;
			goto bailout;
		}

		vector_scale3(&temp[i][0], normal, &dcm_matrix[i][0]);
	}

bailout:
	if (reset) {	/* Our solution is blowing up and we will force back to
			 * initial condition. Hope we are not upside down!
			 */
		dcm_matrix[0][0]= 1.0f;
		dcm_matrix[0][1]= 0.0f;
		dcm_matrix[0][2]= 0.0f;
		dcm_matrix[1][0]= 0.0f;
		dcm_matrix[1][1]= 1.0f;
		dcm_matrix[1][2]= 0.0f;
		dcm_matrix[2][0]= 0.0f;
		dcm_matrix[2][1]= 0.0f;
		dcm_matrix[2][2]= 1.0f;
		fprintf(stderr, "\033[32;1m" "Solution collapsed!\n" "\033[0m");
	}
}

void correct_drift(double heading_x, double heading_y) {
	double acc_magnitude, acc_weight, integrator_magnitude;
	static double scaled_omega_p[3], scaled_omega_i[3];

	/* Roll and Pitch */
	acc_magnitude = sqrt(acc_vector[0] * acc_vector[0] +
			acc_vector[1] * acc_vector[1] +
			acc_vector[2] * acc_vector[2]);

	acc_magnitude /= GRAVITY;

	acc_weight = constrain(1 - 2 * abs(1 - acc_magnitude), 0, 1);

	vector_cross_product3(&roll_pitch_error[0], &acc_vector[0],
			&dcm_matrix[2][0]);

	vector_scale3(&roll_pitch_error[0], Ki_ROLLPITCH * acc_weight,
			&scaled_omega_i[0]);

	vector_add3(omega_i, omega_i, scaled_omega_i);

	/* Calculate yaw error */
	course_error = (dcm_matrix[0][0] * heading_y) - (dcm_matrix[1][0] *
			heading_x);

	/* Applies the yaw correction to the XYZ rotation of the sensor */
	vector_scale3(dcm_matrix[2], course_error, yaw_error);

	vector_scale3(yaw_error, Kp_YAW, scaled_omega_p);

	/* Add proportional component */
	vector_add3(omega_p, scaled_omega_p, omega_p);

	vector_scale3(yaw_error, Ki_YAW, scaled_omega_i);

	/* Add integral component */
	vector_add3(omega_i, scaled_omega_i, omega_i);

	/* Here we will place a limit on the integrator so that the integrator
	 * cannot ever exceed half the saturation limit of the gyros
	 */
	integrator_magnitude = sqrt(vector_dot_product3(omega_i, omega_i));
	if(integrator_magnitude > RAD(300)) {
		vector_scale3(omega_i, 0.5 * RAD(300) / integrator_magnitude, omega_i);
	}
}

/* Remove centrifugal acceleration
 * centrifugal acceleration on the Y axis = speed over ground x gyro Z axis
 *      "           "       "   "  X  "   "   "    "     "    "  "   Y  "
 */
void adjust_accel(double ground_speed) {
	acc_vector[1] += scale_acc((ground_speed/100) * omega[2]);
	acc_vector[2] -= scale_acc((ground_speed/100) * omega[1]);
}

void update_matrix(int (*read_acc)(double *), int (*read_gyr)(double *)) {
	read_acc(acc_vector);
	read_gyr(gyro_vector);

	memset(omega_vector, 0, sizeof(omega_vector));

	print_vector(acc_vector, "Accelerometer");
	print_vector(gyro_vector, "Gyroscope");

	gyro_vector[0] = gyro_scaled_x(gyro_vector[0]); /* gyro x roll */
	gyro_vector[1] = gyro_scaled_y(gyro_vector[1]); /* gyro y pitch */
	gyro_vector[2] = gyro_scaled_z(gyro_vector[2]); /* gyro Z yaw */

	print_vector(gyro_vector, "Gyroscope Scaled");

	vector_add3(gyro_vector, omega_i, omega); /* add proportional component
						   */
	print_vector(omega, "Gyroscope + Proportional");

	vector_add3(omega, omega_p, omega_vector); /* add integral term */

	print_vector(omega_vector, "Gyroscope + Proportional + Integral");

	/* TODO: Inject ground speed */
	adjust_accel(0); /* remove centrifugal acceleration */

	out_matrix[0][0] = 0;
	out_matrix[0][1] = -G_dt * omega_vector[2]; /* -z */
	out_matrix[0][2] =  G_dt * omega_vector[1]; /*  y */
	out_matrix[1][0] =  G_dt * omega_vector[2]; /*  z */
	out_matrix[1][1] = 0;
	out_matrix[1][2] = -G_dt * omega_vector[0]; /* -x */
	out_matrix[2][0] = -G_dt * omega_vector[1]; /* -y */
	out_matrix[2][1] =  G_dt * omega_vector[0]; /*  x */
	out_matrix[2][2] = 0;

	print_matrix(out_matrix, "Output");

	matrix_multiply3(dcm_matrix, out_matrix, temp); /* a*b=c */

	print_matrix(temp, "Output + DCM");

	matrix_add3(dcm_matrix, temp, dcm_matrix);

	print_matrix(dcm_matrix, "Output + DCM + DCM");
}

void euler_angles(double *m) {
	if(m == NULL) {
		return;
	}

	m[0] = -asin(dcm_matrix[2][0]);
	m[1] = atan2(dcm_matrix[2][1], dcm_matrix[2][2]);
	m[2] = atan2(dcm_matrix[1][0], dcm_matrix[0][0]);
}

double constrain(double x, double min, double max) {
	if(x < min) return min;
	if(x > max) return max;
	return x;
}
