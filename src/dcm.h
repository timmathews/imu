#ifndef _DCM_H
#define _DCM_H

#define GRAVITY (100) // TODO: Look up in datasheet

/* WTF do these come from? */
#define Kp_ROLLPITCH 0.015
#define Ki_ROLLPITCH 0.000010
#define Kp_YAW 1.2
#define Ki_YAW 0.00005

#define scale_acc(x) (x*(GRAVITY/9.81))

#define RAD(x) (x*0.01745329252) // pi/180
#define DEG(x) (x*57.2957795131) // 180/pi

#define GYRO_GAIN_X 1 // TODO: Look these up in datasheet
#define GYRO_GAIN_Y 1
#define GYRO_GAIN_Z 1
#define gyro_scaled_x(x) x*RAD(GYRO_GAIN_X)
#define gyro_scaled_y(x) x*RAD(GYRO_GAIN_Y)
#define gyro_scaled_z(x) x*RAD(GYRO_GAIN_Z)

#define G_dt 0.02

int renorm(double *in);
void normalize(void);
void correct_drift(double heading_x, double heading_y);
void adjust_accel(double ground_speed);
void update_matrix(double (*read_adc)(int));
void euler_angles(double *m);
double constrain(double x, double min, double max);

#endif
