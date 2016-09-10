#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include "dcm.h"
#include "lmath.h"
#include "lsm9ds0/lsm9ds0.h"
#include "constants.h"

#define magXmax 550
#define magYmax 601
#define magZmax 1069
#define magXmin -645
#define magYmin -551
#define magZmin -477

struct timespec _start_time;

static volatile int quit = 0;

/* Zero G accelerometer offsets */
double acc_z[3] = {-606.67, -566.69, 354.84};

/* Accelerometer scale multipliers */
double acc_s[3] = {0.00073455, 0.00073484, 0.00074842};

/* SIGINT (Ctrl-C) Handler */
void intHandler(int d)
{
    quit = 1;
}

/* millis returns the number of milliseconds since the application started.
 * Overflows in about 584 million years, be sure to reboot before then.
 * Call clock_gettime first and store value in global _start_time. This only
 * needs to be done once.
 */
uint64_t millis()
{
        struct timespec ts;

        clock_gettime(CLOCK_MONOTONIC_RAW, &ts);

        return 1.0e3 * (
                (ts.tv_sec + (ts.tv_nsec * 1.0e-9)) -
                (_start_time.tv_sec + (_start_time.tv_nsec * 1.0e-9))
        );

}

/* print_formatted_values outputs a nice chart of the current values the nine
 * axes. For this to work well, the screen should be cleared and cursor moved
 * to home before calling.
 */
void print_formatted_values(double *acc, double *gyr, double *mag, char *title)
{
	printf("              %s\n", title);
	printf("    |     X     |     Y     |     Z\n");
	printf("----+-----------+-----------+-----------\n");
	printf("\033[KAcc | % 9.3f | % 9.3f | % 9.3f\n", acc[0], acc[1], acc[2]);
	printf("\033[KGyr | % 9.3f | % 9.3f | % 9.3f\n", gyr[0], gyr[1], gyr[2]);
	printf("\033[KMag | % 9.3f | % 9.3f | % 9.3f\n", mag[0], mag[1], mag[2]);
	printf("\n");
}

int main(int argc, char **argv)
{
        uint64_t time_start, time_elapsed;
	const struct timespec sleep = {0, 25000000L};
	double acc[3], gyr[3], mag[3], mx, my, mz, hdg, pitch, roll;
	int use_json = 0;

	/* Register interrupt handler */
	signal(SIGINT, intHandler);

	/* Configure the IMU chip; currently on i2c bus 1, addresses 0x1d and
	 * 0x6b. TODO: Move these to config file.
	 */
	init_imu("/dev/i2c-1", 0x1D, 0x6B);

	/* imu currently has one optional flag `--reg` which instructs the
	 * program to dump the current register values from the IMU and exit.
	 */
	if(argc > 1 && strncmp(argv[1], "--reg", 5) == 0) {
		dump_registers();
		goto done;
	}

	if(argc > 1 && strncmp(argv[1], "-j", 2) == 0) {
		use_json = 1;
	}

	/* Timer setup. Record start time. */
	clock_gettime(CLOCK_MONOTONIC_RAW, &_start_time);
	time_start = millis();

	/* Main loop. SIGINT interrupt handler sets global quit to 1, breaking
	 * the loop and exiting eventually.
	 */
	while(quit == 0) {
		time_elapsed = millis();

		/* Approximate check to see if it's been 250 milliseconds since
		 * our last measurement. If so, reread everything, otherwise
		 * continue to sleep.
		 */
                if(time_elapsed - time_start > 250) {
			time_start = time_elapsed;

			read_raw_acc(acc); /* Read accelerometer */
			read_raw_gyr(gyr); /* Read gyroscope */
			read_raw_mag(mag); /* Read magnetometer */

			if(!use_json) {
				/* Clear screen, move to 0,0 */
				printf("\033[2J\033[H");

				print_formatted_values(acc, gyr, mag, "Raw Values");
			}

			/* Apply corrections to accelerometer */
			vector_subtract3(acc, acc_z, acc);
			vector_multiply3(acc, acc_s, acc);

			pitch = asin(acc[0]); // arcsin(Ax1)
			roll = asin(acc[1] / -cos(pitch));

			/* Hard iron correction */
			mag[0] -= (magXmin + magXmax) / 2;
			mag[1] -= (magYmin + magYmax) / 2;
			mag[2] -= (magZmin + magZmax) / 2;

			mx = (mag[0] * cos(pitch)) + (mag[2] * sin(pitch));
			my = (mag[0] * sin(roll) * sin(pitch)) +
			     (mag[1] * cos(roll)) -
			     (mag[2] * sin(roll) * cos(pitch));

			hdg = DEG(atan2(my, mx));

			if(hdg < 0) {
				hdg += 360;
			}

			/* TODO: Apply corrections to other components */

			if(use_json) {
				/* Output JSON */
				printf("{\"acc\":{\"x\":%f,\"y\":%f,\"z\":%f},"
				       "{\"gyr\":{\"x\":%f,\"y\":%f,\"z\":%f},"
				       "{\"mag\":{\"x\":%f,\"y\":%f,\"z\":%f},"
				       "\"heading\":%f,\"pitch\":%f,\"roll\":%f}\n",
				       acc[0],acc[1],acc[2], gyr[0],gyr[1],gyr[2],
				       mag[0],mag[1],mag[2], RAD(hdg), pitch, roll);
			} else {
				print_formatted_values(acc, gyr, mag, "Scaled Values");
				printf("X/Y:     % 9.3f | % 9.3f\n", mx, my);
				printf("Heading: % 6.1f\n", hdg);
				printf("Pitch:   % 6.1f\n", DEG(pitch));
				printf("Roll:    % 6.1f\n", DEG(roll));
			}
		}

		nanosleep(&sleep, NULL);
	}

	if(!use_json) {
		/* Move the cursor to line 17. Add 7 for every call to
		 * print_formatted_values.
		 */
		printf("\033[17;0H\n");
	}

done:
	exit(EXIT_SUCCESS);
}
