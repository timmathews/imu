#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include "dcm.h"
#include "lmath.h"
#include "lsm9ds0/lsm9ds0.h"
#include "constants.h"

struct timespec _start_time;

uint64_t millis() {
        struct timespec ts;

        clock_gettime(CLOCK_MONOTONIC_RAW, &ts);

        return 1.0e3 * (
                (ts.tv_sec + (ts.tv_nsec * 1.0e-9)) -
                (_start_time.tv_sec + (_start_time.tv_nsec * 1.0e-9))
        );

}

int main(int argc, char **argv) {
	double mag_raw[3], mag_scaled[3], euler[3], heading;
        uint64_t time_start, time_elapsed;
	const struct timespec sleep = {0, 2500000L};

	init_imu("/dev/i2c-1", 0x1D, 0x6B);

	if(argc > 1 && strncmp(argv[1], "--reg", 5) == 0) {
		dump_registers();
		goto done;
	}

	printf("\n");

	clock_gettime(CLOCK_MONOTONIC_RAW, &_start_time);
	time_start = millis();

	while(1) {
		time_elapsed = millis();

                if(time_elapsed - time_start > 20) {
			read_raw_mag(mag_raw);

			update_matrix(read_raw_acc, read_raw_gyr);
			euler_angles(euler);

			vector_scale3(mag_raw, mag_scale, mag_scaled);

			heading = DEG(atan2(mag_scaled[1], mag_scaled[0]));

			if(heading < 0) heading += 360.0;
			if(heading > 360) heading -= 360.0;

			printf("Mag Raw: %7.3f %7.3f %7.3f %7.3f\n", mag_scaled[0],
					mag_scaled[1], mag_scaled[2], heading);
			printf("Euler: %7.3f %7.3f %7.3f\n", euler[0],
					euler[1], euler[2]);
			printf("\n");
		}

		nanosleep(&sleep, NULL);
	}

done:
	exit(EXIT_SUCCESS);
}
