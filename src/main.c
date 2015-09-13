#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include "dcm.h"
#include "lmath.h"
#include "lsm9ds0/lsm9ds0.h"
#include "constants.h"

int main(int argc, char **argv) {
	double mag_raw[3], mag_scaled[3], heading;

	const struct timespec sleep = {0, 2500000L};

	init_imu("/dev/i2c-1", 0x1D, 0x6B);

	if(argc > 1 && strncmp(argv[1], "--reg", 5) == 0) {
		dump_registers();
		goto done;
	}

	printf("\n");

	while(1) {
		read_raw_mag(mag_raw);

		vector_scale3(mag_raw, mag_scale, mag_scaled);

		heading = DEG(atan2(mag_scaled[1], mag_scaled[0]));

		if(heading < 0) heading += 360.0;
		if(heading > 360) heading -= 360.0;

		printf("Mag Raw: %7.3f %7.3f %7.3f %7.3f\r", mag_scaled[0],
				mag_scaled[1], mag_scaled[2], heading);

		nanosleep(&sleep, NULL);
	}

done:
	exit(EXIT_SUCCESS);
}
