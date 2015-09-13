#include <stdio.h>
#include <stdlib.h>
#include "dcm.h"
#include "lsm9ds0/lsm9ds0.h"

int main(int argc, char **argv) {
	double mag_raw[3];

	init_imu("/dev/i2c-1", 0x1D, 0x6B);

	dump_registers();

	while(1) {
		read_raw_mag(&mag_raw);
		printf("Mag Raw: %7.3f %7.3f %7.3f\r", mag_raw[0], mag_raw[1],
				mag_raw[2]);
		usleep(25000);
	}

	exit(EXIT_SUCCESS);
}
