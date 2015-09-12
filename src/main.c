#include <stdio.h>
#include <stdlib.h>
#include "dcm.h"
#include "lsm9ds0/lsm9ds0.h"

int main(int argc, char **argv) {
	uint8_t gyr_raw[6];
	uint8_t acc_raw[6];
	uint8_t mag_raw[6];

	init_imu("/dev/i2c-1", 0x1D, 0x6B);

	dump_registers();

	exit(EXIT_SUCCESS);
}
