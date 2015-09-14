/*
 * LSM9DS0 Driver
 *
 * Copyright (C) 2015 Tim Mathews <tim@signalk.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the
 *
 *   Free Software Foundation, Inc.
 *   59 Temple Place
 *   Suite 330
 *   Boston, MA 02111-1307 USA
 */

#include <fcntl.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "lsm9ds0.h"
#include "constants.h"

int fd, _g_addr, _xm_addr;

void select_device(int addr)
{
	if (ioctl(fd, I2C_SLAVE, addr) < 0) {
		printf("Failed to select I2C device.");
	}
}

void write_reg(int addr, uint8_t reg, uint8_t value)
{
	select_device(addr);

	int result = i2c_smbus_write_byte_data(fd, reg, value);
	if (result == -1)
	{
		printf ("Failed to write byte to I2C");
		exit(1);
	}
}

void read_reg(int addr, uint8_t reg, int16_t *a)
{
	uint8_t block[6];
	uint8_t size;

	size = sizeof(block);

	select_device(addr);

	int result = i2c_smbus_read_i2c_block_data(fd, 0x80 | reg,
			size, block);

	if (result != size)
	{
		printf("Failed to read block from I2C.");
		exit(1);
	}

	*a     = (int16_t)(block[0] | block[1] << 8);
	*(a+1) = (int16_t)(block[2] | block[3] << 8);
	*(a+2) = (int16_t)(block[4] | block[5] << 8);
}

void init_imu(char *file, int xm_addr, int g_addr)
{
	_xm_addr = xm_addr;
	_g_addr = g_addr;

	fd = open(file, O_RDWR);
	if (fd<0) {
		printf("Unable to open I2C bus!\n");
		exit(1);
	}

	// Enable accelerometer
	// Set output data rate and enable axes
	write_reg(xm_addr, CTRL_REG1_XM,
		CTRL_REG1_XM_AODR_800Hz |
		CTRL_REG1_XM_BDU |
		CTRL_REG1_XM_AZEN |
		CTRL_REG1_XM_AYEN |
		CTRL_REG1_XM_AXEN);

	// Set AA bandwidth, full scale
	write_reg(xm_addr, CTRL_REG2_XM,
		CTRL_REG2_XM_ABW_50Hz |
		CTRL_REG2_XM_AFS_16G);

	//Enable magnetometer
	// Set temp enable, mag resolution, output data rate
	write_reg(xm_addr, CTRL_REG5_XM,
		CTRL_REG5_XM_TEMP_EN |
		CTRL_REG5_XM_M_RES_HIGH |
		CTRL_REG5_XM_ODR_50Hz);

	// Set magnetometer full scale range
	write_reg(xm_addr, CTRL_REG6_XM, CTRL_REG6_XM_MFS_12Gs);

	// Enable continuous conversion mode
	write_reg(xm_addr, CTRL_REG7_XM, 0);

	// Enable gyro
	// Set output data rate, bandwidth, enable axes
	write_reg(g_addr, CTRL_REG1_G,
		CTRL_REG1_G_DR_760Hz_BW_50Hz |
		CTRL_REG1_G_PD |
		CTRL_REG1_G_ZEN |
		CTRL_REG1_G_YEN |
		CTRL_REG1_G_XEN);

	// Set high-pass filter mode and cutoff
	write_reg(g_addr, CTRL_REG2_G, 0);

	// Set full-scale selection, self-test mode
	write_reg(g_addr, CTRL_REG4_G,
		CTRL_REG4_G_BDU |
		CTRL_REG4_G_FS_2000DPS);

	gyro_scale = 0.07; // for 2000dps from datasheet, s2.1 p13
	gyro_scale = RAD(gyro_scale);

	accel_scale = 0.000732; // for 16g from datasheet, s2.1 p13
	accel_scale *= GRAVITY;

	mag_scale = 0.00048; // for 12 gauss from datasheet, s2.1 p13
}

int read_raw_mag(double *m) {
	int16_t d[3];

	read_reg(_xm_addr, OUT_X_L_M, d);
	
	for(int i=0; i<3; ++i) {
		*(m + i) = *(d + i);
	}

	return 0;
}

int read_raw_acc(double *m) {
	int16_t d[3];

	read_reg(_xm_addr, OUT_X_L_A, d);
	
	for(int i=0; i<3; ++i) {
		*(m + i) = *(d + i);
	}

	return 0;
}

int read_raw_gyr(double *m) {
	int16_t d[3];

	read_reg(_g_addr, OUT_X_L_G, d);
	
	for(int i=0; i<3; ++i) {
		*(m + i) = *(d + i);
	}

	return 0;
}

void print_header() {
	printf("   | ");
	for(int i = 0; i < 16; ++i) {
		printf("%02x ", i);
	}
	printf("\n---+");
	for(int i = 0; i < 16; ++i) {
		printf("---");
	}
	printf("\n");
}

void dump_registers()
{
	printf("LSM9DS0 Registers:\n");

	select_device(_g_addr);
	printf("Gyroscope Registers:\n");
	print_header();
	for(uint8_t i = 0; i < 4; ++i) {
		printf("%02x | ", i << 4);
		for(uint8_t j = 0; j < 16; ++j) {
			uint8_t r = (i << 4) | j;
			if(r < 0xf || (r > 0xf && r < 0x20) || r == 0x26 ||
								r > 0x38) {
				printf("-- ");
				continue;
			}

			uint8_t v = i2c_smbus_read_byte_data(fd, 0x80 | r);
			printf("%02x ", (unsigned)v);
		}
		printf("\n");
	}

	select_device(_xm_addr);
	printf("\nAccelerometer Registers:\n");
	print_header();
	for(uint8_t i = 0; i < 4; ++i) {
		printf("%02x | ", i << 4);
		for(uint8_t j = 0; j < 16; ++j) {
			uint8_t r = (i << 4) | j;
			if(r < 0x5 || r == 0xe || r == 0x10 || r == 0x11) {
				printf("-- ");
				continue;
			}

			uint8_t v = i2c_smbus_read_byte_data(fd, 0x80 | r);
			printf("%02x ", (unsigned)v);
		}
		printf("\n");
	}
}
