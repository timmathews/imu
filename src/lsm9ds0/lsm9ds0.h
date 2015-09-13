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

#ifndef _LSM9DS0_H
#define _LSM9DS0_H 1

#include <stdint.h>

float gyro_scale, accel_scale, mag_scale;

void write_reg(int addr, uint8_t reg, uint8_t value);
void read_reg(int addr, uint8_t reg, int16_t *a);
void init_imu(char *file, int xm_addr, int g_addr);
int read_raw_mag(double *m);
void dump_registers();

/** LSM9DS0 Gyro Registers **/
#define WHO_AM_I_G			 0x0F
#define CTRL_REG1_G			 0x20
#define CTRL_REG1_G_DR_95Hz_BW_1250Hz	(0x0 << 4)
#define CTRL_REG1_G_DR_95Hz_BW_25Hz	(0x1 << 4)
#define CTRL_REG1_G_DR_190Hz_BW_1250Hz	(0x4 << 4)
#define CTRL_REG1_G_DR_190Hz_BW_25Hz	(0x5 << 4)
#define CTRL_REG1_G_DR_190Hz_BW_50Hz	(0x6 << 4)
#define CTRL_REG1_G_DR_190Hz_BW_70Hz	(0x7 << 4)
#define CTRL_REG1_G_DR_380Hz_BW_20Hz	(0x8 << 4)
#define CTRL_REG1_G_DR_380Hz_BW_25Hz	(0x9 << 4)
#define CTRL_REG1_G_DR_380Hz_BW_50Hz	(0xA << 4)
#define CTRL_REG1_G_DR_380Hz_BW_100Hz	(0xB << 4)
#define CTRL_REG1_G_DR_760Hz_BW_30Hz	(0xC << 4)
#define CTRL_REG1_G_DR_760Hz_BW_35Hz	(0xD << 4)
#define CTRL_REG1_G_DR_760Hz_BW_50Hz	(0xE << 4)
#define CTRL_REG1_G_DR_760Hz_BW_100Hz	(0xF << 4)
#define CTRL_REG1_G_PD			(0x1 << 3)
#define CTRL_REG1_G_ZEN			(0x1 << 2)
#define CTRL_REG1_G_YEN			(0x1 << 1)
#define CTRL_REG1_G_XEN			(0x1 << 0)

#define CTRL_REG2_G			 0x21
#define CTRL_REG2_G_HPM_NORMAL_RESET	(0x0 << 4)
#define CTRL_REG2_G_HPM_REFERENCE	(0x1 << 4)
#define CTRL_REG2_G_HPM_NORMAL		(0x2 << 4)
#define CTRL_REG2_G_HPM_AUTORESET	(0x3 << 4)
#define CTRL_REG2_G_HPCF_0		(0x0 << 0)
#define CTRL_REG2_G_HPCF_1		(0x1 << 0)
#define CTRL_REG2_G_HPCF_2		(0x2 << 0)
#define CTRL_REG2_G_HPCF_3		(0x3 << 0)
#define CTRL_REG2_G_HPCF_4		(0x4 << 0)
#define CTRL_REG2_G_HPCF_5		(0x5 << 0)
#define CTRL_REG2_G_HPCF_6		(0x6 << 0)
#define CTRL_REG2_G_HPCF_7		(0x7 << 0)
#define CTRL_REG2_G_HPCF_8		(0x8 << 0)
#define CTRL_REG2_G_HPCF_9		(0x9 << 0) 

#define CTRL_REG4_G			 0x23
#define CTRL_REG4_G_BDU			(0x1 << 7)
#define CTRL_REG4_G_BLE			(0x1 << 6)
#define CTRL_REG4_G_FS_245DPS		(0x0 << 4)
#define CTRL_REG4_G_FS_500DPS		(0x1 << 4)
#define CTRL_REG4_G_FS_2000DPS		(0x2 << 4)
#define CTRL_REG4_G_ST_NORMAL		(0x0 << 1)
#define CTRL_REG4_G_ST_0		(0x1 << 1)
#define CTRL_REG4_G_ST_1		(0x3 << 1)
#define CTRL_REG4_G_SIM_3WIRE		(0x1 << 0)

#define OUT_X_L_G			 0x28
#define OUT_X_H_G			 0x29
#define OUT_Y_L_G			 0x2A
#define OUT_Y_H_G			 0x2B
#define OUT_Z_L_G			 0x2C
#define OUT_Z_H_G			 0x2D

/** LSM9DS0 Accel/Magneto (XM) Registers **/
#define CTRL_REG1_XM			 0x20
#define CTRL_REG1_XM_AODR_POWERDOWN	(0x0 << 4)
#define CTRL_REG1_XM_AODR_3_125Hz	(0x1 << 4)
#define CTRL_REG1_XM_AODR_6_25Hz	(0x2 << 4)
#define CTRL_REG1_XM_AODR_12_5Hz	(0x3 << 4)
#define CTRL_REG1_XM_AODR_25Hz		(0x4 << 4)
#define CTRL_REG1_XM_AODR_50Hz		(0x5 << 4)
#define CTRL_REG1_XM_AODR_100Hz		(0x6 << 4)
#define CTRL_REG1_XM_AODR_200Hz		(0x7 << 4)
#define CTRL_REG1_XM_AODR_400Hz 	(0x8 << 4)
#define CTRL_REG1_XM_AODR_800Hz		(0x9 << 4)
#define CTRL_REG1_XM_AODR_1600Hz	(0xA << 4)
#define CTRL_REG1_XM_BDU		(0x1 << 3)
#define CTRL_REG1_XM_AZEN		(0x1 << 2)
#define CTRL_REG1_XM_AYEN		(0x1 << 1)
#define CTRL_REG1_XM_AXEN		(0x1 << 0)

#define CTRL_REG2_XM			 0x21
#define CTRL_REG2_XM_ABW_773Hz		(0x0 << 6)
#define CTRL_REG2_XM_ABW_194Hz		(0x1 << 6)
#define CTRL_REG2_XM_ABW_362Hz		(0x2 << 6)
#define CTRL_REG2_XM_ABW_50Hz 		(0x3 << 6)
#define CTRL_REG2_XM_AFS_2G		(0x0 << 3)
#define CTRL_REG2_XM_AFS_4G		(0x1 << 3)
#define CTRL_REG2_XM_AFS_6G		(0x2 << 3)
#define CTRL_REG2_XM_AFS_8G		(0x3 << 3)
#define CTRL_REG2_XM_AFS_16G		(0x4 << 3)
#define CTRL_REG2_XM_AST_NORMAL		(0x0 << 1)
#define CTRL_REG2_XM_AST_POSITIVE	(0x1 << 1)
#define CTRL_REG2_XM_AST_NEGATIVE	(0x2 << 1)
#define CTRL_REG2_XM_SIM_3WIRE		(0x1 << 0)

#define CTRL_REG5_XM			 0x24
#define CTRL_REG5_XM_TEMP_EN		(0x1 << 7)
#define CTRL_REG5_XM_M_RES_LOW		(0x0 << 5)
#define CTRL_REG5_XM_M_RES_HIGH		(0x3 << 5)
#define CTRL_REG5_XM_ODR_3_125Hz	(0x0 << 2)
#define CTRL_REG5_XM_ODR_6_25Hz		(0x1 << 2)
#define CTRL_REG5_XM_ODR_12_5Hz		(0x2 << 2)
#define CTRL_REG5_XM_ODR_25Hz		(0x3 << 2)
#define CTRL_REG5_XM_ODR_50Hz		(0x4 << 2)
#define CTRL_REG5_XM_ODR_100Hz		(0x5 << 2)
#define CTRL_REG5_XM_LIR2		(0x1 << 1)
#define CTRL_REG5_XM_LIR1		(0x1 << 0)

#define CTRL_REG6_XM			 0x25
#define CTRL_REG6_XM_MFS_2Gs		(0x0 << 5)
#define CTRL_REG6_XM_MFS_4Gs		(0x1 << 5)
#define CTRL_REG6_XM_MFS_8Gs		(0x2 << 5)
#define CTRL_REG6_XM_MFS_12Gs		(0x3 << 5)

#define CTRL_REG7_XM			 0x26
#define CTRL_REG7_XM_AHPM_NORMAL_RESET	(0x0 << 6)
#define CTRL_REG7_XM_AHPM_REFERENCE	(0x1 << 6)
#define CTRL_REG7_XM_AHPM_NORMAL	(0x2 << 6)
#define CTRL_REG7_XM_AHPM_AUTORESET	(0x3 << 6)
#define CTRL_REG7_XM_AFDS		(0x1 << 5)
#define CTRL_REG7_XM_MLP		(0x1 << 2)
#define CTRL_REG7_XM_MD_CONTINUOUS	(0x0 << 0)
#define CTRL_REG7_XM_MD_SINGLE		(0x1 << 0)
#define CTRL_REG7_XM_MD_POWERDOWN	(0x2 << 0)

#define OUT_X_L_M			 0x08
#define OUT_X_H_M			 0x09
#define OUT_Y_L_M			 0x0A
#define OUT_Y_H_M			 0x0B
#define OUT_Z_L_M			 0x0C
#define OUT_Z_H_M			 0x0D

#define OUT_X_L_A			 0x28
#define OUT_X_H_A			 0x29
#define OUT_Y_L_A			 0x2A
#define OUT_Y_H_A			 0x2B
#define OUT_Z_L_A			 0x2C
#define OUT_Z_H_A			 0x2D

#endif
