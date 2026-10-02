#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"

typedef enum
{
  MPU6050_ACCEL_RANGE_2G = 0x00U,
  MPU6050_ACCEL_RANGE_4G = 0x08U,
  MPU6050_ACCEL_RANGE_8G = 0x10U,
  MPU6050_ACCEL_RANGE_16G = 0x18U
} MPU6050_AccelRange;

typedef enum
{
  MPU6050_GYRO_RANGE_250_DPS = 0x00U,
  MPU6050_GYRO_RANGE_500_DPS = 0x08U,
  MPU6050_GYRO_RANGE_1000_DPS = 0x10U,
  MPU6050_GYRO_RANGE_2000_DPS = 0x18U
} MPU6050_GyroRange;

typedef struct
{
  I2C_HandleTypeDef *i2c;
  uint16_t hal_address;
  float accel_lsb_per_g;
  float gyro_lsb_per_dps;
} MPU6050_HandleTypeDef;

typedef struct
{
  int16_t accel_x_raw;
  int16_t accel_y_raw;
  int16_t accel_z_raw;
  int16_t temperature_raw;
  int16_t gyro_x_raw;
  int16_t gyro_y_raw;
  int16_t gyro_z_raw;
  float accel_x_g;
  float accel_y_g;
  float accel_z_g;
  float temperature_c;
  float gyro_x_dps;
  float gyro_y_dps;
  float gyro_z_dps;
} MPU6050_Data;

HAL_StatusTypeDef MPU6050_Init(MPU6050_HandleTypeDef *sensor,
                               I2C_HandleTypeDef *i2c,
                               uint8_t address_7bit,
                               MPU6050_AccelRange accel_range,
                               MPU6050_GyroRange gyro_range,
                               uint32_t timeout_ms);

HAL_StatusTypeDef MPU6050_Read(MPU6050_HandleTypeDef *sensor,
                               MPU6050_Data *data,
                               uint32_t timeout_ms);

#endif/*
 * MPU6050.h
 *
 *  Created on: Sep 3, 2023
 *      Author: suchs
 */
#pragma once

//#define TXT_LEN(x) x, (sizeoff(x)-1)

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_


// Zakresy żyroskopu
#define FS_SEL_250		0
#define FS_SEL_500		8
#define FS_SEL_1000		16
#define FS_SEL_2000		24

// Zakresy akcelerometru
#define AFS_SEL_2G		0
#define AFS_SEL_4G		8
#define AFS_SEL_8G		16
#define AFS_SEL_6G		24


//Define Registers
#define WHO_AM_I_REG		0x75
#define DEVICE_ADDRESS 		0x68
#define PWR_MAGT_1_REG		0x6B
#define PWR_MAGT_2_REG		0x6C
#define CONFIG_REG			0x1A
#define GYRO_CONFIG_REG		0x1B //27
#define ACCEL_CONFIG_REG	0x1C //28
#define SMPLRT_DIV_REG		0x19
#define INT_STATUS_REG		0x3A
#define ACCEL_XOUT_H_REG	0x3B
#define ACCEL_YOUT_H_REG	0x3D
#define ACCEL_ZOUT_H_REG	0x3F
#define TEMP_OUT_H_REG		0x41
#define GYRO_XOUT_H_REG		0x43
#define FIFO_EN_REG 		0x23
#define INT_ENABLE_REG 		0x38
#define I2CMACO_REG 		0x23
#define USER_CNT_REG		0x6A
#define FIFO_COUNTH_REG 	0x72
#define FIFO_R_W_REG 		0x74


void mpu6050_init();
void mpu6050_read();
void WhoAmI();

#endif /* INC_MPU6050_H_ */
