#include "MPU6050.h"

#define MPU6050_ADDRESS_MIN       0x68U
#define MPU6050_ADDRESS_MAX       0x69U
#define MPU6050_REG_GYRO_CONFIG   0x1BU
#define MPU6050_REG_ACCEL_CONFIG  0x1CU
#define MPU6050_REG_ACCEL_XOUT_H  0x3BU
#define MPU6050_REG_PWR_MGMT_1    0x6BU
#define MPU6050_REG_WHO_AM_I      0x75U
#define MPU6050_WHO_AM_I_VALUE    0x68U

static float GetAccelLsbPerG(MPU6050_AccelRange range)
{
  switch (range)
  {
    case MPU6050_ACCEL_RANGE_2G:  return 16384.0f;
    case MPU6050_ACCEL_RANGE_4G:  return 8192.0f;
    case MPU6050_ACCEL_RANGE_8G:  return 4096.0f;
    case MPU6050_ACCEL_RANGE_16G: return 2048.0f;
    default: return 0.0f;
  }
}

static float GetGyroLsbPerDps(MPU6050_GyroRange range)
{
  switch (range)
  {
    case MPU6050_GYRO_RANGE_250_DPS:  return 131.0f;
    case MPU6050_GYRO_RANGE_500_DPS:  return 65.5f;
    case MPU6050_GYRO_RANGE_1000_DPS: return 32.8f;
    case MPU6050_GYRO_RANGE_2000_DPS: return 16.4f;
    default: return 0.0f;
  }
}

static HAL_StatusTypeDef WriteRegister(I2C_HandleTypeDef *i2c,
                                       uint16_t address,
                                       uint8_t reg,
                                       uint8_t value,
                                       uint32_t timeout_ms)
{
  return HAL_I2C_Mem_Write(i2c, address, reg, I2C_MEMADD_SIZE_8BIT,
                           &value, 1U, timeout_ms);
}

static int16_t ReadInt16(const uint8_t *data)
{
  return (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

HAL_StatusTypeDef MPU6050_Init(MPU6050_HandleTypeDef *sensor,
                               I2C_HandleTypeDef *i2c,
                               uint8_t address_7bit,
                               MPU6050_AccelRange accel_range,
                               MPU6050_GyroRange gyro_range,
                               uint32_t timeout_ms)
{
  if (sensor == NULL || i2c == NULL ||
      address_7bit < MPU6050_ADDRESS_MIN || address_7bit > MPU6050_ADDRESS_MAX)
  {
    return HAL_ERROR;
  }

  const float accel_lsb_per_g = GetAccelLsbPerG(accel_range);
  const float gyro_lsb_per_dps = GetGyroLsbPerDps(gyro_range);
  if (accel_lsb_per_g == 0.0f || gyro_lsb_per_dps == 0.0f)
  {
    return HAL_ERROR;
  }

  const uint16_t hal_address = (uint16_t)address_7bit << 1;
  HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(i2c, hal_address, 2U, timeout_ms);
  if (status != HAL_OK)
  {
    return status;
  }

  uint8_t who_am_i = 0U;
  status = HAL_I2C_Mem_Read(i2c, hal_address, MPU6050_REG_WHO_AM_I,
                            I2C_MEMADD_SIZE_8BIT, &who_am_i, 1U, timeout_ms);
  if (status != HAL_OK)
  {
    return status;
  }
  if (who_am_i != MPU6050_WHO_AM_I_VALUE)
  {
    return HAL_ERROR;
  }

  status = WriteRegister(i2c, hal_address, MPU6050_REG_PWR_MGMT_1, 0x01U, timeout_ms);
  if (status != HAL_OK)
  {
    return status;
  }

  status = WriteRegister(i2c, hal_address, MPU6050_REG_GYRO_CONFIG,
                         (uint8_t)gyro_range, timeout_ms);
  if (status != HAL_OK)
  {
    return status;
  }

  status = WriteRegister(i2c, hal_address, MPU6050_REG_ACCEL_CONFIG,
                         (uint8_t)accel_range, timeout_ms);
  if (status != HAL_OK)
  {
    return status;
  }

  sensor->i2c = i2c;
  sensor->hal_address = hal_address;
  sensor->accel_lsb_per_g = accel_lsb_per_g;
  sensor->gyro_lsb_per_dps = gyro_lsb_per_dps;
  return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Read(MPU6050_HandleTypeDef *sensor,
                               MPU6050_Data *data,
                               uint32_t timeout_ms)
{
  if (sensor == NULL || data == NULL || sensor->i2c == NULL ||
      sensor->accel_lsb_per_g <= 0.0f || sensor->gyro_lsb_per_dps <= 0.0f)
  {
    return HAL_ERROR;
  }

  uint8_t raw[14];
  const HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
      sensor->i2c, sensor->hal_address, MPU6050_REG_ACCEL_XOUT_H,
      I2C_MEMADD_SIZE_8BIT, raw, sizeof(raw), timeout_ms);
  if (status != HAL_OK)
  {
    return status;
  }

  MPU6050_Data sample;
  sample.accel_x_raw = ReadInt16(&raw[0]);
  sample.accel_y_raw = ReadInt16(&raw[2]);
  sample.accel_z_raw = ReadInt16(&raw[4]);
  sample.temperature_raw = ReadInt16(&raw[6]);
  sample.gyro_x_raw = ReadInt16(&raw[8]);
  sample.gyro_y_raw = ReadInt16(&raw[10]);
  sample.gyro_z_raw = ReadInt16(&raw[12]);

  sample.accel_x_g = sample.accel_x_raw / sensor->accel_lsb_per_g;
  sample.accel_y_g = sample.accel_y_raw / sensor->accel_lsb_per_g;
  sample.accel_z_g = sample.accel_z_raw / sensor->accel_lsb_per_g;
  sample.temperature_c = sample.temperature_raw / 340.0f + 36.53f;
  sample.gyro_x_dps = sample.gyro_x_raw / sensor->gyro_lsb_per_dps;
  sample.gyro_y_dps = sample.gyro_y_raw / sensor->gyro_lsb_per_dps;
  sample.gyro_z_dps = sample.gyro_z_raw / sensor->gyro_lsb_per_dps;

  *data = sample;
  return HAL_OK;
}