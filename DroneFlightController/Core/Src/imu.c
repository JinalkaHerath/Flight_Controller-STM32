#include "imu.h"

static I2C_HandleTypeDef *hi2c_imu;

void IMU_Init(I2C_HandleTypeDef *hi2c) {
  hi2c_imu = hi2c;
  uint8_t data = 0x01;
  HAL_I2C_Mem_Write(hi2c_imu, MPU6050_ADDR, 0x6B, 1, &data, 1, 100); // Wake up IMU
}

void IMU_ReadData(IMU_Data *data) {
  uint8_t buffer[14];
  HAL_I2C_Mem_Read(hi2c_imu, MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, 1, buffer, 14, 100);

  // Convert raw data (16-bit signed)
  data->accel[0] = (int16_t)(buffer[0] << 8 | buffer[1]) * (16.0f / 32768.0f) * 9.81f;
  data->gyro[0]  = (int16_t)(buffer[8] << 8 | buffer[9]) * (2000.0f / 32768.0f) * (3.14159f / 180.0f);
  // Repeat for Y/Z axes...
}
