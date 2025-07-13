#pragma once
#include "stm32f7xx_hal.h"

#define MPU6050_ADDR         0x68 << 1
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

typedef struct {
  float accel[3];  // m/s²
  float gyro[3];   // rad/s
} IMU_Data;

void IMU_Init(I2C_HandleTypeDef *hi2c);
void IMU_ReadData(IMU_Data *data);
