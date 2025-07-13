/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bmp280.h
  * @brief   BMP280 barometric pressure sensor driver header
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __BMP280_H
#define __BMP280_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Exported types ------------------------------------------------------------*/
typedef struct {
    float pressure;    // hPa
    float temperature; // °C
    float altitude;    // meters
    uint32_t last_update;
} BMP280_Data;

/* Exported constants --------------------------------------------------------*/
#define BMP280_I2C_ADDRESS 0x76 << 1 // 0x76 or 0x77 (depends on SDO pin)
#define BMP280_CHIP_ID     0x58

/* Exported functions --------------------------------------------------------*/
uint8_t BMP280_Init(I2C_HandleTypeDef *hi2c);
void BMP280_ReadData(I2C_HandleTypeDef *hi2c, BMP280_Data *data);
void BMP280_CalculateAltitude(BMP280_Data *data, float seaLevelhPa);

#ifdef __cplusplus
}
#endif

#endif /* __BMP280_H */
