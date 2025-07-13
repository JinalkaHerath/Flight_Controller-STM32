/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bmp280.c
  * @brief   BMP280 barometric pressure sensor driver
  ******************************************************************************
  */
/* USER CODE END Header */

#include "bmp280.h"
#include <math.h>

// Compensation parameters
static struct {
    uint16_t dig_T1;
    int16_t  dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    int32_t  t_fine;
} bmp280_calib;

uint8_t BMP280_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t chip_id = 0;

    // Check device ID
    HAL_I2C_Mem_Read(hi2c, BMP280_I2C_ADDRESS, 0xD0, 1, &chip_id, 1, HAL_MAX_DELAY);
    if(chip_id != BMP280_CHIP_ID) return 0;

    // Read calibration data
    uint8_t calib_data[24];
    HAL_I2C_Mem_Read(hi2c, BMP280_I2C_ADDRESS, 0x88, 1, calib_data, 24, HAL_MAX_DELAY);

    // Parse calibration data
    bmp280_calib.dig_T1 = (uint16_t)(calib_data[1] << 8) | calib_data[0];
    bmp280_calib.dig_T2 = (int16_t)(calib_data[3] << 8) | calib_data[2];
    bmp280_calib.dig_T3 = (int16_t)(calib_data[5] << 8) | calib_data[4];

    bmp280_calib.dig_P1 = (uint16_t)(calib_data[7] << 8) | calib_data[6];
    bmp280_calib.dig_P2 = (int16_t)(calib_data[9] << 8) | calib_data[8];
    bmp280_calib.dig_P3 = (int16_t)(calib_data[11] << 8) | calib_data[10];
    bmp280_calib.dig_P4 = (int16_t)(calib_data[13] << 8) | calib_data[12];
    bmp280_calib.dig_P5 = (int16_t)(calib_data[15] << 8) | calib_data[14];
    bmp280_calib.dig_P6 = (int16_t)(calib_data[17] << 8) | calib_data[16];
    bmp280_calib.dig_P7 = (int16_t)(calib_data[19] << 8) | calib_data[18];
    bmp280_calib.dig_P8 = (int16_t)(calib_data[21] << 8) | calib_data[20];
    bmp280_calib.dig_P9 = (int16_t)(calib_data[23] << 8) | calib_data[22];

    // Configure sensor (oversampling x4 for pressure and temperature, normal mode)
    uint8_t ctrl_meas = 0x27; // 001 001 11 (x1 temp, x1 press, normal mode)
    HAL_I2C_Mem_Write(hi2c, BMP280_I2C_ADDRESS, 0xF4, 1, &ctrl_meas, 1, HAL_MAX_DELAY);

    return 1;
}

static int32_t compensate_T(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T>>3) - ((int32_t)bmp280_calib.dig_T1<<1))) * ((int32_t)bmp280_calib.dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((int32_t)bmp280_calib.dig_T1)) * ((adc_T>>4) - ((int32_t)bmp280_calib.dig_T1))) >> 12) * ((int32_t)bmp280_calib.dig_T3)) >> 14;
    bmp280_calib.t_fine = var1 + var2;
    T = (bmp280_calib.t_fine * 5 + 128) >> 8;
    return T;
}

static uint32_t compensate_P(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)bmp280_calib.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp280_calib.dig_P6;
    var2 = var2 + ((var1*(int64_t)bmp280_calib.dig_P5)<<17);
    var2 = var2 + (((int64_t)bmp280_calib.dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)bmp280_calib.dig_P3)>>8) + ((var1 * (int64_t)bmp280_calib.dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)bmp280_calib.dig_P1)>>33;
    if (var1 == 0) return 0;
    p = 1048576 - adc_P;
    p = (((p<<31) - var2)*3125)/var1;
    var1 = (((int64_t)bmp280_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)bmp280_calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280_calib.dig_P7)<<4);
    return (uint32_t)p;
}

void BMP280_ReadData(I2C_HandleTypeDef *hi2c, BMP280_Data *data) {
    uint8_t raw_data[6];

    // Read pressure and temperature data
    HAL_I2C_Mem_Read(hi2c, BMP280_I2C_ADDRESS, 0xF7, 1, raw_data, 6, HAL_MAX_DELAY);

    // Combine bytes
    int32_t adc_P = (int32_t)(((uint32_t)raw_data[0] << 12) | ((uint32_t)raw_data[1] << 4) | ((uint32_t)raw_data[2] >> 4);
    int32_t adc_T = (int32_t)(((uint32_t)raw_data[3] << 12) | ((uint32_t)raw_data[4] << 4) | ((uint32_t)raw_data[5] >> 4));

    // Compensate data
    int32_t t = compensate_T(adc_T);
    uint32_t p = compensate_P(adc_P);

    // Store results
    data->temperature = (float)t / 100.0f;
    data->pressure = (float)p / 25600.0f; // Convert to hPa
    data->last_update = HAL_GetTick();
}

void BMP280_CalculateAltitude(BMP280_Data *data, float seaLevelhPa) {
    // Simplified altitude calculation (h = 44330 * [1 - (P/P0)^(1/5.255)])
    data->altitude = 44330.0f * (1.0f - powf((data->pressure / seaLevelhPa), 0.1903f));
}
