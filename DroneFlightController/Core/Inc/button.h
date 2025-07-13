/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bmp280.h
  * @brief   BMP280 barometric pressure sensor driver header
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef BUTTON_H
#define BUTTON_H

#include "main.h"

void Button_Init(void);
Button_State Button_GetState(void);
uint8_t Button_WasPressed(void);
uint32_t Button_GetHoldTime(void);

#endif /* BUTTON_H */
