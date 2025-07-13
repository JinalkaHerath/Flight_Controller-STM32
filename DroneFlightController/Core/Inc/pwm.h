#pragma once
#include "stm32f7xx_hal.h"

void PWM_Init(TIM_HandleTypeDef *htim);
void SetMotorPWM(uint16_t pulse_us[4]);
