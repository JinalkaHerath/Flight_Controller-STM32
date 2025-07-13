#include "pwm.h"

static TIM_HandleTypeDef *htim_pwm;

void PWM_Init(TIM_HandleTypeDef *htim) {
  htim_pwm = htim;
  HAL_TIM_PWM_Start(htim_pwm, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(htim_pwm, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(htim_pwm, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(htim_pwm, TIM_CHANNEL_4);
}

void SetMotorPWM(uint16_t pulse_us[4]) {
  __HAL_TIM_SET_COMPARE(htim_pwm, TIM_CHANNEL_1, pulse_us[0]);
  __HAL_TIM_SET_COMPARE(htim_pwm, TIM_CHANNEL_2, pulse_us[1]);
  __HAL_TIM_SET_COMPARE(htim_pwm, TIM_CHANNEL_3, pulse_us[2]);
  __HAL_TIM_SET_COMPARE(htim_pwm, TIM_CHANNEL_4, pulse_us[3]);
}
