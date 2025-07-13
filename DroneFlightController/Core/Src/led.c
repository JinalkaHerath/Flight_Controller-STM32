#include "led.h"
#include "main.h"

static TIM_HandleTypeDef* led_pwm_tim = NULL;

void LED_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Initialize GPIO pins
    GPIO_InitStruct.Pin = LED_RED_PIN | LED_GREEN_PIN | LED_BLUE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4; // Using TIM4 for PWM
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Initialize PWM timer (TIM4)
    TIM_OC_InitTypeDef sConfigOC = {0};

    led_pwm_tim = &htim4;
    led_pwm_tim->Instance = TIM4;
    led_pwm_tim->Init.Prescaler = (SystemCoreClock / 1000000) - 1; // 1MHz
    led_pwm_tim->Init.CounterMode = TIM_COUNTERMODE_UP;
    led_pwm_tim->Init.Period = 255; // 8-bit resolution
    led_pwm_tim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(led_pwm_tim);

    // Configure channels
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(led_pwm_tim, &sConfigOC, TIM_CHANNEL_1); // Red
    HAL_TIM_PWM_ConfigChannel(led_pwm_tim, &sConfigOC, TIM_CHANNEL_2); // Green
    HAL_TIM_PWM_ConfigChannel(led_pwm_tim, &sConfigOC, TIM_CHANNEL_3); // Blue

    HAL_TIM_PWM_Start(led_pwm_tim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(led_pwm_tim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(led_pwm_tim, TIM_CHANNEL_3);
}

void LED_SetColor(LED_Color color) {
    switch(color) {
        case LED_OFF:     LED_SetRGB(0,   0,   0);   break;
        case LED_RED:     LED_SetRGB(255, 0,   0);   break;
        case LED_GREEN:   LED_SetRGB(0,   255, 0);   break;
        case LED_BLUE:    LED_SetRGB(0,   0,   255); break;
        case LED_YELLOW:  LED_SetRGB(255, 255, 0);   break;
        case LED_CYAN:    LED_SetRGB(0,   255, 255); break;
        case LED_MAGENTA: LED_SetRGB(255, 0,   255); break;
        case LED_WHITE:   LED_SetRGB(255, 255, 255); break;
    }
}

void LED_SetRGB(uint8_t red, uint8_t green, uint8_t blue) {
    __HAL_TIM_SET_COMPARE(led_pwm_tim, TIM_CHANNEL_1, red);
    __HAL_TIM_SET_COMPARE(led_pwm_tim, TIM_CHANNEL_2, green);
    __HAL_TIM_SET_COMPARE(led_pwm_tim, TIM_CHANNEL_3, blue);
}

void LED_Blink(LED_Color color, uint16_t period_ms) {
    static uint32_t last_toggle = 0;
    static uint8_t state = 0;

    if(HAL_GetTick() - last_toggle > period_ms) {
        state = !state;
        last_toggle = HAL_GetTick();
        LED_SetColor(state ? color : LED_OFF);
    }
}
