#include "button.h"
#include "main.h"

static uint32_t button_press_time = 0;
static uint8_t button_was_pressed = 0;

void Button_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = BOOT_BUTTON_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BOOT_BUTTON_PORT, &GPIO_InitStruct);
}

Button_State Button_GetState(void) {
    static uint8_t last_state = BUTTON_RELEASED;
    uint8_t current_state = HAL_GPIO_ReadPin(BOOT_BUTTON_PORT, BOOT_BUTTON_PIN) == GPIO_PIN_RESET;

    if(current_state && !last_state) {
        button_press_time = HAL_GetTick();
        button_was_pressed = 1;
    }

    last_state = current_state;

    if(!current_state) return BUTTON_RELEASED;
    if(HAL_GetTick() - button_press_time > 3000) return BUTTON_HELD;
    return BUTTON_PRESSED;
}

uint8_t Button_WasPressed(void) {
    if(button_was_pressed) {
        button_was_pressed = 0;
        return 1;
    }
    return 0;
}

uint32_t Button_GetHoldTime(void) {
    if(HAL_GPIO_ReadPin(BOOT_BUTTON_PORT, BOOT_BUTTON_PIN) == GPIO_PIN_SET)
        return 0;
    return HAL_GetTick() - button_press_time;
}
