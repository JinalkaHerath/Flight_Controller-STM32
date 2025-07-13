#ifndef LED_H
#define LED_H

#include "main.h"

void LED_Init(void);
void LED_SetColor(LED_Color color);
void LED_SetRGB(uint8_t red, uint8_t green, uint8_t blue);
void LED_Blink(LED_Color color, uint16_t period_ms);

#endif /* LED_H */
