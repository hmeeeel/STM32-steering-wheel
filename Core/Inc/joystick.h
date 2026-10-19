#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"

bool joystick_init(ADC_HandleTypeDef *hadc);
void joystick_read(uint16_t *x, uint16_t *y);

#endif /* JOYSTICK_H */
