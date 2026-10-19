#ifndef SERVO_H
#define SERVO_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"

bool     servo_init(TIM_HandleTypeDef *htim, uint32_t channel);
void     servo_set_pulse_us(uint16_t us);
uint16_t servo_get_pulse_us(void);

#endif /* SERVO_H */
