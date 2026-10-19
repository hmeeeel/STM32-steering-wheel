#include "joystick.h"

#define JOY_CHANNELS   2u
#define ADC_MASK       0x0FFFu

static volatile uint32_t s_raw[JOY_CHANNELS];

bool joystick_init(ADC_HandleTypeDef *hadc)
{
    if (HAL_ADCEx_Calibration_Start(hadc) != HAL_OK) {
        return false;
    }
    return HAL_ADC_Start_DMA(hadc, (uint32_t *)s_raw, JOY_CHANNELS) == HAL_OK;
}

void joystick_read(uint16_t *x, uint16_t *y)
{
    *x = (uint16_t)(s_raw[0] & ADC_MASK);
    *y = (uint16_t)(s_raw[1] & ADC_MASK);
}
