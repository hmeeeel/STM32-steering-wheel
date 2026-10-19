#include "servo.h"
#include "app_config.h"

static TIM_HandleTypeDef *s_htim;
static uint32_t           s_channel;
static uint16_t           s_us;
static bool               s_enabled;

static uint32_t timer_clock_hz(void)
{
    RCC_ClkInitTypeDef clk;
    uint32_t           latency;
    HAL_RCC_GetClockConfig(&clk, &latency);
    uint32_t hz = HAL_RCC_GetPCLK1Freq();
    return (clk.APB1CLKDivider == RCC_HCLK_DIV1) ? hz : hz * 2u;
}

bool servo_init(TIM_HandleTypeDef *htim, uint32_t channel)
{
    s_htim = htim;
    s_channel = channel;
    s_enabled = false;
    s_us = 0u;

    uint32_t tick_hz = timer_clock_hz() / (htim->Init.Prescaler + 1u);
    if (tick_hz != SERVO_TICK_HZ || htim->Init.Period != (SERVO_PERIOD_US - 1u)) {
        return false;
    }
    __HAL_TIM_SET_COMPARE(htim, channel, SERVO_NEUTRAL_US);
    if (HAL_TIM_PWM_Start(htim, channel) != HAL_OK) {
        return false;
    }
    s_us = SERVO_NEUTRAL_US;
    s_enabled = true;
    return true;
}

void servo_set_pulse_us(uint16_t us)
{
    if (!s_enabled) {
        return;
    }
    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;
    s_us = us;
    __HAL_TIM_SET_COMPARE(s_htim, s_channel, us);
}

uint16_t servo_get_pulse_us(void)
{
    return s_enabled ? s_us : 0u;
}
