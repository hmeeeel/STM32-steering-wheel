#include "app.h"
#include "app_config.h"
#include "joystick.h"
#include "protocol.h"
#include "servo.h"
#include "stm32f1xx_hal.h"

extern ADC_HandleTypeDef  hadc1;
extern TIM_HandleTypeDef  htim3;
extern UART_HandleTypeDef huart2;

#define RX_RING_SIZE  64u            /* степень двойки */

static volatile uint8_t s_ring[RX_RING_SIZE];
static volatile uint8_t s_head;
static volatile uint8_t s_tail;
static uint8_t          s_rx_byte;
static proto_rx_t       s_rx;
static uint32_t         s_next_tx_ms;
static uint16_t         s_seq;

static void ring_push(uint8_t b)
{
    uint8_t next = (uint8_t)((s_head + 1u) & (RX_RING_SIZE - 1u));
    if (next != s_tail) {
        s_ring[s_head] = b;
        s_head = next;
    }
}

static int ring_pop(uint8_t *b)
{
    if (s_tail == s_head) {
        return 0;
    }
    *b = s_ring[s_tail];
    s_tail = (uint8_t)((s_tail + 1u) & (RX_RING_SIZE - 1u));
    return 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        ring_push(s_rx_byte);
        (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1u);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1u);
    }
}

static void send_telemetry(uint32_t now_ms)
{
    uint16_t x, y;
    joystick_read(&x, &y);

    proto_telemetry_t t;
    t.seq = s_seq++;
    t.t_ms = now_ms;
    t.x = x;
    t.y = y;
    t.servo_us = servo_get_pulse_us();

    char   line[PROTO_TX_MAX];
    size_t n = proto_format_telemetry(line, sizeof line, &t);
    if (n > 0u) {
        (void)HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)n, 10u);
    }
}

void app_init(void)
{
    (void)joystick_init(&hadc1);
    (void)servo_init(&htim3, TIM_CHANNEL_1);
    proto_rx_init(&s_rx);
    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1u);
    s_next_tx_ms = HAL_GetTick() + APP_TELEMETRY_PERIOD_MS;
}

void app_poll(void)
{
    uint8_t     b;
    proto_cmd_t cmd;

    while (ring_pop(&b)) {
        if (proto_rx_feed(&s_rx, b, &cmd) && cmd.type == PROTO_CMD_SERVO) {
            servo_set_pulse_us(cmd.servo_us);
        }
    }

    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - s_next_tx_ms) >= 0) {
        s_next_tx_ms += APP_TELEMETRY_PERIOD_MS;
        if ((int32_t)(now - s_next_tx_ms) >= 0) {
            s_next_tx_ms = now + APP_TELEMETRY_PERIOD_MS;
        }
        send_telemetry(now);
    }
}
