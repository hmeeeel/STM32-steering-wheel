#include "protocol.h"

#include <stdio.h>
#include <string.h>

uint8_t proto_checksum(const char *s, size_t n)
{
    uint8_t cs = 0u;
    while (n--) {
        cs ^= (uint8_t)*s++;
    }
    return cs;
}

size_t proto_format_telemetry(char *out, size_t cap, const proto_telemetry_t *t)
{
    char body[PROTO_TX_MAX];
    int  n = snprintf(body, sizeof body, "JS,%u,%lu,%u,%u,%u",
                      (unsigned)t->seq, (unsigned long)t->t_ms,
                      (unsigned)t->x, (unsigned)t->y, (unsigned)t->servo_us);
    if (n < 0 || (size_t)n >= sizeof body) {
        return 0u;
    }
    int m = snprintf(out, cap, "$%s*%02X\r\n", body, (unsigned)proto_checksum(body, (size_t)n));
    if (m < 0 || (size_t)m >= cap) {
        return 0u;
    }
    return (size_t)m;
}

void proto_rx_init(proto_rx_t *rx)
{
    rx->len = 0u;
    rx->active = 0u;
    rx->overflow = 0u;
}

static int hex_val(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static int parse_u16(const char *s, size_t n, uint16_t *out)
{
    uint32_t v = 0u;
    if (n == 0u || n > 5u) {
        return 0;
    }
    for (size_t i = 0u; i < n; i++) {
        if (s[i] < '0' || s[i] > '9') {
            return 0;
        }
        v = v * 10u + (uint32_t)(s[i] - '0');
    }
    if (v > 65535u) {
        return 0;
    }
    *out = (uint16_t)v;
    return 1;
}

/* line = "SV,1500*5A" (без '$' и без перевода строки) */
static int parse_line(const char *line, size_t len, proto_cmd_t *cmd)
{
    if (len < 4u || line[len - 3u] != '*') {
        return 0;
    }
    int hi = hex_val(line[len - 2u]);
    int lo = hex_val(line[len - 1u]);
    if (hi < 0 || lo < 0) {
        return 0;
    }
    size_t body_len = len - 3u;
    if (proto_checksum(line, body_len) != (uint8_t)((hi << 4) | lo)) {
        return 0;
    }
    if (body_len > 3u && memcmp(line, "SV,", 3u) == 0) {
        uint16_t us;
        if (!parse_u16(line + 3, body_len - 3u, &us)) {
            return 0;
        }
        cmd->type = PROTO_CMD_SERVO;
        cmd->servo_us = us;
        return 1;
    }
    return 0;
}

int proto_rx_feed(proto_rx_t *rx, uint8_t byte, proto_cmd_t *cmd)
{
    if (byte == (uint8_t)'$') {          /* любой '$' = начало нового кадра (ресинхронизация) */
        rx->len = 0u;
        rx->active = 1u;
        rx->overflow = 0u;
        return 0;
    }
    if (!rx->active || byte == (uint8_t)'\r') {
        return 0;
    }
    if (byte == (uint8_t)'\n') {
        rx->active = 0u;
        return rx->overflow ? 0 : parse_line(rx->buf, rx->len, cmd);
    }
    if (rx->len < PROTO_RX_MAX_LINE) {
        rx->buf[rx->len++] = (char)byte;
    } else {
        rx->overflow = 1u;
    }
    return 0;
}
