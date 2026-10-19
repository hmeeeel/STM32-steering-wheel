#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define PROTO_RX_MAX_LINE   32u
#define PROTO_TX_MAX        48u

typedef struct {
    uint16_t seq;
    uint32_t t_ms;
    uint16_t x;
    uint16_t y;
    uint16_t servo_us;
} proto_telemetry_t;

typedef enum { PROTO_CMD_NONE = 0, PROTO_CMD_SERVO } proto_cmd_type_t;

typedef struct {
    proto_cmd_type_t type;
    uint16_t         servo_us;
} proto_cmd_t;

typedef struct {
    char    buf[PROTO_RX_MAX_LINE];
    uint8_t len;
    uint8_t active;
    uint8_t overflow;
} proto_rx_t;

uint8_t proto_checksum(const char *s, size_t n);

size_t proto_format_telemetry(char *out, size_t cap, const proto_telemetry_t *t);

void proto_rx_init(proto_rx_t *rx);

int proto_rx_feed(proto_rx_t *rx, uint8_t byte, proto_cmd_t *cmd);

#endif /* PROTOCOL_H */
