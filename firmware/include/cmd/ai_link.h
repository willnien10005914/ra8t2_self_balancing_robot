#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AI_SYNC0  (0xAAu)
#define AI_SYNC1  (0x55u)

enum {
    AI_TYPE_HEARTBEAT     = 0x01,
    AI_TYPE_MOTION        = 0x10,
    AI_TYPE_BRAKE         = 0x11,
    AI_TYPE_STOP_FOLLOW   = 0x12,
    AI_TYPE_LQR           = 0x20,
    AI_TYPE_ESTOP         = 0x7E,
    AI_TYPE_TELEM_REQ     = 0x80,
    AI_TYPE_TELEM_RESP    = 0x81
};

void ai_link_init(void);
void ai_link_rx_byte(uint8_t b);
void ai_link_poll(uint32_t now_ms);
void ai_link_send_telemetry(float pitch, float vx, float wz, uint16_t fault);

#ifdef __cplusplus
}
#endif
