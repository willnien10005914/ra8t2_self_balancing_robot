#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * AI box / 遙控上層 — Ethernet 通道（RA8T2 支援 GbE）。
 * 預設：TCP Server，埠 APP_CFG_NET_TCP_PORT，frame 與 UART AI 二元協定相同。
 * 另可啟用 UDP 同埠收 MOTION（見 docs/PROTOCOL.md）。
 */
void net_link_init(void);
void net_link_poll(uint32_t now_ms);

/** 由 FSP/FreeRTOS+TCP / lwIP 收到的位元組餵入（與 ai_link 同 frame）。 */
void net_link_rx_byte(uint8_t b);
void net_link_rx_buffer(const uint8_t *data, uint32_t len);

/** 透過目前 TCP client 回傳遙測；未連線則忽略。 */
void net_link_send_telemetry(float pitch, float vx, float wz, uint16_t fault);

bool net_link_client_connected(void);

#ifdef __cplusplus
}
#endif
