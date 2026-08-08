#include "cmd/net_link.h"
#include "cmd/ai_link.h"
#include "cmd/cmd_arbiter.h"
#include "app/app_cfg.h"
#include "util/util.h"

#include <string.h>

/*
 * Ethernet AI 通道：協定與 UART AI 相同（0xAA 0x55 ...）。
 * 實際 socket / r_ether / FreeRTOS+TCP / lwIP 在板級 port：
 *   - net_port_init()
 *   - net_port_poll()
 *   - net_port_send()
 * 收到資料後呼叫 net_link_rx_buffer()。
 */

static bool s_connected;
static uint32_t s_last_rx_ms;

/* --- weak board port (接 FSP Ethernet) --- */

__attribute__((weak)) bool net_port_init(void)
{
    return true; /* 未接 PHY 時仍允許建置；連線功能無効 */
}

__attribute__((weak)) void net_port_poll(void)
{
}

__attribute__((weak)) bool net_port_send(const uint8_t *data, uint32_t len)
{
    (void)data;
    (void)len;
    return false;
}

__attribute__((weak)) void net_port_set_connected(bool connected)
{
    s_connected = connected;
}

void net_link_init(void)
{
    s_connected = false;
    s_last_rx_ms = 0;
    (void)net_port_init();
}

bool net_link_client_connected(void)
{
    return s_connected;
}

void net_link_rx_byte(uint8_t b)
{
    /* 復用 UART AI 解析狀態機會導致交錯；改獨立解析或單消費者。
     * 此處呼叫 ai 同型 frame 路徑：轉成 buffer 單元解析較安全。
     * 簡化：逐 byte 餵獨立小型狀態機 — 直接複用 ai_link_rx_byte 僅能單一來源。
     * 多來源時請只啟用 UART 或 ETH 其一喂 cmd；或由 port 串行化。
     */
    s_last_rx_ms = time_ms_get();
    s_connected = true;
    ai_link_rx_byte(b);
}

void net_link_rx_buffer(const uint8_t *data, uint32_t len)
{
    if (!data) return;
    s_last_rx_ms = time_ms_get();
    s_connected = true;
    for (uint32_t i = 0; i < len; ++i)
    {
        ai_link_rx_byte(data[i]);
    }
}

void net_link_send_telemetry(float pitch, float vx, float wz, uint16_t fault)
{
    /* 與 ai_link_send_telemetry 相同 frame，改走 Ethernet */
    uint8_t frame[3 + 14 + 1];
    uint8_t pay[14];
    memcpy(&pay[0], &pitch, 4);
    memcpy(&pay[4], &vx, 4);
    memcpy(&pay[8], &wz, 4);
    pay[12] = (uint8_t)(fault & 0xFF);
    pay[13] = (uint8_t)(fault >> 8);

    frame[0] = AI_SYNC0;
    frame[1] = AI_SYNC1;
    frame[2] = 14;
    frame[3] = AI_TYPE_TELEM_RESP;
    memcpy(&frame[4], pay, 14);
    {
        uint8_t tmp[16];
        tmp[0] = 14;
        tmp[1] = AI_TYPE_TELEM_RESP;
        memcpy(&tmp[2], pay, 14);
        frame[18] = crc8_maxim(tmp, 16);
    }
    if (s_connected)
    {
        (void)net_port_send(frame, sizeof(frame));
    }
}

void net_link_poll(uint32_t now_ms)
{
    net_port_poll();
    if (s_connected && s_last_rx_ms != 0u &&
        (now_ms - s_last_rx_ms) > (APP_CFG_AI_TIMEOUT_MS * 5u))
    {
        /* 很久無資料：標記斷線（指令逾時仍由 cmd_arbiter 對 AI stamp 處理） */
        s_connected = false;
    }
}
