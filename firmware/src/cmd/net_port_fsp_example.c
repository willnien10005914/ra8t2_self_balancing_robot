/**
 * @file net_port_fsp_example.c
 * @brief FSP FreeRTOS+TCP / lwIP 接線範例（預設不編進專案亦可）。
 *
 * 建議流程：
 * 1. FSP 加入 r_ether / r_rmac + FreeRTOS+TCP（或 lwIP）
 * 2. 靜態 IP 或 DHCP（見 README）
 * 3. TCP listen APP_CFG_NET_TCP_PORT（預設 9000）
 * 4. accept 後 recv → net_link_rx_buffer()
 * 5. net_port_send → send()
 *
 * UDP：綁定同埠，payload 直接是 AI frame（無額外封裝）。
 */

#include "cmd/net_link.h"
#include "app/app_cfg.h"

#if defined(APP_USE_FSP_NET_PORT) && (APP_USE_FSP_NET_PORT)

#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"

static Socket_t s_listen;
static Socket_t s_client = FREERTOS_INVALID_SOCKET;

bool net_port_init(void)
{
    struct freertos_sockaddr addr;
    s_listen = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_STREAM, FREERTOS_IPPROTO_TCP);
    if (s_listen == FREERTOS_INVALID_SOCKET)
    {
        return false;
    }
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = FREERTOS_AF_INET;
    addr.sin_port = FreeRTOS_htons(APP_CFG_NET_TCP_PORT);
    addr.sin_address.ulIP_IPv4 = 0;
    FreeRTOS_bind(s_listen, &addr, sizeof(addr));
    FreeRTOS_listen(s_listen, 1);
    return true;
}

void net_port_poll(void)
{
    uint8_t buf[256];
    if (s_client == FREERTOS_INVALID_SOCKET)
    {
        struct freertos_sockaddr cli;
        uint32_t len = sizeof(cli);
        Socket_t c = FreeRTOS_accept(s_listen, &cli, &len);
        if (c != FREERTOS_INVALID_SOCKET)
        {
            s_client = c;
            TickType_t to = pdMS_TO_TICKS(10);
            FreeRTOS_setsockopt(s_client, 0, FREERTOS_SO_RCVTIMEO, &to, sizeof(to));
        }
        return;
    }
    BaseType_t n = FreeRTOS_recv(s_client, buf, sizeof(buf), 0);
    if (n > 0)
    {
        net_link_rx_buffer(buf, (uint32_t)n);
    }
    else if (n < 0)
    {
        FreeRTOS_closesocket(s_client);
        s_client = FREERTOS_INVALID_SOCKET;
    }
}

bool net_port_send(const uint8_t *data, uint32_t len)
{
    if (s_client == FREERTOS_INVALID_SOCKET) return false;
    return FreeRTOS_send(s_client, data, len, 0) == (BaseType_t)len;
}

#endif /* APP_USE_FSP_NET_PORT */
