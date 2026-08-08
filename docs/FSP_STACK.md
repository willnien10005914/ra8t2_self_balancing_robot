# FSP Stack Mapping（MCK-RA8T2）

在 e² studio → FSP Configuration 建議組態（名稱可自訂，與 `motor_port_fsp.c` 對齊）。

## Motor（雙 Hall FOC）

以官方 **Vector Control with Hall Sensors** sample（RA8T2 / MCK）為基板，複製為雙實例：

| Stack | Instance | Notes |
|-------|----------|-------|
| Motor Vector Control with hall sensors | `g_motor_hall_l` | 左輪 |
| Motor Vector Control with hall sensors | `g_motor_hall_r` | 右輪 |
| Motor Angle/Speed Hall | per motor | `rm_motor_sense_hall` |
| GPT PWM 三相 / ADC 電流 / POEG | per board | 依 MCK 馬達樣本 |

腳位、電流縮放、極對數依 hoverboard 輪重測。Motor Workbench 先穩單輪。

## IMU

| Stack | Config |
|-------|--------|
| `r_iic_master` | LSM6DSK320X（ADDR 0x6A/0x6B） |
| Optional GPT / ICU | 1 kHz tick 或 DRDY |

## UART

| Stack | Use |
|-------|-----|
| `r_sci_uart` `g_uart_console` | ASCII console |
| `r_sci_uart` `g_uart_ai` | AI binary |

啟用 RX 回呼 → `console_rx_byte` / `ai_link_rx_byte`。

## Ethernet（建議正式 AI Box）

RA8T2 支援 Gigabit Ethernet / TSN。建議：

| Stack | Use |
|-------|-----|
| `r_ether` 或 `r_rmac`（依 FSP/裝置） | MAC |
| FreeRTOS+TCP 或 lwIP | TCP/IP |
| FreeRTOS | 網路與指令 task |

預設靜態 IP／埠見 `APP_CFG_NET_*`（`app_cfg.h`）。

1. 鏈路 up 後 `net_port_init()` listen `:9000`
2. `accept` → `recv` → `net_link_rx_buffer()`
3. `net_port_send()` 回遙測

參考實作：`firmware/src/cmd/net_port_fsp_example.c`（編譯時加 `-DAPP_USE_FSP_NET_PORT=1`）。

## OS 選擇

- **Baremetal**：先 bring-up UART / 馬達（Ethernet 較吃力）
- **FreeRTOS**：正式版；`BalanceTask` 500 Hz；`NetTask` / `CmdTask` 跑 socket

## 整合

```c
#include "app/app_main.h"
void hal_entry(void) { app_main(); }
```

Include：`firmware/include`；Source：`firmware/src/**`。

## 燒錄

MCK-RA8T2 J-Link → Download → UART/`status` 驗證。勿在 FOC 運轉中熱插 Hall／功率線。
