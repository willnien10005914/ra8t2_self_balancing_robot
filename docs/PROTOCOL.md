# 通訊協定 — Console / AI Box（UART・Ethernet）/ RC

所有來源經 `cmd_arbiter` 統合為 `motion_cmd_t`。

## 優先權（高 → 低）

1. `ESTOP` / fault clear（任何來源）
2. Console **直接馬達**（`mode`/`speed`/`posset`，僅 LQR off）
3. Console 運動（`fwd`/`left`/`brake`/`lqr`）
4. AI box（UART **或** Ethernet，相同 frame）
5. RC stream（預留）
6. Hold（無指令：`vx=0,wz=0`；LQR on 則自穩）

---

## UART A — 人機 Console（ASCII）

115200 8N1，行尾 `\n`。

| 指令 | 作用 |
|------|------|
| `help` | 列出指令 |
| `status` | 傾角、輪位置/速度、LQR、fault |
| `imu` | 原始 accel/gyro + pitch |
| `pos` | 左右輪 hall counts / rad |
| `mode <l\|r\|both> <speed\|position\|torque>` | 逐輪模式（建議 LQR off） |
| `speed <l\|r\|both> <rpm>` | 速度參考 |
| `posset <l\|r\|both> <counts>` | 位置參考 |
| `lqr <on\|off>` | 自穩開關 |
| `fwd <m_s>` / `back <m_s>` | 線速度 |
| `left <rad_s>` / `right <rad_s>` / `turn <rad_s>` | 差速轉向 |
| `brake` | `vx=wz=0`，**保持 LQR** |
| `estop` / `clear` | 緊急停機 / 清 fault |
| `ai <on\|off>` / `rc <on\|off>` | 閘控 AI / RC |

---

## AI 共用 Frame（UART B 與 Ethernet）

Little-endian：

```
0xAA 0x55 | len | type | payload[len] | crc8
```

- `len`：payload 長度（不含 type）
- `crc8`：poly 0x07，涵蓋 `len..payload`

| type | payload | 意義 |
|------|---------|------|
| `0x01` HEARTBEAT | u32 ms | 保活；逾時 → HOLD |
| `0x10` MOTION | float vx, float wz | m/s, rad/s |
| `0x11` BRAKE | — | 同 console brake |
| `0x12` STOP_FOLLOW | — | 清跟隨 |
| `0x20` LQR | u8 on | 1=on 0=off |
| `0x7E` ESTOP | — | FAULT_SAFE |
| `0x80` TELEM_REQ | — | 要求遙測 |
| `0x81` TELEM_RESP | pitch,vx,wz,fault | MCU→AI |

左右轉為差速（非橫移）。目標偏左 → 正 `wz`。

### UART 接線

AI TX→MCU RX、AI RX←MCU TX、GND 共地、3.3V TTL、115200。RX → `ai_link_rx_byte()`。

### Ethernet 接線（RA8T2 GbE）

預設（`app_cfg.h`）：

| 項目 | 值 |
|------|-----|
| IP | 192.168.0.50 |
| TCP/UDP 埠 | 9000 |
| 遮罩 / GW | 255.255.255.0 / 192.168.0.1 |

1. 網路線接 AI Box 與 MCU（同 switch 或直連）。
2. AI `TCP connect(192.168.0.50, 9000)`，送出與 UART 相同的 frame。
3. FSP：`r_ether` + FreeRTOS+TCP；recv → `net_link_rx_buffer()`。
4. 範例 port：`firmware/src/cmd/net_port_fsp_example.c`（`APP_USE_FSP_NET_PORT`）。

TCP 適合可靠指令；高頻 MOTION 也可用 UDP 同埠，payload 仍為完整 frame。

---

## RC（預留）

`cmd/rc_link.*`：CRSF/SBUS → 正規化 `vx,wz`。丟訊 timeout 同 AI → HOLD。

## 逾時

| 來源 | 預設 | 逾時行為 |
|------|------|----------|
| AI MOTION（UART/ETH） | 300 ms | HOLD（brake，LQR 維持） |
| RC | 200 ms | HOLD |
| Console 運動 | 不逾時 | 直到 `brake` / 改值 |
