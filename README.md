# RA8T2 雙輪 LQR 自我平衡機器人韌體

[![Repo](https://img.shields.io/badge/GitHub-ra8t2__self__balancing__robot-blue)](https://github.com/willnien10005914/ra8t2_self_balancing_robot)

以 **Renesas RA8T2 / MCK-RA8T2** + **RA Flexible Software Package (FSP)** 為基礎的雙輪（hoverboard 型 BLDC + Hall）自穩韌體骨架。

- 內環：Hall FOC **電流環**（平衡時 τ→Iq*；調機可改速度模式）
- 外環：**可切換 LQR / PID**（`balmode`）；開機讀 I2C IMU
- 輪規：6.5" Hall 輪轂 36V/250W/500rpm/15 對極 / Kt≈0.955（`spec/`）
- 指令：UART Console、AI Box（UART 或 **Ethernet**）、預留遙控；`brake` 後繼續自穩並站定

**可以燒錄：** 用 e² studio + FSP 建 MCK-RA8T2 專案，掛入本 repo 的 `firmware/`，接上官方馬達 sample，即可編譯並以 J-Link 下載。

官方參考：

- [RA8T2](https://www.renesas.com/en/products/ra8t2?tab=documentation)（含 GbE / TSN）
- [RA FSP](https://www.renesas.com/en/software-tool/ra-flexible-software-package-fsp)
- [MCK-RA8T2](https://www.renesas.com/en/design-resources/boards-kits/mck-ra8t2)

---

## 倉庫目錄

```
docs/ARCHITECTURE.md      控制階層、時序、狀態機
docs/PROTOCOL.md          Console / AI / Ethernet / RC 協定詳述
docs/FSP_STACK.md         e² studio FSP 模組與腳位建議
docs/TUNING.md            依輪規 / FOC AN 調參（含 PID＋LQR）
spec/                     6.5" 輪轂規格書 + RA8T2 FOC AN PDF
firmware/include/         公開標頭
firmware/src/             應用層原始碼
firmware/fsp_notes/       匯入 e² studio 檢查清單
```

e² studio 產生的 `ra_gen/`、`ra_cfg/` **不進版控**（見 `.gitignore`）。

---

## 快速開始（編譯與燒錄）

1. 安裝 [FSP ≥ 6.5.0](https://github.com/renesas/fsp/releases)（含 e² studio）。
2. 匯入 MCK-RA8T2 **Hall 向量控制** sample（或雙馬達 sample 後改 Hall）。
3. 依 `docs/FSP_STACK.md` 加入 I2C（IMU）、UART×2、可選 Ethernet（`r_ether` / FreeRTOS+TCP）。
4. 專案加入本 repo：
   - Include：`firmware/include`
   - Source：`firmware/src`
5. `hal_entry.c` 呼叫 `app_main()`。
6. Build → Debug（J-Link）→ Download。

詳細 hook 見 `firmware/fsp_notes/IMPORT.md`。

---

## Console 指令用法（UART 人機）

**接線建議**

| 信號 | 說明 |
|------|------|
| MCU TX | → USB-UART RX（電腦） |
| MCU RX | ← USB-UART TX |
| GND | 共地 |
| 鮑率 | **115200 8N1**，行尾 `\n` |

用 `screen` / `minicom` / e² studio Terminal 開啟後，可輸入：

| 指令 | 說明 |
|------|------|
| `help` | 列出指令 |
| `status` | 傾角、輪速、LQR、目前 `vx/wz` |
| `imu` | 加速度／陀螺儀／pitch |
| `pos` | 左右輪 Hall 計數與弧度 |
| `mode <l\|r\|both> <speed\|position\|torque>` | 逐輪模式（**建議 LQR off**） |
| `speed <l\|r\|both> <rpm>` | 速度模式參考 |
| `posset <l\|r\|both> <counts>` | 位置模式參考 |
| `lqr on` / `lqr off` | 啟停自穩（外環） |
| `balmode lqr\|pid\|off` | 外環演算法切換 |
| `pid` / `pid pitch\|vel\|yaw <kp> <ki> <kd>` | 讀／寫外環 PID；並提示 FOC PI |
| `fwd <m/s>` | 前進（平衡 on 時為參考速度） |
| `back <m/s>` | 後退 |
| `left <rad/s>` / `right <rad/s>` | 差速轉向（兩輪車轉彎，非橫移） |
| `turn <rad/s>` | +左 / −右 |
| `brake` | `vx=wz=0`，**維持 LQR 自穩** |
| `estop` | 緊急停機（PWM off） |
| `clear` | 清除 fault（需先直立安全） |
| `ai on\|off` | 是否接受 AI box（UART/Ethernet） |
| `rc on\|off` | 預留遙控 |

**建議操作流程**

```text
status
lqr on
fwd 0.3
left 0.4
brake
status
```

啟用 LQR 後，上層（人機／AI／遙控）應只下達 **前進／後退／轉向／煞車**，不要用 `speed`/`posset` 直接硬推輪子。

---

## AI Box 接線與指令

AI Box（視覺跟隨）與 RA8T2 可用 **UART** 或 **Ethernet**。兩者使用**同一套二元 frame**（見下方）。同時只建議一邊當主要即時控制源，另一邊可關：`ai off` 或拔線。

### 方案 A — UART（低延遲、接線簡單）

| 信號 | AI Box | RA8T2 |
|------|--------|-------|
| TX | AI TX | MCU SCI RX（AI UART） |
| RX | AI RX | MCU SCI TX |
| GND | 共地 | 共地 |
| 電平 | 3.3V TTL（必要時加電平轉換） | 3.3V |
| 鮑率 | 115200 8N1（可於 FSP 調整） | 同左 |

程式路徑：UART RX IRQ → `ai_link_rx_byte()` → `cmd_arbiter`。

### 方案 B — Ethernet（RA8T2 原生支援）

[RA8T2](https://www.renesas.com/en/products/ra8t2) 內建 **Gigabit Ethernet（含 TSN switch）**，適合 AI Box 走網路線。

| 項目 | 建議預設（可改 `app_cfg.h`） |
|------|------------------------------|
| 介面 | 板載 RJ45 / PHY（依 MCK 或自製板） |
| IP | 靜態 `192.168.0.50`（`APP_CFG_NET_STATIC_IP`） |
| 遮罩 | `255.255.255.0` |
| 閘道 | `192.168.0.1` |
| TCP | Server，埠 **9000**（`APP_CFG_NET_TCP_PORT`） |
| UDP | 同埠 **9000**（可選，payload 即 frame） |

接線：

1. AI Box 與 MCU 接同一交換器／直連（直連時注意交叉或自動 MDI-X）。
2. AI Box 設同網段 IP，例如 `192.168.0.20`。
3. TCP 連線：`192.168.0.50:9000`，連上後送 frame 即可。
4. FSP 需啟用 `r_ether`/`r_rmac` + FreeRTOS+TCP（或 lwIP）；實作參考 `firmware/src/cmd/net_port_fsp_example.c`（定義 `APP_USE_FSP_NET_PORT=1`）。

應用層：`net_link_rx_buffer()` / `net_link_poll()`（見 `firmware/src/cmd/net_link.c`）。

### AI 共用二元協定（UART / Ethernet 相同）

Little-endian：

```text
0xAA 0x55 | len | type | payload[len] | crc8
```

- `crc8`：poly `0x07`，涵蓋 `len + type + payload`（不含 sync）

| type | payload | 意義 |
|------|---------|------|
| `0x01` | u32 ms | HEARTBEAT；逾時則 HOLD（站定自穩） |
| `0x10` | float vx, float wz | MOTION：m/s、rad/s |
| `0x11` | — | BRAKE |
| `0x12` | — | STOP_FOLLOW |
| `0x20` | u8 on | LQR on/off |
| `0x7E` | — | ESTOP |
| `0x80` | — | 要求遙測 |
| `0x81` | pitch,vx,wz,fault | 遙測回覆（MCU→AI） |

視覺邏輯建議：

- 目標偏遠 → 正 `vx`
- 目標偏左 → 正 `wz`（差速左轉）
- 遺失目標 → `BRAKE` / `STOP_FOLLOW`（保持 LQR）

AI MOTION 逾時預設 **300 ms** → 自動 `brake`（自穩不關）。詳見 `docs/PROTOCOL.md`。

### Python 範例（Ethernet TCP）

```python
import socket, struct, zlib  # crc 請用協定 poly 0x07 自實作

def crc8(data: bytes) -> int:
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if (c & 0x80) else ((c << 1) & 0xFF)
    return c

def motion_frame(vx: float, wz: float) -> bytes:
    payload = struct.pack("<ff", vx, wz)
    body = bytes([len(payload), 0x10]) + payload
    return b"\xAA\x55" + body + bytes([crc8(body)])

s = socket.create_connection(("192.168.0.50", 9000), timeout=1.0)
s.sendall(motion_frame(0.3, 0.2))   # 前進並左轉
s.sendall(b"\xAA\x55" + bytes([0, 0x11, crc8(bytes([0, 0x11]))]))  # brake
```

---

## 控制優先權（簡表）

1. ESTOP / fault  
2. Console 直接馬達模式（僅 LQR off）  
3. Console 運動指令  
4. AI（UART 或 Ethernet）  
5. RC（預留）  
6. Hold（`vx=wz=0`，LQR on 則自穩）

---

## Bring-up 順序

1. 單輪 Hall FOC（Motor Workbench）  
2. 雙輪 `speed` / `pos` + console  
3. IMU 姿態 200–500 Hz  
4. LQR 靜止自穩 → `brake`  
5. 差速跟隨 + AI UART  
6. 接上 Ethernet AI / 遙控仲裁  

---

## 安全注意

- 傾角超限、IMU 失聯、過流 → FAULT（PWM off）  
- 首次上電請架車或用手扶持再 `lqr on`  
- AI / 網路斷線逾時 → `brake` 站定，不是斷電摔倒（除非進 ESTOP）  

---

## 授權與貢獻

專案用途為學習與原型開發。PR / Issue 請開在  
https://github.com/willnien10005914/ra8t2_self_balancing_robot
