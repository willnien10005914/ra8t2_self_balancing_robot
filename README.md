# RA8T2 雙輪 LQR 自我平衡機器人韌體

[![Repo](https://img.shields.io/badge/GitHub-ra8t2__self__balancing__robot-blue)](https://github.com/willnien10005914/ra8t2_self_balancing_robot)

以 **Renesas RA8T2 / MCK-RA8T2** + **RA Flexible Software Package (FSP)** + **e² studio** 為基礎的雙輪（hoverboard 型 BLDC + Hall）自穩韌體骨架。

> **重要：** 這是 *application framework*，**不是** clone 後一鍵就能 flash 的完整 e² studio 專案。  
> 正確流程是：用 Renesas 官方馬達 sample 建 FSP 專案 → 掛入本倉庫 `firmware/` → 接線 → 編譯燒錄。

| 能力 | 狀態 |
|------|------|
| 雙輪 Hall FOC / 扭矩路徑 τ→Iq* | 架構就緒，需接 FSP motor API |
| 外環 LQR / PID 可切 | 有原始碼 + 離線產 K |
| Console / AI UART·Ethernet | 有協定與接線說明 |
| clone 後直接 Build | **否**（需先有 FSP 專案殼） |

官方工具與硬體：

- [RA FSP 下載](https://www.renesas.com/en/software-tool/ra-flexible-software-package-fsp) / [GitHub Releases](https://github.com/renesas/fsp/releases)
- [RA8T2](https://www.renesas.com/en/products/ra8t2?tab=documentation)
- [MCK-RA8T2](https://www.renesas.com/en/design-resources/boards-kits/mck-ra8t2)
- [Hall 向量控制 AN（含 RA8T2 sample）](https://www.renesas.com/en/document/apn/vector-control-permanent-magnetic-synchronous-motor-hall-sensors-mckmcb-ra-family)

---

# 第一次使用 SOP（建議照順序做）

## 0. 你會拿到什麼 / 不會拿到什麼

**本倉庫有：**

- `firmware/src`、`firmware/include`：自穩、指令、安全、馬達抽象層
- `params/`、`tools/`：物理參數與 LQR K 產生器
- `docs/`：架構、協定、調參、五階段 bring-up
- `spec/`：6.5" 輪轂規格書、Renesas FOC AN PDF

**本倉庫沒有（需用 Renesas SDK 產生）：**

- e² studio 工程（`.cproject` / `.project`）
- FSP 組態 `ra_cfg/`、`ra_gen/`（由 Smart Configurator 產生）

---

## 1. Clone 本專案

```bash
git clone git@github.com:willnien10005914/ra8t2_self_balancing_robot.git
cd ra8t2_self_balancing_robot
```

HTTPS：

```bash
git clone https://github.com/willnien10005914/ra8t2_self_balancing_robot.git
```

---

## 2. 安裝 Renesas SDK（e² studio + FSP）

1. 到 [FSP Releases](https://github.com/renesas/fsp/releases) 下載 **FSP Platform Installer**（建議 **≥ v6.5.0**，內含 e² studio）。
2. macOS / Windows / Linux 依官網安裝；安裝時勾選 **RA8** 裝置支援。
3. 安裝後確認可開啟 **e² studio**，並能看到 FSP / RA Smart Configurator。
4. （選用）安裝 [Renesas Motor Workbench (RMW)](https://www.renesas.com/en/software-tool/renesas-motor-workbench) 調 FOC。

> 本應用原始碼是標準 C；**編譯與燒錄以 e² studio 為主**，不要指望只靠 `gcc` 單獨編完整 MCU 映像。

---

## 3. 用 e² studio 建立 / 匯入可編譯的「專案殼」

本倉庫本身不能當頂層工程直接 Open。請任選一種：

### 方法 A（建議）：匯入官方 MCK-RA8T2 Hall FOC Sample

1. 從 Renesas 下載 **Vector Control with Hall Sensors** sample（RA8T2 / MCK）。  
   文件入口：[Hall 向量控制 AN](https://www.renesas.com/en/document/apn/vector-control-permanent-magnetic-synchronous-motor-hall-sensors-mckmcb-ra-family)
2. e² studio → **File → Import → Existing Projects into Workspace** → 選 sample 專案目錄。
3. 確認 Board = **MCK-RA8T2**（或你的目標板）、Device = **RA8T2**。
4. 開啟 **FSP Configuration**（`configuration.xml`），確認已有 motor Hall stack；雙輪需兩個 motor 實例（見 `docs/FSP_STACK.md`）。

### 方法 B：新建空白 RA 專案再加 Motor Stack

1. **File → New → Renesas C/C++ Project → Renesas RA**  
2. 選 **MCK-RA8T2** / RA8T2，FreeRTOS 或 Baremetal。  
3. 在 Stacks 加入：
   - 2× `Motor Vector Control with hall sensors`（或官方雙馬達結構）
   - `r_iic_master`（IMU）
   - 1～2× `r_sci_uart`（Console / AI）
   - （可選）Ethernet + FreeRTOS+TCP  
4. **Generate Project Content**。

詳細模組表：[`docs/FSP_STACK.md`](docs/FSP_STACK.md)

---

## 4. 把本倉庫掛進 e² studio 專案（Build 必要）

假設：

- e² studio 專案在：`~/e2_studio/workspace/mck_ra8t2_foc/`
- 本倉庫在：`~/projects/ra8t2_self_balancing_robot/`

### 4.1 加入 Include / Source

在專案 **Properties → C/C++ Build → Settings**：

**Include paths** 加上（依相對路徑調整）：

```text
${ProjDirPath}/../../../projects/ra8t2_self_balancing_robot/firmware/include
```

或在專案旁邊放 symlink / 把 repo 放進 workspace 後用：

```text
${workspace_loc:/ra8t2_self_balancing_robot/firmware/include}
```

**Source** 把整個目錄加入編譯：

```text
.../ra8t2_self_balancing_robot/firmware/src
```

（e² studio：專案右鍵 → **New → Folder → Advanced → Link to alternate location**，連到 `firmware/src`、`firmware/include` 最省事。）

### 4.2 修改 `hal_entry.c`（或 `main` 入口）

```c
#include "app/app_main.h"

void hal_entry(void)
{
    /* 可先初始化 FSP motor / UART / I2C，再進應用 */
    app_main();   /* 不返回 */
}
```

### 4.3 實作弱符號板級埠（否則馬達/IMU 只是 stub）

見 [`firmware/fsp_notes/IMPORT.md`](firmware/fsp_notes/IMPORT.md)：

| 符號 | 接到 |
|------|------|
| `imu_i2c_write` / `imu_i2c_read` | `R_IIC_Master_*` |
| `console_uart_write` + RX→`console_rx_byte` | SCI UART |
| `ai_uart_write` / `ai_link_rx_byte` | 第二組 UART（或暫時合用） |
| `time_ms_set_port` | SysTick / GPT ms |
| `motor_port_*` | `RM_MOTOR_HALL_*` 左右輪 |
| `safety_port_read` / `safety_port_hw_shutdown` | ADC + **POEG** |

---

## 5. 硬體接線（第一次必看）

### 5.1 電源與安全

- Hoverboard 輪 / 逆變器：**36 V** 系統（規格書額定 36 V；適用範圍見 `spec/`）。
- **第一次務必輪子離地懸空**，旁站急停人員。
- PWM 硬關斷接 POEG（軟體 `estop` 不夠當唯一防護）。

### 5.2 馬達相線與 Hall（6.5" 規格書預設）

| 功能 | 線色（預設） |
|------|----------------|
| 相 U / V / W | 黃 / 藍 / 綠 |
| Hall VCC / HA / HB / HC / GND / TEMP | 紅 / 黃 / 藍 / 綠 / 黑 / 白 |
| 極對數 | **15** |
| Hall | **120°** |

FSP / RMW 務必設對 pole pairs 與 Hall 類型。順序不對會卡住發熱——見規格書「36 種換線」與 `docs/TUNING.md`。

### 5.3 IMU（LSM6DSK320X）

| 信號 | 接法 |
|------|------|
| SDA / SCL | RA8T2 I2C（`r_iic_master`） |
| VDD / GND | 3.3 V / GND |
| ADDR | 通常 0x6A 或 0x6B |

### 5.4 Console UART（人機指令）

| MCU | USB-UART |
|-----|----------|
| TX | RX |
| RX | TX |
| GND | GND |
| 鮑率 | **115200 8N1** |

### 5.5 AI Box（可稍後做）

- UART：見下文「AI Box」  
- Ethernet：`192.168.0.50:9000`（預設）

馬達 / 堆疊細節：[`docs/FSP_STACK.md`](docs/FSP_STACK.md)、[`docs/TUNING.md`](docs/TUNING.md)

---

## 6. 產生 LQR 增益（離線，可在 PC 做）

此步**不必**連板；改車體參數後要重跑。

```bash
cd ra8t2_self_balancing_robot
python3 -m pip install -r tools/requirements.txt

# 編輯物理參數（質量、COM、輪距、Kt…）
# vim params/robot_params.yaml

python3 tools/lqr_gain_gen.py
# 輸出：firmware/src/balance/balance_lqr_gain.c
```

然後在 e² studio **重新 Build**。  
預設檔已提交一組 bootstrap K；**實測參數前勿載人。**

---

## 7. 編譯（Build）

在 e² studio：

1. 選中 FSP 專案（含已連結的 `firmware/src`）。  
2. **Project → Build Project**（鐵槌）。  
3. 確認 0 Error。常見問題：
   - include path 沒加 `firmware/include`
   - `app_main` 未宣告／未連 source
   - FSP Generate 後未存檔

產物一般在專案 `Debug/*.elf`。

---

## 8. 燒錄（Flash）與除錯

1. MCK-RA8T2 用 USB 接電腦（板載 J-Link OB；或外接 J-Link）。  
2. e² studio → **Run → Debug Configurations** → **Renesas GDB Hardware Debugging**（或專案內建 Debug）。  
3. 選正確 Device（RA8T2）與 Debugger。  
4. **Debug** → 下載（Download）→ Reset → Run。  
5. 另開 Serial Terminal（115200）看開機訊息：
   ```text
   ra8t2 balance boot
   wheel 6.5in ...
   balance actuator=TORQUE (tau->Iq*)
   ```

也可用 J-Link Commander / RTT，但首次建議 e² studio Debug。

---

## 9. 上電後第一次指令（安全）

輪子**離地**：

```text
help
status
balmode off
lqr off
mode both speed
speed both 60
```

確認轉動正確後再：

```text
balmode pid
lqr on
status
brake
```

完整五階段：[`docs/BRINGUP_PHASES.md`](docs/BRINGUP_PHASES.md)

---

## 10. SOP 總覽檢查清單

- [ ] Clone 倉庫  
- [ ] 安裝 FSP + e² studio（≥ 6.5）  
- [ ] 匯入 / 新建 MCK-RA8T2 Hall FOC 專案並 Generate  
- [ ] Link `firmware/include` + `firmware/src`  
- [ ] `hal_entry` → `app_main()`  
- [ ] 實作 I2C / UART / motor_port / safety POEG hooks  
- [ ] 接 36V、相線、Hall、IMU、Console（輪離地）  
- [ ] （可選）`python3 tools/lqr_gain_gen.py`  
- [ ] Build → Debug 燒錄  
- [ ] Console 先 `speed` 調機，再 `balmode pid`  

更細的 import 勾選：[`firmware/fsp_notes/IMPORT.md`](firmware/fsp_notes/IMPORT.md)

---

# 倉庫目錄

```
docs/ARCHITECTURE.md      控制階層、時序、狀態機
docs/PROTOCOL.md          Console / AI / Ethernet / RC 協定
docs/FSP_STACK.md         FSP stacks 與腳位建議
docs/TUNING.md            輪規 / FOC / PID / 扭矩調參
docs/BRINGUP_PHASES.md    五階段 bring-up
params/robot_params.yaml  物理參數（LQR 產生器輸入）
tools/lqr_gain_gen.py     離線產 balance_lqr_gain.c
spec/                     輪轂規格 + FOC AN PDF
firmware/include|src      應用韌體
firmware/fsp_notes/       匯入 e² studio 清單
```

---

# Console 指令

**接線：** MCU TX→USB-UART RX，RX←TX，GND，**115200 8N1**。

| 指令 | 說明 |
|------|------|
| `help` | 列出指令 |
| `status` | 狀態 / pitch / Iq / 模式 / safety mask |
| `imu` / `pos` | IMU / 輪位置 |
| `mode` / `speed` / `posset` | 逐輪調機（建議平衡 off） |
| `lqr on\|off` | 啟停自穩外環 |
| `balmode lqr\|pid\|off` | 外環演算法 |
| `pid` / `pid pitch\|vel\|yaw kp ki kd` | PID 增益 |
| `fwd` / `back` / `left` / `right` / `turn` / `brake` | 運動（平衡 on） |
| `estop` / `clear` | 急停 / 清 fault |
| `ai on\|off` / `rc on\|off` | AI / RC 閘控 |

```text
status
balmode pid
lqr on
fwd 0.3
brake
```

平衡開啟時請只下 `vx/wz` 類指令，不要用 `speed` 硬推輪。

---

# AI Box 接線與指令

UART 或 Ethernet，**同一二元 frame**。詳見 [`docs/PROTOCOL.md`](docs/PROTOCOL.md)。

**UART：** AI TX→MCU RX，RX←TX，GND，3.3V，115200。  
**Ethernet：** TCP `192.168.0.50:9000`（`app_cfg.h` 可改）。

```text
0xAA 0x55 | len | type | payload | crc8(poly 0x07)
type 0x10 = MOTION(float vx, float wz)
```

---

# 控制與安全（摘要）

```text
AI/Console → vx,wz → LQR/PID → τL/τR → Iq*=τ/Kt → FOC 電流環 → PWM
```

- 扭矩模式：`APP_CFG_BALANCE_USE_TORQUE=1`（預設）  
- 調機速度模式：改為 `0`  
- 安全：傾角 / IMU / UV·OV·OC·OT / ESTOP；硬體需接 POEG  

優先權：ESTOP > Console 直接馬達 > Console 運動 > AI > RC > Hold。

---

# 授權

學習與原型開發用途。Issue / PR：  
https://github.com/willnien10005914/ra8t2_self_balancing_robot
