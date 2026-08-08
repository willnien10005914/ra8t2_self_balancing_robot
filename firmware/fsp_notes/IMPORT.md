# 匯入 e² studio / FSP 檢查清單（中文）

搭配 README「第一次使用 SOP」。

## 前置

1. 安裝 [FSP ≥ 6.5.0 Platform Installer](https://github.com/renesas/fsp/releases)（含 e² studio）。
2. 準備 [MCK-RA8T2](https://www.renesas.com/en/design-resources/boards-kits/mck-ra8t2) 或等效 RA8T2 馬達板。
3. 下載並 Import **Hall 向量控制** sample（RA8T2）。

## 掛載本倉庫

1. 將 `firmware/include`、`firmware/src` **Link** 進 FSP 專案（或加 Include / Source path）。
2. `hal_entry.c`（或等價入口）呼叫 `app_main()`。
3. **Generate Project Content** 後再 Build。

## 必須實作的板級函式（weak 預設為空/假資料）

| 函式 | 用途 |
|------|------|
| `imu_i2c_write` / `imu_i2c_read` | LSM6DSK320X |
| `console_uart_write`；RX 呼叫 `console_rx_byte` | 人機 115200 |
| `ai_uart_write`；RX 呼叫 `ai_link_rx_byte` | AI UART |
| `time_ms_set_port` | 系統毫秒 |
| `motor_port_open/run/stop/speed_set/torque_set/read_fb` | 接 `RM_MOTOR_HALL_*` |
| `safety_port_read` | VBUS/IBUS/TEMP/相電流 ADC |
| `safety_port_hw_shutdown` | **POEG / PWM 硬關斷** |

## 馬達參數（FSP / RMW）

- Pole pairs = **15**
- Hall = **120°**
- 額定約 36 V / 5 A；峰值電流門檻先 ≤ 12 A
- 平衡模式預設下 **Iq\***（扭矩），不是 rpm

## 離線產 LQR（PC）

```bash
python3 -m pip install -r tools/requirements.txt
python3 tools/lqr_gain_gen.py
```

## 官方文件

- https://renesas.github.io/fsp/group___m_o_t_o_r___h_a_l_l.html
- https://www.renesas.com/en/design-resources/boards-kits/mck-ra8t2
- https://github.com/renesas/fsp/releases
