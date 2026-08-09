# 控制調參與 Spec 對應

## 規格來源

| 檔案 | 用途 |
|------|------|
| `spec/6.5 inch (Hall sensor) 1.pdf` | 6.5" 輪轂參數（金海芯） |
| `spec/ra8t2-sensorless-foc.pdf` | Renesas FOC AN R01AN6839（速度 PI + 電流 PI） |

## 輪轂關鍵參數（已寫入 `motor_params.h`）

- 直徑 ≈ 165mm → 半徑 **0.0825 m**
- 額定 **36 V / 250 W / 500±50 rpm / 5±0.5 A**
- 最大扭矩 **8.9 N·m**，磁極 **15 對極**，霍爾 **120°**
- 相線：黃 U / 藍 V / 綠 W；霍爾：紅 VCC、黃 HA、藍 HB、綠 HC、黑 GND、白溫控
- 適用電壓範圍 DC24–100 V（建議以 36 V 系統 bring-up）

## 建議控制架構（對齊 GPT Hoverboard 建議 + Renesas FOC）

```
AI/Console/RC → vx,wz
        ↓
外環 500Hz：LQR  或  PID（balmode 可切）
        ↓ 左右輪扭矩 τL/τR (N·m)   ← 正式路徑（APP_CFG_BALANCE_USE_TORQUE=1）
        ↓ Iq* = τ / Kt
內環 FSP FOC：電流 PI (Id=0, Iq*) → PWM   （旁路速度環）
        ↑ Hall 電氣角 / 轉速
IMU → pitch / pitch_rate / yaw_rate
```

調機時可設 `APP_CFG_BALANCE_USE_TORQUE=0` 改走速度模式。

- **內環電流 PI**：FSP `rm_motor_hall`（扭矩模式只下 Iq*）
- **Kt**：`MOTOR_SPEC_KT_NM_PER_A`（由額定估算 ≈0.955，臺架後覆寫）
- **外環 LQR**：輸出共同/差速扭矩（非 rpm）
- **外環 PID**：傾角→τ，速度→lean，偏航→差速 τ

## Console

```text
balmode lqr|pid|off
lqr on|off
pid                         # 顯示外環增益 + FOC 建議
pid pitch <kp> <ki> <kd>
pid vel <kp> <ki> <kd>
pid yaw <kp> <ki> <kd>
```

## FSP 設定檢查清單

1. Pole pairs = **15**
2. Hall 120° electrical
3. Speed loop ~1 kHz；Current / PWM ~20 kHz（見 FOC AN）
4. 過流門檻先用 ≤ **12 A**（`MOTOR_SPEC_PEAK_CURRENT_A`），額定區 5 A
5. 速度指令上限由 **0.5 m/s × 輪徑** 換算 rpm（約 58 rpm @ 6.5"）——`APP_CFG_RPM_CMD_MAX`
6. 外環 `vx` 硬上限預設 **0.50 m/s**（`APP_CFG_SPEED_SAFETY_LOCK=1`）；起步線性 ax=0.25 m/s²
7. 解禁／調加速度：見 README「速度安全鎖」；須改 `app_cfg.h` 重編，runtime 不可解

## Bring-up

見 **`docs/BRINGUP_PHASES.md`**（五階段：馬達 → IMU → PID → LQR 辨識 → 載人）。

### 產生 / 更新 LQR K

```bash
pip install -r tools/requirements.txt
# 編輯 params/robot_params.yaml（質量、COM、慣量、Kt…）
python3 tools/lqr_gain_gen.py
# 寫入 firmware/src/balance/balance_lqr_gain.c 後重新編譯
```

### 安全閾值（36V）

| 檢查 | 預設 |
|------|------|
| UV | 30 V |
| OV | 43 V |
| Ibus OC | 25 A |
| OT | 70 °C |
| 馬達 Iq | ≤ 12 A |

實作 `safety_port_read`（ADC）與 `safety_port_hw_shutdown`（POEG）。

