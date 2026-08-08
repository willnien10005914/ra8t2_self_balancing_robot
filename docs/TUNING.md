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

## 建議控制架構（對齊常見自平衡 + Renesas FOC）

ChatGPT 分享連結需登入無法抓全文；實作採業界／Renesas AN 常見級聯：

```
AI/Console/RC → vx,wz
        ↓
外環 500Hz：LQR  或  PID（balmode 可切）
        ↓ 左右輪速度參考 (rpm)
內環 FSP FOC：速度 PI → 電流 PI (Id/Iq) → PWM
        ↑ Hall 電氣角 / 轉速
IMU → pitch / pitch_rate / yaw_rate
```

- **內環 PID/PI**：在 FSP `rm_motor_hall`（勿自己再疊一層電流環）
- **外環 LQR**：多狀態穩定 + 站定（預設）
- **外環 PID**：傾角 + 速度 lean + 偏航（參數少，適合先站起來）

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
5. 速度指令上限 **550 rpm**（`APP_CFG_RPM_CMD_MAX`）
6. 外環 `vx` 上限預設 1.5 m/s（遠低於額定 4.3 m/s）

## Bring-up

1. 懸空單輪 Hall FOC + RMW 調速度/電流 PI  
2. 雙輪 `speed` / `pos`  
3. `balmode pid` + 扶持站立  
4. 穩定後改 `balmode lqr` 並 offline 辨識覆寫 `BALANCE_LQR_K`
