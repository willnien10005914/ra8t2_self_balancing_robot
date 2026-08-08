# Hoverboard / RA8T2 五階段 Bring-up

依 GPT 建議：**先讓馬達與符號正確，再 PID，再 LQR，最後才載人。**

## Phase 1 — Motor bring-up（不要平衡）

目標：兩顆 6.5" Hall 輪可穩低速轉。

1. 依 `motor_params.h` / 規格書接相線與 Hall（15 對極、120°）。
2. FSP `rm_motor_hall` + RMW 調電流 PI。
3. Console（`lqr off` / `balmode off`）：
   ```text
   mode both speed
   speed both 60
   ```
4. 確認方向、Hall 順序、過流縮放。

完成標準：空載 ±60–100 rpm 平滑、無卡滯異響。

## Phase 2 — IMU

1. LSM6DSK320X I2C；`imu` / `status` 看 pitch。
2. 校準軸向與 `g_balance_theta_bias_rad`。
3. 目標靜止 pitch 噪訊儘量 < 0.3°（視機構）。

## Phase 3 — PID 原型（驗證符號）

```text
balmode pid
lqr on
```

扶持站立，確認：

- 前傾 → 輪往前加 τ（同向）
- 左右輪差速轉向符號
- `brake` 站定不瘋狂抖

**不要**在符號未驗證前直接猛調 LQR。

## Phase 4 — 參數辨識 + LQR

1. 量測／更新 `params/robot_params.yaml`（質量、COM、輪距、慣量、Kt、R）。
2. 產生增益：
   ```bash
   pip install -r tools/requirements.txt
   python3 tools/lqr_gain_gen.py
   ```
3. 重新編譯；`balmode lqr`。
4. 先空載／沙袋，再慢慢提高。

## Phase 5 — 接近載人（高風險）

必須具備：

- 硬體 POEG / E-stop（`safety_port_hw_shutdown`）
- UV/OV/OC/OT（`safety_update`）
- 傾角／IMU fault
- 輪滑與地面測試程序

載人測試不屬於一般 demo，需防護與急停人員。

## 架構口訣

```text
AI/RC 只下 vx,wz
RA8T2：LQR/PID → τ → Iq*
FOC：電流環 + Hall
安全：軟體 + POEG 硬關斷
```
